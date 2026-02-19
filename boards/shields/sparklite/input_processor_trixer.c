/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_trixer

#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

/*
 * 6DoF Joystick Input Processor (Trixer)
 *
 * Fuses three MLX90393 magnetometers into a 6-axis joystick (x, y, z, rx, ry, rz).
 *
 * Hardware Configuration:
 * - Three magnetometer chips (MLX90393) on circular PCB at configured radius
 * - Chip Alpha (idx 0): 0° position (6 o'clock), die pin 1 at bottom right
 * - Chip Beta (idx 1): -120° position (2 o'clock), module edge faces outward
 * - Chip Gamma (idx 2): +120° position (10 o'clock), module edge faces outward
 * - Magnets glued on ring PCB, hovering 8mm above sensors
 *
 * Coordinate System: Z-up (right-handed, screen coords)
 *   +X: Right
 *   +Y: Down (screen coordinates, +Y is down for mouse)
 *   +Z: Up
 *
 * Processing Pipeline:
 * 1. Cache sensor readings from all three chips until sync
 * 2. Transform chip-local XY to world coordinates using rotation angles
 * 3. Calculate centroid (x, y, z translation) from averaged sensor positions
 * 4. Calculate Z-differentials for pitch/roll using geometric constraints
 * 5. Calculate yaw from XY displacement cross products with radial vectors
 * 6. Report relative motion events at configured interval
 *
 * Rotation Calculations:
 *   - Pitch (around X): arcsin((z_gamma - z_beta) / (radius * sqrt(3)))
 *   - Roll (around Y): arcsin((-z_alpha + z_beta + z_gamma) / (2 * radius))
 *   - Yaw (around Z): atan2(cross product of radial and displacement vectors) / radius
 *
 * Chip mounting angles (for local-to-world transform):
 *   - Alpha: 0°
 *   - Beta: -120° (-2π/3)
 *   - Gamma: +120° (+2π/3)
 */

#define IDX_SEN_ALPHA   0
#define IDX_SEN_BETA    1
#define IDX_SEN_GAMMA   2
#define NUM_SENSORS     3
#define MIN_SENSOR_NEUTRAL 1  /* Min sensors in neutral to trigger device neutral (1-3) */

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#define RAD_TO_DEG (180.0f / M_PI)
#define SQRT3 (1.732050808f)

static const float chip_angles_rad[NUM_SENSORS] = {
    [IDX_SEN_ALPHA] = 0.0f,
    [IDX_SEN_BETA]  = -2.0f * M_PI / 3.0f,
    [IDX_SEN_GAMMA] = 2.0f * M_PI / 3.0f
};

/* 3D vector for intermediate calculations */
struct vec3 {
    float x, y, z;
};

struct zip_trixer_config {
    uint32_t sync_report_ms;
    uint32_t radius_um;
    uint32_t xy_sensitivity_num;
    uint32_t xy_sensitivity_denom;
    uint32_t z_sensitivity_num;
    uint32_t z_sensitivity_denom;
    uint32_t rotation_scale_num;
    uint32_t rotation_scale_denom;
    uint32_t yaw_scale_num;
    uint32_t yaw_scale_denom;
    uint32_t neutral_timeout_ms;
};

struct zip_trixer_sensor_rel {
    int16_t x, y, z;
};

struct zip_trixer_data {
    const struct device *dev;
    struct zip_trixer_sensor_rel caches[NUM_SENSORS];
    bool caches_valid[NUM_SENSORS];
    int64_t last_rpt_time;

    /* Scheduled work for sending neutral report */
    struct k_work_delayable neutral_work;
    bool neutral_pending;
    bool in_neutral_state;

    /* Accumulated output values (scaled and ready to report) */
    int16_t x, y, z;        /* Translation: centroid of three magnet positions */
    int16_t rx, ry, rz;     /* Rotation: pitch (X), roll (Y), yaw (Z) in degrees */
};

/* Rotate 2D vector counter-clockwise by given angle (radians) */
static inline void rotate2d(float *x, float *y, float angle)
{
    float cos_a = cosf(angle);
    float sin_a = sinf(angle);
    float x_new = (*x) * cos_a - (*y) * sin_a;
    float y_new = (*x) * sin_a + (*y) * cos_a;
    *x = x_new;
    *y = y_new;
}

/* Forward declaration */
static void neutral_work_handler(struct k_work *work);

static int trixer_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state)
{
    const struct zip_trixer_config *config = dev->config;
    struct zip_trixer_data *data = dev->data;

    /* Validate sensor index */
    if (param1 >= NUM_SENSORS) {
        return ZMK_INPUT_PROC_STOP;
    }

    /* Store sensor reading in cache */
    if (event->code == INPUT_REL_X) {
        data->caches[param1].x = event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->caches[param1].y = event->value;
    } else if (event->code == INPUT_REL_Z) {
        data->caches[param1].z = event->value;
    }
    
    if (event->sync) {
        data->caches_valid[param1] = true;
    }

    /*
     * Count sensors in neutral (reporting zero) and check for any motion.
     * Device enters neutral when at least MIN_SENSOR_NEUTRAL sensors are idle.
     * Any motion cancels pending neutral and exits neutral state immediately.
     * This check happens before requiring all sensors to allow partial neutral detection.
     */
    int neutral_count = 0;
    bool any_motion = false;
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (data->caches[i].x != 0 || data->caches[i].y != 0 || data->caches[i].z != 0) {
            any_motion = true;
        } else {
            neutral_count++;
        }
    }

    if (any_motion) {
        /* Cancel any pending neutral timeout */
        if (data->neutral_pending) {
            k_work_cancel_delayable(&data->neutral_work);
            data->neutral_pending = false;
        }

        /* Exit neutral state if we were in it */
        if (data->in_neutral_state) {
            LOG_DBG("NEUTRAL exited (motion detected)");
            data->in_neutral_state = false;
        }
    } else if (neutral_count >= MIN_SENSOR_NEUTRAL) {
        /*
         * Sufficient sensors in neutral - schedule neutral report after timeout.
         * This handles the case where sensors enter deadzone at different times
         * by waiting for the full timeout period after the last sync.
         */
        if (!data->neutral_pending && !data->in_neutral_state) {
            k_work_schedule(&data->neutral_work, K_MSEC(config->neutral_timeout_ms));
            data->neutral_pending = true;
        }

        /* Clear caches and wait for next cycle or timeout */
        for (int i = 0; i < NUM_SENSORS; i++) {
            data->caches_valid[i] = false;
            data->caches[i].x = 0;
            data->caches[i].y = 0;
            data->caches[i].z = 0;
        }
        return ZMK_INPUT_PROC_STOP;
    }

    /* Process when all sensors have valid data for 6DoF calculations */
    if (!data->caches_valid[IDX_SEN_ALPHA] ||
        !data->caches_valid[IDX_SEN_BETA] ||
        !data->caches_valid[IDX_SEN_GAMMA]) {
        return ZMK_INPUT_PROC_STOP;
    }

    /* Transform chip-local readings to world coordinates */
    struct vec3 magnet_world[NUM_SENSORS];  /* XY in world frame, Z remains local */
    float z_mm[NUM_SENSORS];                 /* Z scaled to millimeters */
    
    float radius_mm = config->radius_um / 1000.0f;
    float xy_scale = (float)config->xy_sensitivity_num / (float)config->xy_sensitivity_denom;
    float z_scale = (float)config->z_sensitivity_num / (float)config->z_sensitivity_denom;
    float rot_scale = (float)config->rotation_scale_num / (float)config->rotation_scale_denom;
    float yaw_scale = (float)config->yaw_scale_num / (float)config->yaw_scale_denom;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        float local_x = (float)data->caches[i].x;
        float local_y = (float)data->caches[i].y;
        float local_z = (float)data->caches[i].z;
        
        float world_x = local_x;
        float world_y = local_y;
        rotate2d(&world_x, &world_y, chip_angles_rad[i]);
        
        magnet_world[i].x = world_x;
        magnet_world[i].y = world_y;
        magnet_world[i].z = local_z;
        
        z_mm[i] = local_z * z_scale;
    }

    struct vec3 centroid;
    centroid.x = (magnet_world[0].x + magnet_world[1].x + magnet_world[2].x) / 3.0f;
    centroid.y = (magnet_world[0].y + magnet_world[1].y + magnet_world[2].y) / 3.0f;
    centroid.z = (z_mm[0] + z_mm[1] + z_mm[2]) / 3.0f;

    data->x = (int16_t)roundf(centroid.x * xy_scale);
    data->y = (int16_t)roundf(centroid.y * xy_scale);
    data->z = (int16_t)roundf(centroid.z);

    float z_alpha_rel = z_mm[IDX_SEN_ALPHA] - centroid.z;
    float z_beta_rel = z_mm[IDX_SEN_BETA] - centroid.z;
    float z_gamma_rel = z_mm[IDX_SEN_GAMMA] - centroid.z;

    float sin_pitch = (z_gamma_rel - z_beta_rel) / (radius_mm * SQRT3);
    float sin_roll = (-z_alpha_rel + z_beta_rel + z_gamma_rel) / (2.0f * radius_mm);

    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    if (sin_roll > 1.0f) sin_roll = 1.0f;
    if (sin_roll < -1.0f) sin_roll = -1.0f;

    float pitch_rad = asinf(sin_pitch);
    float roll_rad = asinf(sin_roll);

    /*
     * Yaw calculation using cross product method.
     *
     * Yaw rotation moves each magnet tangentially around the Z axis. The tangential
     * displacement is measured by cross(radial_vector, displacement_vector), which
     * gives radius * yaw_angle for pure rotation while canceling pure translation.
     *
     * Radial vectors (unit vectors from center to each chip position):
     *   - Alpha (0°):       (0, 1)
     *   - Beta (-120°):     (-√3/2, -1/2)
     *   - Gamma (+120°):    (√3/2, -1/2)
     *
     * For each sensor: cross_z = radial_x * dy - radial_y * dx
     * Then: yaw = avg(cross_z) / radius
     */

    /* XY displacements from calibrated origin */
    float dx_alpha = magnet_world[IDX_SEN_ALPHA].x;
    float dy_alpha = magnet_world[IDX_SEN_ALPHA].y;
    float dx_beta = magnet_world[IDX_SEN_BETA].x;
    float dy_beta = magnet_world[IDX_SEN_BETA].y;
    float dx_gamma = magnet_world[IDX_SEN_GAMMA].x;
    float dy_gamma = magnet_world[IDX_SEN_GAMMA].y;

    /* Cross products: radial × displacement (Z-component only) */
    float cross_alpha = 0.0f * dy_alpha - 1.0f * dx_alpha;                  /* Alpha: (0, 1) */
    float cross_beta = (-SQRT3 / 2.0f) * dy_beta - (-0.5f) * dx_beta;       /* Beta: (-√3/2, -1/2) */
    float cross_gamma = (SQRT3 / 2.0f) * dy_gamma - (-0.5f) * dx_gamma;     /* Gamma: (√3/2, -1/2) */

    /* Average tangential displacement and convert to yaw angle */
    float avg_cross = (cross_alpha + cross_beta + cross_gamma) / 3.0f;
    float yaw_rad = avg_cross / radius_mm;

    /* Clamp to [-π, π] range */
    if (yaw_rad > M_PI) yaw_rad = M_PI;
    if (yaw_rad < -M_PI) yaw_rad = -M_PI;

    data->rx = (int16_t)roundf(pitch_rad * RAD_TO_DEG * rot_scale);
    data->ry = (int16_t)roundf(roll_rad * RAD_TO_DEG * rot_scale);
    data->rz = (int16_t)roundf(yaw_rad * RAD_TO_DEG * yaw_scale);

    /* Reset caches for next sensor cycle */
    for (int i = 0; i < NUM_SENSORS; i++) {
        data->caches_valid[i] = false;
        data->caches[i].x = 0;
        data->caches[i].y = 0;
        data->caches[i].z = 0;
    }

    event->value = 0;
    event->sync = false;

    int64_t now = k_uptime_get();

    /* Report accumulated values at configured interval */
    if (now - data->last_rpt_time >= config->sync_report_ms) {
        bool have_pos = data->x != 0 || data->y != 0 || data->z != 0;
        bool have_rot = data->rx != 0 || data->ry != 0 || data->rz != 0;
        
        if (have_pos || have_rot) {
            data->last_rpt_time = now;
            
            /* Report translation (X, Y, Z) - sync on last axis if no rotation */
            if (data->x != 0) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->x,
                           data->y == 0 && data->z == 0 && !have_rot, K_NO_WAIT);
            }
            if (data->y != 0) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->y,
                           data->z == 0 && !have_rot, K_NO_WAIT);
            }
            if (data->z != 0) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Z, data->z,
                           !have_rot, K_NO_WAIT);
            }

            /* Report rotation (RX, RY, RZ) - sync on RZ */
            if (data->rx != 0) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_RX, data->rx,
                           data->ry == 0 && data->rz == 0, K_NO_WAIT);
            }
            if (data->ry != 0) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_RY, data->ry,
                           data->rz == 0, K_NO_WAIT);
            }
            if (data->rz != 0) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_RZ, data->rz,
                           true, K_NO_WAIT);
            }

            /* Clear reported values */
            data->x = data->y = data->z = 0;
            data->rx = data->ry = data->rz = 0;
        }
    }

    return ZMK_INPUT_PROC_STOP;
}

/*
 * Work handler for sending neutral (zero) report.
 * Called when the neutral timeout expires - all sensors have been idle.
 */
static void neutral_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct zip_trixer_data *data = CONTAINER_OF(dwork, struct zip_trixer_data, neutral_work);
    const struct device *dev = data->dev;

    /* Mark neutral as no longer pending but we are now in neutral state */
    data->neutral_pending = false;
    data->in_neutral_state = true;

    /* Send zero on all 6 axes to HID joystick */
    input_report(dev, INPUT_EV_REL, INPUT_REL_X, 0, false, K_NO_WAIT);
    input_report(dev, INPUT_EV_REL, INPUT_REL_Y, 0, false, K_NO_WAIT);
    input_report(dev, INPUT_EV_REL, INPUT_REL_Z, 0, false, K_NO_WAIT);
    input_report(dev, INPUT_EV_REL, INPUT_REL_RX, 0, false, K_NO_WAIT);
    input_report(dev, INPUT_EV_REL, INPUT_REL_RY, 0, false, K_NO_WAIT);
    input_report(dev, INPUT_EV_REL, INPUT_REL_RZ, 0, true, K_NO_WAIT);
    
    LOG_DBG("NEUTRAL report sent (all axes zero)");
}

static struct zmk_input_processor_driver_api trixer_driver_api = {
    .handle_event = trixer_handle_event,
};

static int trixer_init(const struct device *dev)
{
    struct zip_trixer_data *data = dev->data;
    data->dev = dev;

    /* Initialize work item for neutral timeout */
    k_work_init_delayable(&data->neutral_work, neutral_work_handler);
    data->neutral_pending = false;
    data->in_neutral_state = false;

    /* Clear sensor caches */
    for (int i = 0; i < NUM_SENSORS; i++) {
        data->caches[i].x = 0;
        data->caches[i].y = 0;
        data->caches[i].z = 0;
        data->caches_valid[i] = false;
    }

    return 0;
}

#define TRIXER_INST(n)                                                                         \
    static struct zip_trixer_data data_##n = {};                                               \
    static struct zip_trixer_config config_##n = {                                             \
        .sync_report_ms = DT_INST_PROP(n, sync_report_ms),                                     \
        .radius_um = DT_INST_PROP_OR(n, radius_um, 18500),                                     \
        .xy_sensitivity_num = DT_INST_PROP_OR(n, xy_sensitivity_num, 1),                       \
        .xy_sensitivity_denom = DT_INST_PROP_OR(n, xy_sensitivity_denom, 1),                   \
        .z_sensitivity_num = DT_INST_PROP_OR(n, z_sensitivity_num, 1),                         \
        .z_sensitivity_denom = DT_INST_PROP_OR(n, z_sensitivity_denom, 1),                     \
        .rotation_scale_num = DT_INST_PROP_OR(n, rotation_scale_num, 1),                       \
        .rotation_scale_denom = DT_INST_PROP_OR(n, rotation_scale_denom, 1),                   \
        .yaw_scale_num = DT_INST_PROP_OR(n, yaw_scale_num, 1),                                 \
        .yaw_scale_denom = DT_INST_PROP_OR(n, yaw_scale_denom, 1),                             \
        .neutral_timeout_ms = DT_INST_PROP_OR(n, neutral_timeout_ms, 50),                      \
    };                                                                                         \
    DEVICE_DT_INST_DEFINE(n, &trixer_init, NULL, &data_##n, &config_##n, POST_KERNEL,          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &trixer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TRIXER_INST)
