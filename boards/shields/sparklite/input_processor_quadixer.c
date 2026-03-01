/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_quadixer

#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

/*
 * 6DoF Joystick Input Processor (Quadixer)
 *
 * Fuses four PAT9125 optical sensors into a 6-axis joystick (x, y, z, rx, ry, rz).
 *
 * Hardware Configuration:
 * - Four PAT9125 sensor breakout boards mounted on central stem
 * - Sensor board bottom facing Z axis, beams emitting horizontally
 * - Sensor orientations (facing direction outward from center):
 *   - Alpha (idx 0): facing South (toward user), local X -> World X, local Y -> World Z
 *   - Beta (idx 1):  facing East,  local X -> World Y, local Y -> World Z
 *   - Gamma (idx 2): facing North, local X -> -World X, local Y -> World Z
 *   - Delta (idx 3): facing West,  local X -> -World Y, local Y -> World Z
 * - Opposite sensors: Alpha↔Gamma (South↔North), Beta↔Delta (East↔West)
 *
 * Coordinate System: Z-up (right-handed)
 *   +X: East
 *   +Y: North
 *   +Z: Up
 *
 * Sensor Local Coordinates:
 *   - Local X: points in direction sensor is facing
 *   - Local Y: points toward board bottom (world Z direction)
 *
 * Processing Pipeline:
 * 1. Cache sensor readings from all four chips until sync
 * 2. Transform chip-local X/Y to world coordinates using direct axis mapping
 * 3. Calculate centroid (x, y, z translation) from averaged sensor positions
 * 4. Calculate pitch/roll from Z-differentials of opposite sensors
 * 5. Calculate yaw from average cross product of radial and displacement vectors
 * 6. Report relative motion events at configured interval
 *
 * Rotation Calculations:
 *   - Pitch (around X): arcsin((z_alpha - z_delta) / radius)
 *   - Roll (around Y): arcsin((z_beta - z_gamma) / radius)
 *   - Yaw (around Z): avg(cross(radial, displacement)) / radius
 *
 * Chip mounting angles (counter-clockwise, for local-to-world transform):
 *   - Alpha: 0°
 *   - Beta: 90° (π/2)
 *   - Gamma: 180° (π)
 *   - Delta: 270° (3π/2)
 */

#define IDX_SEN_ALPHA   0
#define IDX_SEN_BETA    1
#define IDX_SEN_GAMMA   2
#define IDX_SEN_DELTA   3
#define NUM_SENSORS     4
#define MIN_SENSOR_NEUTRAL 2  /* Min sensors in neutral to trigger device neutral (1-4) */

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

/* 3D vector for intermediate calculations */
struct vec3 {
    float x, y, z;
};

struct zip_quadixer_config {
    uint32_t sync_report_ms;
    uint32_t sensor_radius_um;
    uint32_t tracker_radius_um;
    uint32_t xy_sensitivity_num;
    uint32_t xy_sensitivity_denom;
    uint32_t z_sensitivity_num;
    uint32_t z_sensitivity_denom;
    uint32_t pitch_scale_num;
    uint32_t pitch_scale_denom;
    uint32_t roll_scale_num;
    uint32_t roll_scale_denom;
    uint32_t yaw_scale_num;
    uint32_t yaw_scale_denom;
    uint32_t neutral_timeout_ms;
    uint8_t smooth_len;
    uint16_t rpt_dzn_x, rpt_dzn_y, rpt_dzn_z;
    uint16_t rpt_dzn_rx, rpt_dzn_ry, rpt_dzn_rz;
};

struct zip_quadixer_sensor_rel {
    int16_t x, y;
};

struct zip_quadixer_data {
    const struct device *dev;
    struct zip_quadixer_sensor_rel deltas[NUM_SENSORS];
    bool deltas_valid[NUM_SENSORS];
    int64_t last_rpt_time;

    /* Scheduled work for sending neutral report */
    struct k_work_delayable neutral_work;
    bool neutral_pending;
    bool in_neutral_state;

    /* Accumulated output values (scaled and ready to report) */
    int16_t x, y, z;        /* Translation: centroid of four sensor positions */
    int16_t rx, ry, rz;     /* Rotation: pitch (X), roll (Y), yaw (Z) in degrees */

    /* Velocity tracking for bounce detection (no smoothing - direct values) */
    int16_t velocity_tangent;
    int16_t velocity_radial;
    int16_t velocity_z;

    /* Motion state machine */
    enum zip_quadixer_motion_state {
        ZIP_QUADIXER_MOTION_STATE_USER_INPUT,
        ZIP_QUADIXER_MOTION_STATE_BOUNCING,
        ZIP_QUADIXER_MOTION_STATE_SETTLING,
        ZIP_QUADIXER_MOTION_STATE_NEUTRAL
    };

    enum zip_quadixer_motion_state motion_state;
};

/* Forward declaration */
static void neutral_work_handler(struct k_work *work);

static int quadixer_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state)
{
    const struct zip_quadixer_config *config = dev->config;
    struct zip_quadixer_data *data = dev->data;

    /* Validate sensor index */
    if (param1 >= NUM_SENSORS) {
        return ZMK_INPUT_PROC_STOP;
    }

    if (event->code == INPUT_REL_X) {
        data->deltas[param1].x += event->value;
    } else if (event->code == INPUT_REL_Y) {
        data->deltas[param1].y += event->value;
    }

    if (event->sync) {
        data->deltas_valid[param1] = true;
    }

    /* Process when all sensors have valid data for 6DoF calculations */
    if (
           !data->deltas_valid[IDX_SEN_ALPHA] || !data->deltas_valid[IDX_SEN_BETA]
        // || !data->deltas_valid[IDX_SEN_GAMMA] || !data->deltas_valid[IDX_SEN_DELTA]
    ) {
        return ZMK_INPUT_PROC_STOP;
    }

    /*
     * Transform chip-local readings to world coordinates.
     *
     * PAT9125 vertical mounting:
     * - Local X: tangential motion (perpendicular to radius)
     * - Local Y: vertical motion (world Z)
     *
     * Tangential direction at angle θ: (-sin(θ), cos(θ))
     */
    struct vec3 sensor_world[NUM_SENSORS];

    float sensor_radius_mm = config->sensor_radius_um / 1000.0f;
    float tracker_radius_mm = config->tracker_radius_um / 1000.0f;
    float xy_scale = (float)config->xy_sensitivity_num / (float)config->xy_sensitivity_denom;
    float z_scale = (float)config->z_sensitivity_num / (float)config->z_sensitivity_denom;
    float pitch_scale = (float)config->pitch_scale_num / (float)config->pitch_scale_denom;
    float roll_scale = (float)config->roll_scale_num / (float)config->roll_scale_denom;
    float yaw_scale = (float)config->yaw_scale_num / (float)config->yaw_scale_denom;

    for (int i = 0; i < NUM_SENSORS; i++) {
        float local_x = (float)data->deltas[i].x;  /* Sensor local X axis */
        float local_y = (float)data->deltas[i].y;  /* Sensor local Y axis -> World Z */

        /*
         * Direct axis mapping based on sensor facing direction:
         * - Alpha (South):  local X -> World X,  local Y -> World Z
         * - Beta (East):   local X -> World Y,  local Y -> World Z
         * - Gamma (North): local X -> -World X, local Y -> World Z
         * - Delta (West):  local X -> -World Y, local Y -> World Z
         */
        switch (i) {
        case IDX_SEN_ALPHA:
            sensor_world[i].x = local_x;
            sensor_world[i].y = 0;
            break;
        case IDX_SEN_BETA:
            sensor_world[i].x = 0;
            sensor_world[i].y = -local_x;
            break;
        case IDX_SEN_GAMMA:
            sensor_world[i].x = -local_x;
            sensor_world[i].y = 0;
            break;
        case IDX_SEN_DELTA:
            sensor_world[i].x = 0;
            sensor_world[i].y = local_x;
            break;
        }
        // sensor_world[i].z = local_y;
    }

    // LOG_DBG("sensor_world[ALPHA]: x=%.2f y=%.2f z=%.2f", sensor_world[IDX_SEN_ALPHA].x, sensor_world[IDX_SEN_ALPHA].y, sensor_world[IDX_SEN_ALPHA].z);
    // LOG_DBG("sensor_world[BETA]:  x=%.2f y=%.2f z=%.2f", sensor_world[IDX_SEN_BETA].x, sensor_world[IDX_SEN_BETA].y, sensor_world[IDX_SEN_BETA].z);
    // LOG_DBG("sensor_world[GAMMA]: x=%.2f y=%.2f z=%.2f", sensor_world[IDX_SEN_GAMMA].x, sensor_world[IDX_SEN_GAMMA].y, sensor_world[IDX_SEN_GAMMA].z);
    // LOG_DBG("sensor_world[DELTA]: x=%.2f y=%.2f z=%.2f", sensor_world[IDX_SEN_DELTA].x, sensor_world[IDX_SEN_DELTA].y, sensor_world[IDX_SEN_DELTA].z);

    /* ============================================================================
     * Calculate Centroid and Rotation (Pitch/Roll/Yaw)
     * ========================================================================= */
    float centroid_x = 0.0f;
    float centroid_y = 0.0f;
    float centroid_z = 0.0f;

    for (int i = 0; i < NUM_SENSORS; i++) {
        centroid_x += sensor_world[i].x;
        centroid_y += sensor_world[i].y;
        centroid_z += sensor_world[i].z;
    }
    centroid_x /= NUM_SENSORS;
    centroid_y /= NUM_SENSORS;
    centroid_z /= NUM_SENSORS;

    float z_alpha = sensor_world[IDX_SEN_ALPHA].z;
    float z_beta = sensor_world[IDX_SEN_BETA].z;
    float z_gamma = sensor_world[IDX_SEN_GAMMA].z;
    float z_delta = sensor_world[IDX_SEN_DELTA].z;

    float pitch_rad = asinf((z_alpha - z_delta) / tracker_radius_mm);
    float roll_rad = asinf((z_beta - z_gamma) / tracker_radius_mm);

    float cross_sum = 0.0f;
    for (int i = 0; i < NUM_SENSORS; i++) {
        float rx = sensor_world[i].x;
        float ry = sensor_world[i].y;
        float dx = sensor_world[i].x - centroid_x;
        float dy = sensor_world[i].y - centroid_y;
        cross_sum += (rx * dy - ry * dx);
    }
    float yaw_rad = cross_sum / (sensor_radius_mm * NUM_SENSORS);

    data->x = (int16_t)roundf(centroid_x * xy_scale);
    data->y = (int16_t)roundf(centroid_y * xy_scale);
    data->z = (int16_t)roundf(centroid_z * z_scale);
    data->rx = (int16_t)roundf(pitch_rad * RAD_TO_DEG * pitch_scale);
    data->ry = (int16_t)roundf(roll_rad * RAD_TO_DEG * roll_scale);
    data->rz = (int16_t)roundf(yaw_rad * RAD_TO_DEG * yaw_scale);


    /* Reset deltas valid for next sensor cycle */
    for (int i = 0; i < NUM_SENSORS; i++) {
        data->deltas_valid[i] = false;
    }

    event->value = 0;
    event->sync = false;


    LOG_DBG("x=%5d y=%5d z=%5d", data->x, data->y, data->z);



    // /* apply deadzones */
    // bool x_in_dz = abs(data->x) < (int)config->rpt_dzn_x;
    // bool y_in_dz = abs(data->y) < (int)config->rpt_dzn_y;
    // bool z_in_dz = abs(data->z) < (int)config->rpt_dzn_z;
    // bool rx_in_dz = abs(data->rx) < (int)config->rpt_dzn_rx;
    // bool ry_in_dz = abs(data->ry) < (int)config->rpt_dzn_ry;
    // bool rz_in_dz = abs(data->rz) < (int)config->rpt_dzn_rz;
    // bool in_deadzone = x_in_dz && y_in_dz && z_in_dz && rx_in_dz && ry_in_dz && rz_in_dz;
    // // LOG_DBG("%d %d %d %d %d %d", 
    // //     abs(data->x), abs(data->y), abs(data->z),
    // //     abs(data->rx), abs(data->ry), abs(data->rz));
    // if (in_deadzone) {
    //     if (!data->neutral_pending && !data->in_neutral_state) {
    //         k_work_schedule(&data->neutral_work, K_MSEC(0));
    //         data->neutral_pending = true;
    //     }
    //     return ZMK_INPUT_PROC_STOP;
    // }


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
    struct zip_quadixer_data *data = CONTAINER_OF(dwork, struct zip_quadixer_data, neutral_work);
    const struct device *dev = data->dev;
    const struct zip_quadixer_config *config = dev->config;

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
    // LOG_DBG("NEUTRAL report sent (all axes zero)");
}

static struct zmk_input_processor_driver_api quadixer_driver_api = {
    .handle_event = quadixer_handle_event,
};

static int quadixer_init(const struct device *dev)
{
    struct zip_quadixer_data *data = dev->data;
    const struct zip_quadixer_config *config = dev->config;
    data->dev = dev;

    /* Initialize work item for neutral timeout */
    k_work_init_delayable(&data->neutral_work, neutral_work_handler);
    data->neutral_pending = false;
    data->in_neutral_state = false;

    /* Initialize velocity tracking and motion state */
    data->velocity_tangent = 0;
    data->velocity_radial = 0;
    data->velocity_z = 0;
    data->motion_state = ZIP_QUADIXER_MOTION_STATE_NEUTRAL;

    /* Clear sensor deltas */
    for (int i = 0; i < NUM_SENSORS; i++) {
        data->deltas[i].x = 0;
        data->deltas[i].y = 0;
        data->deltas_valid[i] = false;
    }

    return 0;
}

#define QUADIXER_INST(n)                                                                         \
    static struct zip_quadixer_data data_##n = {};                                               \
    static struct zip_quadixer_config config_##n = {                                             \
        .sync_report_ms = DT_INST_PROP(n, sync_report_ms),                                       \
        .sensor_radius_um = DT_INST_PROP_OR(n, sensor_radius_um, 18500),                                       \
        .tracker_radius_um = DT_INST_PROP_OR(n, tracker_radius_um, 25000),                                       \
        .xy_sensitivity_num = DT_INST_PROP_OR(n, xy_sensitivity_num, 1),                         \
        .xy_sensitivity_denom = DT_INST_PROP_OR(n, xy_sensitivity_denom, 1),                     \
        .z_sensitivity_num = DT_INST_PROP_OR(n, z_sensitivity_num, 1),                           \
        .z_sensitivity_denom = DT_INST_PROP_OR(n, z_sensitivity_denom, 1),                       \
        .pitch_scale_num = DT_INST_PROP_OR(n, pitch_scale_num, 1),                               \
        .pitch_scale_denom = DT_INST_PROP_OR(n, pitch_scale_denom, 1),                           \
        .roll_scale_num = DT_INST_PROP_OR(n, roll_scale_num, 1),                                 \
        .roll_scale_denom = DT_INST_PROP_OR(n, roll_scale_denom, 1),                             \
        .yaw_scale_num = DT_INST_PROP_OR(n, yaw_scale_num, 1),                                   \
        .yaw_scale_denom = DT_INST_PROP_OR(n, yaw_scale_denom, 1),                               \
        .neutral_timeout_ms = DT_INST_PROP_OR(n, neutral_timeout_ms, 50),                        \
        .smooth_len = DT_INST_PROP(n, smooth_len),                                               \
        .rpt_dzn_x = DT_INST_PROP(n, rpt_dzn_x),                                                 \
        .rpt_dzn_y = DT_INST_PROP(n, rpt_dzn_y),                                                 \
        .rpt_dzn_z = DT_INST_PROP(n, rpt_dzn_z),                                                 \
        .rpt_dzn_rx = DT_INST_PROP(n, rpt_dzn_rx),                                               \
        .rpt_dzn_ry = DT_INST_PROP(n, rpt_dzn_ry),                                               \
        .rpt_dzn_rz = DT_INST_PROP(n, rpt_dzn_rz),                                               \
    };                                                                                           \
    DEVICE_DT_INST_DEFINE(n, &quadixer_init, NULL, &data_##n, &config_##n, POST_KERNEL,          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &quadixer_driver_api);

DT_INST_FOREACH_STATUS_OKAY(QUADIXER_INST)
