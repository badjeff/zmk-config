/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_cirque

#include <stdlib.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>
#include <zephyr/devicetree/gpio.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>

enum cirque_gesture_mode {
    CIRQUE_GESTURE_NONE = 0,
    CIRQUE_GESTURE_PENDING, // rim touch during the hold-on-edge window, not scroll yet
    CIRQUE_GESTURE_CURSOR,
    CIRQUE_GESTURE_SCROLL,
};

/*
 * Inertial cursor motion ("flick then coast").
 * Launch velocity and glide tick are both derived from a small history ring of
 * samples (x, y, ts) rather than more configured timings.
 */
#define CIRQUE_HISTORY_LEN 6      /* history ring length (samples of x/y/ts) */
#define CIRQUE_TICK_MIN_MS 4      /* clamp bounds for the glide tick derived from history */
#define CIRQUE_TICK_MAX_MS 40
#define CIRQUE_GLIDE_MAX_MS 2000  /* fail-safe cap so a glide can never run forever */
#define CIRQUE_VEL_CAP 16384      /* sanity cap on launch velocity, counts/s */
#define CIRQUE_MOV_SHIFT 8        /* fixed point shift for sub-count glide distance */
#define CIRQUE_MOV_MASK ((1 << CIRQUE_MOV_SHIFT) - 1)

struct zip_cirque_config {
    bool primary_tap_enable;
    bool secondary_tap_enable;
    uint32_t tap_max_ms;         // tap detection: max gesture duration
    uint32_t tap_max_drift;      // tap detection: max gesture movement per axis, counts
    uint32_t edge_scroll_margin_h_pct;
    uint32_t edge_scroll_margin_v_pct;
    uint32_t edge_scroll_segment;
    uint32_t edge_scroll_gesture_delay_ms;    // hold-on-edge: qualify rim touch as scroll
    uint32_t edge_scroll_gesture_delay_drift; // hold-on-edge: max drift during the hold
    uint32_t session_end_idle_ms;
    uint32_t sync_rel_xy_ms;
    uint32_t sync_rel_v_ms;
    bool inertial_enable;            // true = coast on non-tap cursor release (DT default)
                                     // set `inertial-disable` in DT to turn off
    uint32_t inertial_decay_pct;     // glide velocity decay per tick, percent
    int32_t inertial_start_velocity; // counts/s; 0 = coast on every non-tap release
    struct gpio_dt_spec feedback_gpio; // haptic pin, driven high while a pulse plays
    bool has_feedback_gpio;            // true when feedback-gpios is present in DT
    uint32_t feedback_edge_scroll_start_ms;   // pulse duration on scroll gesture recognised
    uint32_t feedback_edge_scroll_trigger_ms; // pulse duration on segment-crossing wheel tick
};

struct zip_cirque_sample {
    int16_t x, y;
    int64_t ts;
};

struct zip_cirque_data {
    const struct device *dev;
    int64_t last_sync_rel_xy_ts;
    int64_t last_sync_rel_v_ts;
    int16_t sx, sy, sz; // sampling coord
    int16_t hs;         // historial segment coord
    int16_t dx, dy;     // cursor pending delta
    int16_t dv;         // scroll wheel pending delta
    enum cirque_gesture_mode gesture_mode;
    struct k_work_delayable session_work; // session end and inertial glide tick
    struct k_work_delayable feedback_work; // ends the current haptic feedback pulse
    struct zip_cirque_sample gesture_start; // gesture start sample
    struct zip_cirque_sample gesture_last; // gesture last sample; x/y = -1 when not sampled
    /* inertial glide state (flick then coast, cursor mode only) */
    struct k_spinlock lock; // protects the glide handoff between input thread and worker
    struct zip_cirque_sample hist[CIRQUE_HISTORY_LEN]; // history ring of samples (x, y, ts)
    uint8_t hist_head, hist_len; // ring window start / number of valid samples
    bool gliding;
    int32_t gvx, gvy;    // glide velocity, counts/s
    int32_t accx, accy;  // accumulated sub-count glide distance (fixed point)
    uint16_t tick_ms;    // glide tick, derived from history at launch
    int64_t glide_start_ts;
};

/* X-axis value range: [0..2047] (left to right) */
/* Y-axis value range: [0..1535] (top to bottom) */
/* Z-axis value range: [0..63] (hover to touch down) */
/* Circle center for Cirque trackpad (2047 x 1535 range) */
static const int16_t cx = 1024;
static const int16_t cy = 768;
static const float pi = 3.14159265358979323846f;

static float cirque_ellipse_dist(const struct zip_cirque_config *config,
                                   int16_t x, int16_t y) {
    /*
     * Touch pad is a perfect circle but coordinates are reported in a rectangle.
     * The circular coordination of the center region maps to an ellipse in rectangular coordinates.
     * Detect center region by radius distance to the center of the circle touch pad
     * using an elliptical boundary; horizontal/vertical margins are percentages.
     */
    float margin_h_pct = (float)config->edge_scroll_margin_h_pct;
    float margin_v_pct = (float)config->edge_scroll_margin_v_pct;
    float a = MAX(1.0f, (float)cx * (1.0f - margin_h_pct * 0.01f));
    float b = MAX(1.0f, (float)cy * (1.0f - margin_v_pct * 0.01f));
    int16_t rx = x - cx;
    int16_t ry = y - cy;
    return ((float)rx * (float)rx) / (a * a) + ((float)ry * (float)ry) / (b * b);
}

static void cirque_hist_push(struct zip_cirque_data *data, int16_t x, int16_t y,
                             int64_t ts) {
    uint8_t idx = (uint8_t)((data->hist_head + data->hist_len) % CIRQUE_HISTORY_LEN);
    data->hist[idx].x = x;
    data->hist[idx].y = y;
    data->hist[idx].ts = ts;
    if (data->hist_len < CIRQUE_HISTORY_LEN) {
        data->hist_len++;
    } else {
        data->hist_head = (uint8_t)((data->hist_head + 1) % CIRQUE_HISTORY_LEN);
    }
}

static bool cirque_glide_launch(struct zip_cirque_data *data,
                                const struct zip_cirque_config *config) {
    if (!config->inertial_enable) {
        return false;
    }
    if (data->hist_len < 2) {
        return false;
    }

    uint8_t newest =
        (uint8_t)((data->hist_head + data->hist_len + CIRQUE_HISTORY_LEN - 1) % CIRQUE_HISTORY_LEN);

    /* glide tick: mean frame interval over the recorded window, clamped to sane bounds */
    int64_t span = data->hist[newest].ts - data->hist[data->hist_head].ts;
    if (span <= 0) {
        return false;
    }
    uint16_t tick =
        (uint16_t)CLAMP(span / (data->hist_len - 1), CIRQUE_TICK_MIN_MS, CIRQUE_TICK_MAX_MS);

    /* launch velocity from the newest interval only, so pre-flick
     * hesitation does not blunt it */
    uint8_t prev = (uint8_t)((newest + CIRQUE_HISTORY_LEN - 1) % CIRQUE_HISTORY_LEN);
    int64_t dt = data->hist[newest].ts - data->hist[prev].ts;
    if (dt < 1) {
        dt = 1;
    }
    int32_t vx = (int32_t)(((int64_t)(data->hist[newest].x - data->hist[prev].x) * 1000) / dt);
    int32_t vy = (int32_t)(((int64_t)(data->hist[newest].y - data->hist[prev].y) * 1000) / dt);

    /* flick threshold; 0 = coast on every non-tap release */
    if (config->inertial_start_velocity > 0 &&
        (int64_t)vx * vx + (int64_t)vy * vy <
            (int64_t)config->inertial_start_velocity * config->inertial_start_velocity) {
        return false;
    }

    /* the caller already ended the gesture; the tick now owns the cursor */
    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->gvx = CLAMP(vx, -CIRQUE_VEL_CAP, CIRQUE_VEL_CAP);
    data->gvy = CLAMP(vy, -CIRQUE_VEL_CAP, CIRQUE_VEL_CAP);
    data->accx = data->accy = 0;
    data->tick_ms = tick;
    data->glide_start_ts = k_uptime_get();
    data->gliding = true;
    k_spin_unlock(&data->lock, key);

    LOG_DBG("v: (%d, %d), tick: %u, span: %lld", vx, vy, tick, span);
    return true;
}

static void cirque_glide_tick(struct zip_cirque_data *data,
                              const struct zip_cirque_config *config) {
    k_spinlock_key_t key = k_spin_lock(&data->lock);

    if (!data->gliding) {
        /* cancelled by a new touch between reschedule and tick */
        k_spin_unlock(&data->lock, key);
        return;
    }

    /* geometric decay: velocity remains decay_pct per tick */
    data->gvx = (int32_t)((int64_t)data->gvx * (int64_t)config->inertial_decay_pct / 100);
    data->gvy = (int32_t)((int64_t)data->gvy * (int64_t)config->inertial_decay_pct / 100);

    /* distance for this tick; sub-count remainder carries over in fixed point */
    int32_t fx =
        data->accx + (int32_t)(((int64_t)data->gvx * (1 << CIRQUE_MOV_SHIFT) * data->tick_ms) / 1000);
    int32_t fy =
        data->accy + (int32_t)(((int64_t)data->gvy * (1 << CIRQUE_MOV_SHIFT) * data->tick_ms) / 1000);
    data->accx = fx & CIRQUE_MOV_MASK;
    data->accy = fy & CIRQUE_MOV_MASK;
    int16_t mx = (int16_t)(fx >> CIRQUE_MOV_SHIFT); // arithmetic shift: floor division
    int16_t my = (int16_t)(fy >> CIRQUE_MOV_SHIFT);
    bool have_mx = mx != 0;
    bool have_my = my != 0;

    /* stop when a whole tick moves less than one count, or on the fail-safe cap */
    if ((!have_mx && !have_my) || k_uptime_get() - data->glide_start_ts > CIRQUE_GLIDE_MAX_MS) {
        data->gliding = false;
        k_spin_unlock(&data->lock, key);
        LOG_DBG("GLIDE end. v: (%d, %d)", data->gvx, data->gvy);
        return;
    }
    k_spin_unlock(&data->lock, key);

    /* same emission pattern as the cursor path: sync flag on the last event */
    if (have_mx) {
        input_report(data->dev, INPUT_EV_REL, INPUT_REL_X, mx, !have_my, K_NO_WAIT);
    }
    if (have_my) {
        input_report(data->dev, INPUT_EV_REL, INPUT_REL_Y, my, true, K_NO_WAIT);
    }

    k_work_reschedule(&data->session_work, K_MSEC(data->tick_ms));
}

/* haptic feedback: one-shot pulse, pin driven high for duration_ms */
static void cirque_feedback_off(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct zip_cirque_data *data =
        CONTAINER_OF(d_work, struct zip_cirque_data, feedback_work);
    const struct zip_cirque_config *config = data->dev->config;

    gpio_pin_set_dt(&config->feedback_gpio, 0);
}

static void cirque_feedback_pulse(struct zip_cirque_data *data, uint32_t duration_ms) {
    const struct zip_cirque_config *config = data->dev->config;

    if (!config->has_feedback_gpio) {
        return;
    }
    gpio_pin_set_dt(&config->feedback_gpio, 1);
    k_work_reschedule(&data->feedback_work, K_MSEC(MAX(duration_ms, 1u)));
}

static void cirque_session_work(struct k_work *work) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct zip_cirque_data *data =
        CONTAINER_OF(d_work, struct zip_cirque_data, session_work);
    const struct device *dev = data->dev;
    const struct zip_cirque_config *config = dev->config;

    if (data->gliding) {
        /* inertia owns the cursor; the tick decides whether the work re-arms */
        cirque_glide_tick(data, config);
        return;
    }

    if (data->gesture_mode == CIRQUE_GESTURE_NONE) {
        /* spurious wake after a glide or a reset */
        return;
    }

    /* re-arm guard: the session must actually have gone idle; a frame that
     * raced the work reschedule must not end a live gesture */
    if (k_uptime_get() - data->gesture_last.ts < config->session_end_idle_ms) {
        k_work_reschedule(&data->session_work, K_MSEC(config->session_end_idle_ms));
        return;
    }

    bool is_a_tap = false;
    int64_t gt = data->gesture_last.ts - data->gesture_start.ts;
    int16_t gx = abs(data->gesture_start.x - data->gesture_last.x);
    int16_t gy = abs(data->gesture_start.y - data->gesture_last.y);
    // LOG_DBG("TAP? gt: %lld, gx: %d, gy: %d", gt, gx, gy);
    is_a_tap = (
        gt < config->tap_max_ms
        && gx < config->tap_max_drift
        && gy < config->tap_max_drift
    );

    bool cursor_session = data->gesture_mode == CIRQUE_GESTURE_CURSOR;

    data->gesture_mode = CIRQUE_GESTURE_NONE;
    data->hs = -1;
    data->gesture_last.x = data->gesture_last.y = -1;
    // LOG_DBG("");

    if (is_a_tap) {
        if (config->primary_tap_enable) {
            uint16_t btn = INPUT_BTN_TOUCH;
            if (config->secondary_tap_enable) {
                float d = cirque_ellipse_dist(config, data->gesture_start.x, data->gesture_start.y);
                if (d > 1.0f) {
                    btn = INPUT_BTN_1;
                }
            }
            // LOG_DBG("TAP: %f", d);
            input_report_key(data->dev, btn, true, true, K_FOREVER);
            k_sleep(K_MSEC(21));
            input_report_key(data->dev, btn, false, true, K_FOREVER);
        }
        return;
    }

    /* non-tap session ended (finger lifted): a cursor drag may coast */
    if (cursor_session && cirque_glide_launch(data, config)) {
        k_work_reschedule(&data->session_work, K_MSEC(data->tick_ms));
    }
}

static int zip_handle_event(const struct device *dev, struct input_event *event, uint32_t param1,
                           uint32_t param2, struct zmk_input_processor_state *state) {

    const struct zip_cirque_config *config = dev->config;
    struct zip_cirque_data *data = dev->data;

    /* fetch all axis values from input sub-sys */
    if (event->code == INPUT_ABS_X) data->sx = event->value;
    else if (event->code == INPUT_ABS_Y) data->sy = event->value;
    else if (event->code == INPUT_ABS_Z) data->sz = event->value;

    int64_t now = k_uptime_get();

    /* on all axis value ready */
    if (event->sync) {
        // LOG_DBG("Z: %d, X: %d, Y: %d", data->sz, data->sx, data->sy);

        if (data->gliding) {
            if (data->sz > 0) {
                /* finger re-contacted: cancel the glide, treat as a new session */
                k_spinlock_key_t key = k_spin_lock(&data->lock);
                data->gliding = false;
                k_spin_unlock(&data->lock, key);
            } else {
                /* lift frames while gliding: swallowed, the glide owns the cursor */
                event->value = 0;
                event->sync = false;
                return ZMK_INPUT_PROC_STOP; // 1;
            }
        }

        if (data->gesture_mode == CIRQUE_GESTURE_NONE) {
            float d = cirque_ellipse_dist(config, data->sx, data->sy);
            // LOG_DBG("d: %f, X: %d, Y: %d", d, data->sx, data->sy);
            if (d <= 1.0f) {
                /* touch on center region */
                data->gesture_mode = CIRQUE_GESTURE_CURSOR;
                // LOG_DBG("CURSOR");
            } else if (config->edge_scroll_gesture_delay_ms > 0) {
                /* touch on edge region: hold first, scroll after the delay */
                data->gesture_mode = CIRQUE_GESTURE_PENDING;
                LOG_DBG("PENDING");
            } else {
                /* touch on edge region (outside elliptical center region) */
                data->gesture_mode = CIRQUE_GESTURE_SCROLL;
                LOG_DBG("SCROLL");
                cirque_feedback_pulse(data, config->feedback_edge_scroll_start_ms);
            }
            if (data->gesture_mode != CIRQUE_GESTURE_NONE) {
                data->gesture_start.ts = now;
                data->gesture_start.x = data->sx;
                data->gesture_start.y = data->sy;
                data->hist_head = data->hist_len = 0;
            }
        }

        int16_t x = 0;
        int16_t y = 0;

        if (data->gesture_mode == CIRQUE_GESTURE_CURSOR) {
            /* convert to relative input */
            x = data->sx - (data->gesture_last.x >= 0 ? data->gesture_last.x : data->sx);
            y = data->sy - (data->gesture_last.y >= 0 ? data->gesture_last.y : data->sy);
            data->dx += x;
            data->dy += y;
            cirque_hist_push(data, data->sx, data->sy, now);
            data->gesture_last =
                (struct zip_cirque_sample){.x = data->sx, .y = data->sy, .ts = now};
            k_work_reschedule(&data->session_work, K_MSEC(config->session_end_idle_ms));
        }
        else if (data->gesture_mode == CIRQUE_GESTURE_PENDING) {
            /* hold-to-scroll: the rim touch must stay within the drift
             * threshold for the full delay before wheel output starts;
             * breaking the hold makes the touch silent until it comes
             * to rest and holds again */
            int16_t gx = abs(data->gesture_start.x - data->sx);
            int16_t gy = abs(data->gesture_start.y - data->sy);
            // LOG_DBG("HOLD? gx: %d, gy: %d", gx, gy);

            if (now - data->gesture_start.ts >= config->edge_scroll_gesture_delay_ms) {
                data->gesture_mode = CIRQUE_GESTURE_SCROLL;
                LOG_DBG("SCROLL");
                cirque_feedback_pulse(data, config->feedback_edge_scroll_start_ms);
            } else if (gx >= config->edge_scroll_gesture_delay_drift ||
                       gy >= config->edge_scroll_gesture_delay_drift) {
                /* finger moved during the hold */
                LOG_DBG("HOLD BROKEN on %lld ms", now - data->gesture_start.ts);
                if (config->has_feedback_gpio) {
                    /* haptics can announce scroll onset at the right moment:
                     * drop the attempt, hold again from rest */
                    data->gesture_mode = CIRQUE_GESTURE_NONE;
                }
                else if (cirque_ellipse_dist(config, data->sx, data->sy) <= 1.0f) {
                    /* no haptic cue, but the finger left the rim into the
                     * center region: the scroll attempt ends immediately */
                    data->gesture_mode = CIRQUE_GESTURE_NONE;
                }
                /* otherwise: stay in PENDING, the attempt is not
                 * abandoned and the gesture stays undetermined */
            }

            /* keep positions tracked while holding: no wheel jump on release */
            data->hs = -1;
            data->gesture_last =
                (struct zip_cirque_sample){.x = data->sx, .y = data->sy, .ts = now};
            k_work_reschedule(&data->session_work, K_MSEC(config->session_end_idle_ms));
        }
        else if (data->gesture_mode == CIRQUE_GESTURE_SCROLL) {
            /* convert to wheel input */
            x = data->sx - (data->gesture_last.x >= 0 ? data->gesture_last.x : data->sx);
            y = data->sy - (data->gesture_last.y >= 0 ? data->gesture_last.y : data->sy);
            int16_t v = 0;

            /* Split circle into segments */
            int n_sgmt = (int)config->edge_scroll_segment;
            if (n_sgmt >= 2) {

                /* Vector from center to current finger position */
                int16_t rx = data->sx - cx;
                int16_t ry = data->sy - cy;

                float angle = atan2f((float)ry, (float)rx);
                if (angle < 0.0f) {
                    angle += 2.0f * pi;
                }
                float sgmt_angle = (2.0f * pi) / (float)n_sgmt;
                int sgmt_idx = (int)(angle / sgmt_angle) % n_sgmt;

                if (data->hs != sgmt_idx) {
                    int diff = sgmt_idx - data->hs;
                    if      (diff > n_sgmt / 2)  diff -= n_sgmt;
                    else if (diff < -n_sgmt / 2) diff += n_sgmt;

                    // ignore first touch in scroll session
                    if (data->hs != -1) {
                        v += diff > 0 ? -1 : 1;
                        cirque_feedback_pulse(data, config->feedback_edge_scroll_trigger_ms);
                    }

                    data->hs = sgmt_idx;
                    LOG_DBG("seg: %d, v: %d", sgmt_idx, v);
                }
            }
            /* fallback to swipe up/down */
            else {
                int16_t tangential = x - y;
                if      (tangential > 3)    { v = 1; }
                else if (tangential < -3)   { v = -1; }
                else                        { v = 0; }
            }

            data->dv += v;
            data->gesture_last =
                (struct zip_cirque_sample){.x = data->sx, .y = data->sy, .ts = now};
            k_work_reschedule(&data->session_work, K_MSEC(config->session_end_idle_ms));
        }
        else if (data->gesture_mode == CIRQUE_GESTURE_NONE) {
            data->hs = -1;
            data->gesture_last.x = data->gesture_last.y = -1;
        }

    } /* if (event->sync) */
    else {
        event->value = 0;
        event->sync = false;
        return ZMK_INPUT_PROC_STOP; // 1;
    }

    if (now - data->last_sync_rel_xy_ts > config->sync_rel_xy_ms) {
        bool have_dx = data->dx != 0;
        bool have_dy = data->dy != 0;
        if (have_dx || have_dy) {
            // LOG_DBG("x: %d, y: %d", data->dx, data->dy);
            data->last_sync_rel_xy_ts = now;
            if (have_dx) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_X, data->dx, 
                    !have_dy, K_NO_WAIT);
            }
            if (have_dy) {
                input_report(dev, INPUT_EV_REL, INPUT_REL_Y, data->dy, 
                    true, K_NO_WAIT);
            }
            data->dx = data->dy = 0;
        }
    }

    if (now - data->last_sync_rel_v_ts > config->sync_rel_v_ms) {
        bool have_dv = data->dv != 0;
        if (have_dv) {
            // LOG_DBG("v: %d", data->dv > 0 ? 1 : -1);
            data->last_sync_rel_v_ts = now;
            input_report(dev, INPUT_EV_REL, INPUT_REL_WHEEL, 
                data->dv > 0 ? 1 : -1, true, K_NO_WAIT);
            data->dv = 0;
        }
    }

    event->value = 0;
    event->sync = false;
    return ZMK_INPUT_PROC_STOP; // 1;
}

static struct zmk_input_processor_driver_api zip_driver_api = {
    .handle_event = zip_handle_event,
};

static int zip_init(const struct device *dev) {
    struct zip_cirque_data *data = dev->data;
    const struct zip_cirque_config *config = dev->config;
    data->dev = dev;
    data->hs = -1;
    data->gesture_last.x = data->gesture_last.y = -1;
    data->dx = data->dy = data->dv = 0;
    data->gesture_mode = CIRQUE_GESTURE_NONE;
    data->hist_head = data->hist_len = 0;
    data->gliding = false;
    k_work_init_delayable(&data->session_work, cirque_session_work);
    k_work_init_delayable(&data->feedback_work, cirque_feedback_off);
    if (config->has_feedback_gpio) {
        if (!gpio_is_ready_dt(&config->feedback_gpio)) {
            LOG_ERR("feedback GPIO not ready");
            return -ENODEV;
        }
        int ret = gpio_pin_configure_dt(&config->feedback_gpio, GPIO_OUTPUT_LOW);
        if (ret < 0) {
            LOG_ERR("feedback GPIO configure failed: %d", ret);
            return ret;
        }
    }
    return 0;
}

#define ZIP_INST(n)                                                                       \
    static struct zip_cirque_data data_##n = {};                                          \
    static struct zip_cirque_config config_##n = {                                        \
        .primary_tap_enable = !DT_INST_PROP(n, primary_tap_disable),                      \
        .secondary_tap_enable = !DT_INST_PROP(n, secondary_tap_disable),                  \
        .tap_max_ms = DT_INST_PROP(n, tap_max_ms),                                        \
        .tap_max_drift = DT_INST_PROP(n, tap_max_drift),                                  \
        .edge_scroll_margin_h_pct = DT_INST_PROP(n, edge_scroll_margin_h_pct),            \
        .edge_scroll_margin_v_pct = DT_INST_PROP(n, edge_scroll_margin_v_pct),            \
        .edge_scroll_segment = DT_INST_PROP(n, edge_scroll_segment),                      \
        .edge_scroll_gesture_delay_ms =                                                   \
            DT_INST_PROP(n, edge_scroll_gesture_delay_ms),                                \
        .edge_scroll_gesture_delay_drift =                                                \
            DT_INST_PROP(n, edge_scroll_gesture_delay_drift),                             \
        .session_end_idle_ms = DT_INST_PROP(n, session_end_idle_ms),                      \
        .sync_rel_xy_ms = DT_INST_PROP(n, sync_rel_xy_ms),                                \
        .sync_rel_v_ms = DT_INST_PROP(n, sync_rel_v_ms),                                  \
        .inertial_enable = !DT_INST_PROP(n, inertial_disable),                            \
        .inertial_decay_pct = DT_INST_PROP(n, inertial_decay_pct),                        \
        .inertial_start_velocity = DT_INST_PROP(n, inertial_start_velocity),              \
        COND_CODE_1(DT_INST_NODE_HAS_PROP(n, feedback_gpios),                             \
            (.feedback_gpio = GPIO_DT_SPEC_INST_GET(n, feedback_gpios),                   \
             .has_feedback_gpio = true,),                                                 \
            ())                                                                           \
        .feedback_edge_scroll_start_ms =                                                  \
            DT_INST_PROP(n, feedback_edge_scroll_start_ms),                               \
        .feedback_edge_scroll_trigger_ms =                                                \
            DT_INST_PROP(n, feedback_edge_scroll_trigger_ms),                             \
    };                                                                                    \
    DEVICE_DT_INST_DEFINE(n, &zip_init, NULL, &data_##n, &config_##n, POST_KERNEL,        \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &zip_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ZIP_INST)
