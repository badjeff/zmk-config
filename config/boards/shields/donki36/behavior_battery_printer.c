/*
 * Minimal ZMK behavior: types battery percentage with a label when activated.
 * Uses zmk_battery_state_of_charge().
 *
 * Source:
 * https://github.com/alan0ford/zmk-lplancks/blob/GHPilotBatt/boards/shields/lplancks/behavior_battery_printer.c
 *
 */

#define DT_DRV_COMPAT zmk_behavior_battery_printer

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/battery.h>
#include <dt-bindings/zmk/keys.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
#include <zmk/split/central.h>
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* increased buffer to contain prefix + digits + space + percent */
#define MAX_CHARS 32
#define TYPE_DELAY_MS 10

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_battery_printer_data {
    struct k_work_delayable typing_work;
    uint8_t chars[MAX_CHARS];
    uint8_t chars_len;
    uint8_t current_idx;
    bool key_pressed;
};

struct behavior_battery_printer_config {
    /* reserved for future options */
};

static inline void reset_typing_state(struct behavior_battery_printer_data *data) {
    data->current_idx = 0;
    data->key_pressed = false;
    data->chars_len = 0;
    memset(data->chars, 0, sizeof(data->chars));
}

/* explicit mapping for top-row digits (safe) */
static const uint32_t digit_keycodes[10] = {
    NUMBER_0, NUMBER_1, NUMBER_2, NUMBER_3, NUMBER_4,
    NUMBER_5, NUMBER_6, NUMBER_7, NUMBER_8, NUMBER_9
};

/* map an ASCII char used in our phrase to an encoded keycode understood by
 * raise_zmk_keycode_state_changed_from_encoded().
 *
 * Supports:
 *  - digits '0'..'9' -> top-row NUMBER_*
 *  - letters a..z / A..Z -> letter keycodes (uppercase uses LS(...) to require shift)
 *  - space ' ' -> SPACE
 *  - dot '.' -> DOT
 *  - percent '%' -> PERCENT (macro includes shift)
 */
static uint32_t char_to_encoded_keycode(uint8_t ch) {
    /* digits */
    if (ch >= '0' && ch <= '9') {
        return digit_keycodes[ch - '0'];
    }

    /* space */
    if (ch == ' ') {
        return SPACE;
    }

    /* dot/period */
    if (ch == '.') {
        return DOT;
    }

    /* percent sign (uses define that includes shift) */
    if (ch == '%') {
        return PERCENT;
    }

    return 0;
    // /* letters */
    // /* map lowercase and uppercase by explicit cases to use the key macros defined
    //  * in dt-bindings/zmk/keys.h. For uppercase, wrap with LS(...) so Shift is sent.
    //  *
    //  * Note: LS(...) and letter macros (A, B, C, ...) are available via keys headers.
    //  * If for some reason LS isn't defined in your build, we can instead press/release
    //  * an explicit SHIFT key around the letter; tell me if LS isn't available.
    //  */
    // bool upper = false;
    // if (ch >= 'A' && ch <= 'Z') {
    //     upper = true;
    //     ch = (uint8_t)(ch - 'A' + 'a'); /* normalize to lowercase for switch */
    // }

    // switch (ch) {
    // case 'a':
    //     return upper ? LS(A) : A;
    // case 'b':
    //     return upper ? LS(B) : B;
    // case 'c':
    //     return upper ? LS(C) : C;
    // case 'd':
    //     return upper ? LS(D) : D;
    // case 'e':
    //     return upper ? LS(E) : E;
    // case 'f':
    //     return upper ? LS(F) : F;
    // case 'g':
    //     return upper ? LS(G) : G;
    // case 'h':
    //     return upper ? LS(H) : H;
    // case 'i':
    //     return upper ? LS(I) : I;
    // case 'j':
    //     return upper ? LS(J) : J;
    // case 'k':
    //     return upper ? LS(K) : K;
    // case 'l':
    //     return upper ? LS(L) : L;
    // case 'm':
    //     return upper ? LS(M) : M;
    // case 'n':
    //     return upper ? LS(N) : N;
    // case 'o':
    //     return upper ? LS(O) : O;
    // case 'p':
    //     return upper ? LS(P) : P;
    // case 'q':
    //     return upper ? LS(Q) : Q;
    // case 'r':
    //     return upper ? LS(R) : R;
    // case 's':
    //     return upper ? LS(S) : S;
    // case 't':
    //     return upper ? LS(T) : T;
    // case 'u':
    //     return upper ? LS(U) : U;
    // case 'v':
    //     return upper ? LS(V) : V;
    // case 'w':
    //     return upper ? LS(W) : W;
    // case 'x':
    //     return upper ? LS(X) : X;
    // case 'y':
    //     return upper ? LS(Y) : Y;
    // case 'z':
    //     return upper ? LS(Z) : Z;
    // default:
    //     return 0;
    // }
}

static void send_key(struct behavior_battery_printer_data *data) {
    if (data->current_idx >= data->chars_len || data->current_idx >= ARRAY_SIZE(data->chars)) {
        reset_typing_state(data);
        return;
    }

    uint8_t ch = data->chars[data->current_idx];
    uint32_t keycode = char_to_encoded_keycode(ch);
    if (!keycode) {
        LOG_WRN("behavior_battery_printer: unsupported char '%c' (idx %u)", ch, data->current_idx);
        reset_typing_state(data);
        return;
    }

    bool pressed = !data->key_pressed;
    raise_zmk_keycode_state_changed_from_encoded(keycode, pressed, k_uptime_get());
    data->key_pressed = pressed;

    if (pressed) {
        /* schedule release */
        k_work_schedule(&data->typing_work, K_MSEC(TYPE_DELAY_MS));
        return;
    } else {
        /* released -> next char */
        data->current_idx++;
        if (data->current_idx < data->chars_len) {
            k_work_schedule(&data->typing_work, K_MSEC(TYPE_DELAY_MS));
        } else {
            reset_typing_state(data);
        }
    }
}

static void type_keys_work(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct behavior_battery_printer_data *data = CONTAINER_OF(dwork, struct behavior_battery_printer_data, typing_work);
    send_key(data);
}

static int behavior_battery_printer_init(const struct device *dev) {
    struct behavior_battery_printer_data *data = dev->data;
    k_work_init_delayable(&data->typing_work, type_keys_work);
    reset_typing_state(data);
    return 0;
}

/* convert small uint -> ascii digits (0..999) */
static void uint_to_chars(uint32_t v, uint8_t *buffer, uint8_t *len) {
    char tmp[4];
    int t = 0;
    if (v == 0) {
        buffer[0] = '0';
        *len = 1;
        return;
    }
    while (v > 0 && t < (int)sizeof(tmp)) {
        tmp[t++] = '0' + (v % 10);
        v /= 10;
    }
    for (int i = 0; i < t; i++) {
        buffer[i] = tmp[t - 1 - i];
    }
    *len = t;
}

/* on_pressed: build literal "Level Bat. " + digits + " %" and type it */
static int on_pressed(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_battery_printer_data *data = dev->data;

    if (data->key_pressed || data->current_idx) {
        /* typing in progress, ignore */
        return ZMK_BEHAVIOR_OPAQUE;
    }

    /* read battery percentage (0..100) from ZMK API */
    uint8_t percent = zmk_battery_state_of_charge();

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
#if IS_ENABLED(CONFIG_ZMK_SPLIT_BLE_CENTRAL_BATTERY_LEVEL_FETCHING)
    uint8_t source = event.source;
    if (source != 0xFF) {
        zmk_split_central_get_peripheral_battery_level(source, &percent);
    }
#endif
#endif

    if (percent > 100) percent = 100;

    LOG_INF("behavior_battery_printer: battery SOC read = %u%%", percent);

    reset_typing_state(data);

    /* prefix exactly as requested (capital L and B) */
    // const char *prefix = "Level Bat. ";
    // size_t p = strlen(prefix);

    // /* ensure buffer fits */
    // if (p + 4 + 3 >= ARRAY_SIZE(data->chars)) { /* prefix + up to 3 digits + space + percent */
    //     LOG_ERR("behavior_battery_printer: buffer too small for prefix");
    //     return ZMK_BEHAVIOR_OPAQUE;
    // }

    // /* copy prefix */
    // memcpy(data->chars, prefix, p);
    // data->chars_len = p;

    /* append digits of percent */
    uint8_t digitbuf[4];
    uint8_t digitlen = 0;
    uint_to_chars(percent, digitbuf, &digitlen);
    for (int i = 0; i < digitlen; i++) {
        data->chars[data->chars_len++] = digitbuf[i];
    }

    /* append space and percent sign */
    data->chars[data->chars_len++] = '%';
    data->chars[data->chars_len++] = ' ';

    /* start typing */
    data->current_idx = 0;
    data->key_pressed = false;
    send_key(data);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_battery_printer_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

#define BAT_INST(idx) \
    static struct behavior_battery_printer_data behavior_battery_printer_data_##idx; \
    static const struct behavior_battery_printer_config behavior_battery_printer_config_##idx = {}; \
    BEHAVIOR_DT_INST_DEFINE(idx, behavior_battery_printer_init, NULL, \
                            &behavior_battery_printer_data_##idx, \
                            &behavior_battery_printer_config_##idx, \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, \
                            &behavior_battery_printer_api);

DT_INST_FOREACH_STATUS_OKAY(BAT_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
