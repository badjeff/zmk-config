/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

/*
Initial log should look like below if initialization success.

[00:00:00.629,150] <inf> input_mlx90393: Initializing MLX90393 at address 0x0C
[00:00:00.629,730] <inf> input_mlx90393: exit status: 0x02
[00:00:00.630,310] <inf> input_mlx90393: reset status: 0x06
[00:00:00.631,195] <inf> input_mlx90393: read 0x00 config status: 0x02
[00:00:00.632,080] <inf> input_mlx90393: read 0x02 config status: 0x02
[00:00:01.032,257] <inf> input_mlx90393: woc threshold: XY:0 Z:0
[00:00:01.033,142] <inf> input_mlx90393: read WOXY_THRESHOLD status: 0x02
[00:00:01.034,027] <inf> input_mlx90393: read WOZ_THRESHOLD status: 0x02
[00:00:01.034,912] <inf> input_mlx90393: read 0x01 config status: 0x02
[00:00:01.035,491] <inf> input_mlx90393: wake-on change mode status: 0x42
[00:00:01.035,522] <inf> input_mlx90393: MLX90393 input driver initialized successfully
*/

#define DT_DRV_COMPAT melexis_mlx90393_input

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/hid.h>
#include <zmk/endpoints.h>
#include <stdlib.h>
#include <stdbool.h>

LOG_MODULE_REGISTER(input_mlx90393, CONFIG_ZMK_LOG_LEVEL);

#define MLX90383_STATUS_ERR_BITS (BIT(4) | BIT(3))

struct mlx90393_config {
    union {
        struct i2c_dt_spec i2c;
        struct spi_dt_spec spi;
    } bus;
	const struct gpio_dt_spec irq_gpio;
    uint16_t calib_cycle;
    uint16_t woc_thd_xy, woc_thd_z;
    uint16_t rpt_dzn_x, rpt_dzn_y, rpt_dzn_z;
};

struct mlx90393_data {
    const struct device *dev;
    struct gpio_callback irq_gpio_cb;
    struct k_work trigger_work;
    struct k_work read_work;
    bool calibrated;
    uint16_t calibra_cnt;
    int16_t org_x, org_y, org_z;
    int32_t sum_x, sum_y, sum_z;
    int32_t min_x, min_y, min_z;
    int32_t max_x, max_y, max_z;
    int16_t thd_xy; // 0x07h: WOXY_THRESHOLD [15:0] (signed)
    int16_t thd_z;  // 0x08h: WOZ_THRESHOLD [15:0] (signed)
};

static int mlx90393_cmd_read(const struct device *dev,
                             const uint8_t *cmd, size_t cmd_len,
                             uint8_t *resp, size_t resp_len) {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

    const struct mlx90393_config *config = dev->config;
    int ret;
    ret = i2c_write_dt(&config->bus.i2c, cmd, cmd_len);
    if (!ret) {
        ret = i2c_read_dt(&config->bus.i2c, resp, resp_len);
    }
    return ret;

#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

    const struct mlx90393_config *config = dev->config;
    int err = 0;
    size_t total = cmd_len + resp_len;
    uint8_t tx_total[total];
    uint8_t rx_total[total];
    memcpy(tx_total, cmd, cmd_len);
    memset(tx_total + cmd_len, 0x00, resp_len);
    const struct spi_buf tx_buf = { .buf = tx_total, .len = total };
    const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    const struct spi_buf rx_buf = { .buf = rx_total, .len = total };
    const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };
    err = spi_transceive_dt(&config->bus.spi, &tx_set, &rx_set);
    if (err) {
        LOG_ERR("SPI transceive failed: %d", err);
        return err;
    }
    // LOG_HEXDUMP_DBG(rx_total, total, "spi rx_total");
    memcpy(resp, rx_total + cmd_len, resp_len);
    return 0;

#else
    return -ENOTSUP;
#endif
}

static void mlx90393_set_interrupt(const struct device *dev, const bool en) {
    const struct mlx90393_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Cannot set interrupt: %s", en ? "ENABLE" : "DISABLE");
    }
}

static int mlx90393_reset(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    // struct mlx90393_data *data = dev->data;
    uint8_t cmd;
    uint8_t status;
    int ret;

    cmd = 0xF0; // Reset
    ret = mlx90393_cmd_read(dev, &cmd, 1, &status, 1);

    if (ret < 0) {
        LOG_WRN("Failed to read reset status: 0x%02X", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read eset status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("reset status: 0x%02x", status);

    return ret;
}

static int mlx90393_exit_burst_mode(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    // struct mlx90393_data *data = dev->data;
    uint8_t cmd;
    uint8_t status;
    int ret;

    cmd = 0x80; // Exit mode
    ret = mlx90393_cmd_read(dev, &cmd, 1, &status, 1);

    if (ret < 0) {
        LOG_WRN("Failed to read exit status: 0x%02X", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read exit status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("exit status: 0x%02x", status);

    return ret;
}

static int mlx90393_start_woc_mode(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    // struct mlx90393_data *data = dev->data;
    uint8_t cmd;
    uint8_t status;
    int ret;

    cmd = 0x2E; // Start Wake-up on Change Mode for zyx
    ret = mlx90393_cmd_read(dev, &cmd, 1, &status, 1);

    if (ret < 0) {
        LOG_WRN("Failed to read wake-on change mode status: %d", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read wake-on change mode: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("wake-on change mode status: 0x%02x", status);

    return ret;
}

static int mlx90393_set_config(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    // struct mlx90393_data *data = dev->data;
    uint8_t reg_config[4];
    uint8_t status;
    int ret;

    reg_config[0] = 0x60;  // Write command
    reg_config[1] = 0x00;  // AH: BIST disabled
    reg_config[2] = 0x1C;  // AL: GAIN_SEL = 1, Hall plate spinning rate = DEFAULT(0xC)
    reg_config[3] = 0x00 << 2;  // Select address register
    ret = mlx90393_cmd_read(dev, reg_config, 4, &status, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read 0x00 config status: %d", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read 0x00 config status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("read 0x00 config status: 0x%02x", status);

    reg_config[0] = 0x60;  // Write register command
    reg_config[1] = 0x02;  // AH: 0x02
    reg_config[2] = 0xB4;  // AL: 0xB4, RES for magnetic measurement = 0
    reg_config[3] = 0x02 << 2;  // Select address register, (0x02 << 2)
    ret = mlx90393_cmd_read(dev, reg_config, 4, &status, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read 0x02 config status: %d", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read 0x02 config status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("read 0x02 config status: 0x%02x", status);

    return ret;
}

static int mlx90393_set_woc_threshold(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    struct mlx90393_data *data = dev->data;

    LOG_INF("setting woc threshold xy:%d z:%d", data->thd_xy, data->thd_z);
    uint8_t reg_config[4];
    uint8_t status;
    int ret;

    reg_config[0] = 0x60;  // Write command
    reg_config[1] = (data->thd_xy & 0xFF00) >> 8;   // Data [15:8]
    reg_config[2] = (data->thd_xy & 0x00FF);        // Data [7:0]
    reg_config[3] = 0x07 << 2;  // Select address register, WOXY_THRESHOLD
    ret = mlx90393_cmd_read(dev, reg_config, 4, &status, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read WOXY_THRESHOLD status: %d", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read WOXY_THRESHOLD status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("read WOXY_THRESHOLD status: 0x%02x", status);

    reg_config[0] = 0x60;  // Write command
    reg_config[1] = (data->thd_z & 0xFF00) >> 8;    // Data [15:8]
    reg_config[2] = (data->thd_z & 0x00FF);         // Data [7:0]
    reg_config[3] = 0x08 << 2;  // Select address register, WOZ_THRESHOLD
    ret = mlx90393_cmd_read(dev, reg_config, 4, &status, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read WOZ_THRESHOLD status: %d", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read WOZ_THRESHOLD status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("read WOZ_THRESHOLD status: 0x%02x", status);

    reg_config[0] = 0x60;  // Write register command
    reg_config[1] = 0x10;  // AH: WOC_DIFF
    reg_config[2] = 0x00;  // AL: BURST_DATA_RATE = DEFAULT (0x0)
    reg_config[3] = 0x01 << 2;  // Select address register, (0x02 << 2)
    ret = mlx90393_cmd_read(dev, reg_config, 4, &status, 1);

    if (ret < 0) {
        LOG_ERR("Failed to read 0x01 config status: %d", ret);
        return ret;
    }
    else if (status & MLX90383_STATUS_ERR_BITS) {
        LOG_ERR("read 0x01 config status: 0x%02X", status);
        return -EINVAL;
    }
    LOG_INF("read 0x01 config status: 0x%02x", status);

    return ret;
}

static void mlx90393_work_handler(struct k_work *work) {
    struct mlx90393_data *data = CONTAINER_OF(work, struct mlx90393_data, read_work);
    const struct device *dev = data->dev;
    const struct mlx90393_config *config = dev->config;
    
    uint8_t cmd;
    uint8_t read_data[7];
    int ret;
    int16_t x, y, z;

    // Read measurement after conversion time
    cmd = 0x4E;
    ret = mlx90393_cmd_read(dev, &cmd, 1, read_data, 7);

    if (ret < 0) {
        LOG_ERR("Failed to read measurement data: %d", ret);
        goto reenable_irq;
    }
    // Convert the data (big endian MSB:LSB for each axis)
    x = (int16_t)sys_get_be16(&read_data[1]);
    y = (int16_t)sys_get_be16(&read_data[3]);
    z = (int16_t)sys_get_be16(&read_data[5]);
    
    if (!data->calibrated) {
        data->sum_x += x;
        data->sum_y += y; 
        data->sum_z += z;
        data->min_x = MIN(data->min_x, x);
        data->min_y = MIN(data->min_y, y);
        data->min_z = MIN(data->min_z, z);
        data->max_x = MAX(data->max_x, x);
        data->max_y = MAX(data->max_y, y);
        data->max_z = MAX(data->max_z, z);
        data->calibra_cnt++;

        if ((data->calibra_cnt % config->calib_cycle) == 0) {
            LOG_INF("Calibration %d/50 - Current: X:%d Y:%d Z:%d", data->calibra_cnt, x, y, z);
        }

        if (data->calibra_cnt >= config->calib_cycle) {
            data->org_x = data->sum_x / config->calib_cycle;
            data->org_y = data->sum_y / config->calib_cycle;
            data->org_z = data->sum_z / config->calib_cycle;
            LOG_INF("calibrated org: x=%6d y=%6d z=%6d", data->org_x, data->org_y, data->org_z);

            data->thd_xy = MAX(data->max_x - data->min_x, data->max_y - data->min_y);
            data->thd_z = (data->max_z - data->min_z);
            LOG_INF("min/max: X:%6d/%6d Y:%6d/%6d Z:%6d/%6d", 
                    (int)data->min_x, (int)data->max_x, 
                    (int)data->min_y, (int)data->max_y, 
                    (int)data->min_z, (int)data->max_z);

            //
            // trim woc threshold to 2/3 of calibrated result
            // dealing with 3 facts: noise reduction, resolution, power efficiency
            //
            data->thd_xy = ( data->thd_xy * 2/3 ) + config->woc_thd_xy;
            data->thd_z = ( data->thd_z * 2/3 ) + config->woc_thd_z;
            LOG_INF("thd xy:%6d z:%6d", data->thd_xy, data->thd_z);

            data->calibrated = true;

            ret = mlx90393_exit_burst_mode(dev);
            if (ret < 0) {
                LOG_ERR("Failed to exit burst mode: %d", ret);
                goto reenable_irq;
            }

            k_msleep(100);

            ret = mlx90393_set_woc_threshold(dev);
            if (ret < 0) {
                LOG_ERR("Failed to set woc threshold: %d", ret);
                goto reenable_irq;
            }

            ret = mlx90393_start_woc_mode(dev);
            if (ret < 0) {
                LOG_ERR("Failed to start wake-on change mode: %d", ret);
                goto reenable_irq;
            }

        }
        goto reenable_irq; // Skip input during calibration
    }

    int16_t crd_x = x - data->org_x;
    int16_t crd_y = y - data->org_y;
    int16_t crd_z = z - data->org_z;
    
    if (abs(crd_x) < (int)config->rpt_dzn_x) crd_x = 0;
    if (abs(crd_y) < (int)config->rpt_dzn_y) crd_y = 0;
    if (abs(crd_z) < (int)config->rpt_dzn_z) crd_z = 0;

    // LOG_DBG("raw: X:%6d Y:%6d Z:%6d | origin: X:%6d Y:%6d Z:%6d | crd: X:%6d Y:%6d Z:%6d", 
    //         x, y, z, data->org_x, data->org_y, data->org_z, crd_x, crd_y, crd_z);
    // LOG_DBG("min/max: X:%6d/%6d Y:%6d/%6d Z:%6d/%6d", 
    //         (int)data->min_x, (int)data->max_x, (int)data->min_y, (int)data->max_y, 
    //         (int)data->min_z, (int)data->max_z);
    // LOG_DBG("thd xy:%6d z:%6d", data->thd_xy, data->thd_z);

    if (crd_x || crd_y || crd_z) {
        if (crd_x) {
            input_report_rel(dev, INPUT_ABS_X, crd_x, !crd_y && !crd_z, K_FOREVER);
        }
        if (crd_y) {
            input_report_rel(dev, INPUT_ABS_Y, crd_y, !crd_z, K_FOREVER);
        }
        if (crd_z) {
            input_report_rel(dev, INPUT_ABS_Z, crd_z, true, K_FOREVER);
        }
        LOG_DBG("crd x/y/z: %6d / %6d / %6d", crd_x, crd_y, crd_z);
    }

reenable_irq:
    mlx90393_set_interrupt(dev, true);
}

static void mlx90393_irq_gpio_cb(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct mlx90393_data *data = CONTAINER_OF(cb, struct mlx90393_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    mlx90393_set_interrupt(dev, false);
    k_work_submit(&data->read_work);
}

static int mlx90393_init_irq(const struct device *dev) {
    int err;
    struct mlx90393_data *data = dev->data;
    const struct mlx90393_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, mlx90393_irq_gpio_cb, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    return err;
}

static int mlx90393_init(const struct device *dev) {
    const struct mlx90393_config *config = dev->config;
    struct mlx90393_data *data = dev->data;
    int ret;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
    if (!device_is_ready(config->bus.i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    LOG_INF("Initializing MLX90393 at address 0x%02X", config->bus.i2c.addr);
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
    if (!device_is_ready(config->bus.spi.bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }
    LOG_INF("Initializing MLX90393 on SPI bus");
#else
    LOG_ERR("Neither I2C or SPI bus is ready");
    return -ENODEV;
#endif

    data->dev = dev;
    data->calibrated = false;
    data->calibra_cnt = 0;
    data->org_x = data->org_y = data->org_z = 0;
    data->sum_x = data->sum_y = data->sum_z = 0;
    data->min_x = data->min_y = data->min_z = INT32_MAX;
    data->max_x = data->max_y = data->max_z = INT32_MIN;
    data->thd_xy = data->thd_z = 0;

    ret = mlx90393_exit_burst_mode(dev);
    if (ret < 0) {
        LOG_ERR("Failed to exit burst mode: %d", ret);
        return ret;
    }

    ret = mlx90393_reset(dev);
    if (ret < 0) {
        LOG_ERR("Failed to reset: %d", ret);
        return ret;
    }

    ret = mlx90393_set_config(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set config: %d", ret);
        return ret;
    }
    
    k_msleep(300);

    ret = mlx90393_init_irq(dev);
    if (ret < 0) {
        LOG_ERR("Failed to init irq gpio: %d", ret);
        return -EIO;
    }
    k_work_init(&data->read_work, mlx90393_work_handler);
    mlx90393_set_interrupt(dev, true);

    k_msleep(100);

    ret = mlx90393_set_woc_threshold(dev);
    if (ret < 0) {
        LOG_ERR("Failed to set woc threshold: %d", ret);
        return ret;
    }

    ret = mlx90393_start_woc_mode(dev);
    if (ret < 0) {
        LOG_ERR("Failed to start wake-on change mode: %d", ret);
        return ret;
    }

    LOG_INF("MLX90393 input driver initialized successfully");
    return 0;
}

#define CONFIG_ZMK_INPUT_MLX90393_INIT_PRIORITY 80

#define MLX90393_INST(n)                                                                          \
    static struct mlx90393_data mlx90393_data_##n;                                                \
    static const struct mlx90393_config mlx90393_config_##n = {                                   \
        COND_CODE_1(DT_INST_ON_BUS(n, i2c),                                                       \
                    (.bus = {.i2c = I2C_DT_SPEC_INST_GET(n)} ),                                   \
                    (.bus = {.spi = SPI_DT_SPEC_INST_GET(n,                                       \
                                                         SPI_OP_MODE_MASTER |                     \
                                                         SPI_WORD_SET(8) |                        \
                                                         SPI_MODE_CPOL | SPI_MODE_CPHA |          \
                                                         /* SPI_HOLD_ON_CS | SPI_LOCK_ON | */     \
                                                         SPI_TRANSFER_MSB,                        \
                                                         0)} )                                    \
        ),                                                                                        \
        .irq_gpio = GPIO_DT_SPEC_INST_GET_OR(n, irq_gpios, {}),                                   \
        .calib_cycle = DT_INST_PROP(n, calib_cycle),                                              \
        .woc_thd_xy = DT_INST_PROP(n, woc_thd_xy),                                                \
        .woc_thd_z = DT_INST_PROP(n, woc_thd_z),                                                  \
        .rpt_dzn_x = DT_INST_PROP(n, rpt_dzn_x),                                                  \
        .rpt_dzn_y = DT_INST_PROP(n, rpt_dzn_y),                                                  \
        .rpt_dzn_z = DT_INST_PROP(n, rpt_dzn_z),                                                  \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, mlx90393_init, NULL, &mlx90393_data_##n, &mlx90393_config_##n,       \
                          POST_KERNEL, CONFIG_ZMK_INPUT_MLX90393_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MLX90393_INST)
