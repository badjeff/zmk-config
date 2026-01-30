/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

/*

## Overview

This is a MLX90393 driver for Zephye Input Sub-system.

- I2C/SPI Bus Compitable
- Auto-calibration for dead zone
- Intrrupt with on-chip Wake-On Change Mode
- Auto-switch to full-res Burst Mode
- Downshift to Wake-on Change mode


## Device tree node sample:

&pinctrl {
    spi2_default: spi2_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 20)>,
                    <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };
    spi2_sleep: spi2_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                    <NRF_PSEL(SPIM_MOSI, 0, 20)>,
                    <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

#include <dt-bindings/spi/spi.h>
&spi2 {
    compatible = "nordic,nrf-spim";
    status = "okay";
    pinctrl-0 = <&spi2_default>;
    pinctrl-1 = <&spi2_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio1 4 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
    spi_mlx90393: mlx90393@0 {
        compatible = "melexis,mlx90393-input";
        status = "okay";
        spi-max-frequency = <1000000>;
        reg = <0>;
        irq-gpios = <&gpio1 6 (GPIO_ACTIVE_HIGH | GPIO_PULL_DOWN)>;
        no-z;
    };
};


## Sample initial log:

[00:00:00.554,382] <inf> input_mlx90393: Initializing MLX90393 on SPI bus
[00:00:00.554,412] <inf> input_mlx90393: set mask_zyxt: 0xF6
[00:00:00.554,504] <inf> input_mlx90393: exit status: 0x01
[00:00:00.554,565] <inf> input_mlx90393: reset status: 0x05
[00:00:00.554,687] <inf> input_mlx90393: read 0x00 config status: 0x01
[00:00:00.554,809] <inf> input_mlx90393: read 0x02 config status: 0x01
[00:00:00.954,986] <inf> input_mlx90393: setting woc threshold xy:0 z:0
[00:00:00.955,108] <inf> input_mlx90393: read WOXY_THRESHOLD status: 0x01
[00:00:00.955,230] <inf> input_mlx90393: read WOZ_THRESHOLD status: 0x01
[00:00:00.955,322] <inf> input_mlx90393: read 0x01 config status: 0x01
[00:00:00.955,413] <inf> input_mlx90393: wake-on change mode status: 0x41
[00:00:00.955,413] <inf> input_mlx90393: MLX90393 input driver initialized successfully


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
    bool no_x, no_y, no_z;
    uint16_t calib_cycle;
    uint16_t ex_woc_thd_xy, ex_woc_thd_z;
    uint16_t downshift;
    uint16_t rpt_dzn_x, rpt_dzn_y, rpt_dzn_z;
};

struct mlx90393_data {
    const struct device *dev;
    struct gpio_callback irq_gpio_cb;
    struct k_work trigger_work;
    struct k_work read_work;
    uint8_t mask_zyxt;
    bool calibrated;
    uint16_t calibra_cnt;
    int16_t org_x, org_y, org_z;
    int32_t sum_x, sum_y, sum_z;
    int32_t min_x, min_y, min_z;
    int32_t max_x, max_y, max_z;
    int16_t thd_xy; // 0x07h: WOXY_THRESHOLD [15:0] (signed)
    int16_t thd_z;  // 0x08h: WOZ_THRESHOLD [15:0] (signed)
    bool in_woc_mode, in_burst_mode;
    uint16_t burst_in_deadzone;
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

static void mlx90393_set_current_mode(const struct device *dev, bool in_woc_mode, bool in_burst_mode) {
    struct mlx90393_data *data = dev->data;
    data->in_woc_mode = in_woc_mode;
    data->in_burst_mode = in_burst_mode;
}

static int mlx90393_exit_mode(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    // struct mlx90393_data *data = dev->data;
    int ret = 0;
    uint8_t cmd;
    uint8_t status;

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
    mlx90393_set_current_mode(dev, false, false);

    // k_msleep(1); // if BURST_DATA_RATE down to zero, wait at least 1ms

    return ret;
}

static int mlx90393_start_burst_mode(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    struct mlx90393_data *data = dev->data;
    int ret = 0;
    if (!data->in_burst_mode) {
        uint8_t cmd;
        uint8_t status;

        cmd = 0x1E & data->mask_zyxt; // Start Burst Mode for zyx
        ret = mlx90393_cmd_read(dev, &cmd, 1, &status, 1);

        if (ret < 0) {
            LOG_WRN("Failed to read burst mode status: %d", ret);
            return ret;
        }
        else if (status & MLX90383_STATUS_ERR_BITS) {
            LOG_ERR("read burst mode: 0x%02X", status);
            return -EINVAL;
        }
        LOG_INF("burst mode status: 0x%02x", status);
        mlx90393_set_current_mode(dev, false, true);
    }
    return ret;
}

static int mlx90393_start_woc_mode(const struct device *dev) {
    // const struct mlx90393_config *config = dev->config;
    struct mlx90393_data *data = dev->data;
    int ret = 0;
    if (!data->in_woc_mode) {
        uint8_t cmd;
        uint8_t status;

        cmd = 0x2E & data->mask_zyxt; // Start Wake-up on Change Mode for zyx
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
        mlx90393_set_current_mode(dev, true, false);
    }
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
    const struct mlx90393_config *config = dev->config;
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
    reg_config[2] = 0x01;  // AL: BURST_DATA_RATE = N * 20ms
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
    cmd = 0x4E & data->mask_zyxt;
    ret = mlx90393_cmd_read(dev, &cmd, 1, read_data, 7);

    if (ret < 0) {
        LOG_ERR("Failed to read measurement data: %d", ret);
        goto reenable_irq;
    }
    // Convert the data (big endian MSB:LSB for each axis)
    uint8_t _data_offset_ = 1;
    if (data->mask_zyxt & BIT(1)) {
        x = (int16_t)sys_get_be16(&read_data[_data_offset_]);
        _data_offset_ += 2;
    }
    if (data->mask_zyxt & BIT(2)) {
        y = (int16_t)sys_get_be16(&read_data[_data_offset_]);
        _data_offset_ += 2;
    }
    if (data->mask_zyxt & BIT(3)) {
        z = (int16_t)sys_get_be16(&read_data[_data_offset_]);
        // _data_offset_ += 2;
    }
    
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
            LOG_INF("calibrating %d/%d: x:%d y:%d z:%d", data->calibra_cnt, config->calib_cycle, x, y, z);
        }

        if (data->calibra_cnt >= config->calib_cycle) {
            data->org_x = data->sum_x / config->calib_cycle;
            data->org_y = data->sum_y / config->calib_cycle;
            data->org_z = data->sum_z / config->calib_cycle;
            LOG_INF("calibrated org: x:%d y:%d z:%d", data->org_x, data->org_y, data->org_z);

            data->thd_xy = MAX(data->max_x - data->min_x, data->max_y - data->min_y);
            data->thd_z = (data->max_z - data->min_z);
            LOG_INF("min/max: x:%d/%d y:%d/%d z:%d/%d", 
                    (int)data->min_x, (int)data->max_x, 
                    (int)data->min_y, (int)data->max_y, 
                    (int)data->min_z, (int)data->max_z);

            //
            // convert woc threshold to calibrated result
            // tune `thd_trim` with 3 facts: noise reduction, resolution, power efficiency
            //
            float thd_trim = 0.89; // normal distribution trim
            data->thd_xy = ( data->thd_xy * thd_trim ) + config->ex_woc_thd_xy;
            data->thd_z = ( data->thd_z * thd_trim ) + config->ex_woc_thd_z;
            LOG_INF("thd xy:%6d z:%6d", data->thd_xy, data->thd_z);

            data->calibrated = true;

            ret = mlx90393_exit_mode(dev);
            if (ret < 0) {
                LOG_ERR("Failed to exit mode: %d", ret);
                goto reenable_irq;
            }

            ret = mlx90393_set_woc_threshold(dev);
            if (ret < 0) {
                LOG_ERR("Failed to set woc threshold: %d", ret);
                goto reenable_irq;
            }

            //
            // when calibration done and enter WOC mode to reduce power;
            //
            ret = mlx90393_start_woc_mode(dev);
            if (ret < 0) {
                LOG_ERR("Failed to start wake-on change mode: %d", ret);
                goto reenable_irq;
            }

        }
        goto reenable_irq; // Skip input during calibration
    }

    int16_t dx = x - data->org_x;
    int16_t dy = y - data->org_y;
    int16_t dz = z - data->org_z;
    
    if (abs(dx) < (int)config->rpt_dzn_x) dx = 0;
    if (abs(dy) < (int)config->rpt_dzn_y) dy = 0;
    if (abs(dz) < (int)config->rpt_dzn_z) dz = 0;

    // LOG_DBG("raw x:%6d y:%6d z:%6d | org x:%d y:%d z:%d | crd z:%6d y:%6d z:%6d",
    //         x, y, z, data->org_x, data->org_y, data->org_z, dx, dy, dz);
    // LOG_DBG("min/max: x:%d/%d y:%d/%d z:%d/%d", 
    //         (int)data->min_x, (int)data->max_x, (int)data->min_y, (int)data->max_y, 
    //         (int)data->min_z, (int)data->max_z);
    // LOG_DBG("thd xy:%d z:%d", data->thd_xy, data->thd_z);

    if (dx || dy || dz) {
        if (dx) {
            input_report_rel(dev, INPUT_REL_X, dx, !dy && !dz, K_FOREVER);
        }
        if (dy) {
            input_report_rel(dev, INPUT_REL_Y, dy, !dz, K_FOREVER);
        }
        if (dz) {
            input_report_rel(dev, INPUT_REL_Z, dz, true, K_FOREVER);
        }
        LOG_DBG("delta x/y/z: %6d / %6d / %6d", dx, dy, dz);
    }

    bool in_deadzone = !dx && !dy && !dz;

    //
    // when the most recent readings are all within the dead zone of the downshift interval.
    // this means the joystick has returned to the neutral position, switching back to WOC mode;
    //
    if (in_deadzone && data->in_burst_mode) {

        data->burst_in_deadzone = data->burst_in_deadzone + 1;

        #if CONFIG_ZMK_LOG_LEVEL >= LOG_LEVEL_DBG
        if (data->burst_in_deadzone % (config->downshift/3) == 0) {
            LOG_DBG("burst_in_deadzone: %d", data->burst_in_deadzone);
        }
        #endif

        if (data->burst_in_deadzone >= config->downshift) {
            data->burst_in_deadzone = 0;

            ret = mlx90393_exit_mode(dev);
            if (ret < 0) {
                LOG_ERR("Failed to exit mode: %d", ret);
                goto reenable_irq;
            }

            ret = mlx90393_start_woc_mode(dev);
            if (ret < 0) {
                LOG_ERR("Failed to start burst mode: %d", ret);
                goto reenable_irq;
            }
        }

        goto reenable_irq;
    }

    //
    // when WOC intrrupt triggered, switch to burst most to max out the resolution;
    //
    if (!in_deadzone && !data->in_burst_mode) {

        ret = mlx90393_exit_mode(dev);
        if (ret < 0) {
            LOG_ERR("Failed to exit mode: %d", ret);
            goto reenable_irq;
        }

        ret = mlx90393_start_burst_mode(dev);
        if (ret < 0) {
            LOG_ERR("Failed to start burst mode: %d", ret);
            goto reenable_irq;
        }
        // LOG_WRN("BURST MODE!");

        goto reenable_irq;
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
    data->in_woc_mode = false;
    data->in_burst_mode = false;
    data->burst_in_deadzone = 0;

    uint8_t mask_zyxt = 0xFE;
    if (config->no_x) mask_zyxt ^= BIT(1);
    if (config->no_y) mask_zyxt ^= BIT(2);
    if (config->no_z) mask_zyxt ^= BIT(3);
    data->mask_zyxt = mask_zyxt;
    LOG_INF("set mask_zyxt: 0x%02X", data->mask_zyxt);

    ret = mlx90393_exit_mode(dev);
    if (ret < 0) {
        LOG_ERR("Failed to exit mode: %d", ret);
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
        .no_x = DT_INST_PROP(n, no_x),                                                            \
        .no_y = DT_INST_PROP(n, no_y),                                                            \
        .no_z = DT_INST_PROP(n, no_z),                                                            \
        .calib_cycle = DT_INST_PROP(n, calib_cycle),                                              \
        .ex_woc_thd_xy = DT_INST_PROP(n, ex_woc_thd_xy),                                          \
        .ex_woc_thd_z = DT_INST_PROP(n, ex_woc_thd_z),                                            \
        .downshift = DT_INST_PROP(n, downshift),                                                  \
        .rpt_dzn_x = DT_INST_PROP(n, rpt_dzn_x),                                                  \
        .rpt_dzn_y = DT_INST_PROP(n, rpt_dzn_y),                                                  \
        .rpt_dzn_z = DT_INST_PROP(n, rpt_dzn_z),                                                  \
    };                                                                                            \
    DEVICE_DT_INST_DEFINE(n, mlx90393_init, NULL, &mlx90393_data_##n, &mlx90393_config_##n,       \
                          POST_KERNEL, CONFIG_ZMK_INPUT_MLX90393_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MLX90393_INST)
