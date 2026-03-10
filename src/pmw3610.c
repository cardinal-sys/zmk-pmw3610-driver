/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3610_alt

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/pm/device.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/keycode_state_changed.h>

/* HID Usage Page: Keyboard/Keypad (0x07) */
#define HID_USAGE_KEY 0x07

/* HID Usage IDs for arrow keys */
#define HID_USAGE_KEY_KEYBOARD_RIGHT_ARROW 0x4F
#define HID_USAGE_KEY_KEYBOARD_LEFT_ARROW  0x50
#define HID_USAGE_KEY_KEYBOARD_DOWN_ARROW  0x51
#define HID_USAGE_KEY_KEYBOARD_UP_ARROW    0x52
#include "pmw3610.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_ALT_LOG_LEVEL);

enum pmw3610_init_step {
    ASYNC_INIT_STEP_POWER_UP,
    ASYNC_INIT_STEP_CLEAR_OB1,
    ASYNC_INIT_STEP_CHECK_OB1,
    ASYNC_INIT_STEP_CONFIGURE,
    ASYNC_INIT_STEP_COUNT
};

static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP]  = 10 + CONFIG_PMW3610_ALT_INIT_POWER_UP_EXTRA_DELAY_MS,
    [ASYNC_INIT_STEP_CLEAR_OB1] = 200,
    [ASYNC_INIT_STEP_CHECK_OB1] = 50,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3610_async_init_power_up(const struct device *dev);
static int pmw3610_async_init_clear_ob1(const struct device *dev);
static int pmw3610_async_init_check_ob1(const struct device *dev);
static int pmw3610_async_init_configure(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP]  = pmw3610_async_init_power_up,
    [ASYNC_INIT_STEP_CLEAR_OB1] = pmw3610_async_init_clear_ob1,
    [ASYNC_INIT_STEP_CHECK_OB1] = pmw3610_async_init_check_ob1,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3610_async_init_configure,
};

//////// Layer helpers ////////

static bool pmw3610_layer_match(const uint8_t *layers, size_t len) {
    if (len == 0 || layers == NULL) {
        return false;
    }
    uint8_t active = (uint8_t)zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < len; i++) {
        if (layers[i] == active) {
            return true;
        }
    }
    return false;
}

//////// SPI helpers ////////

static int pmw3610_read(const struct device *dev, uint8_t addr, uint8_t *value, uint8_t len) {
    const struct pixart_config *cfg = dev->config;
    const struct spi_buf tx_buf = { .buf = &addr, .len = sizeof(addr) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf rx_buf[] = {
        { .buf = NULL,  .len = sizeof(addr) },
        { .buf = value, .len = len },
    };
    const struct spi_buf_set rx = { .buffers = rx_buf, .count = ARRAY_SIZE(rx_buf) };
    return spi_transceive_dt(&cfg->spi, &tx, &rx);
}

static int pmw3610_read_reg(const struct device *dev, uint8_t addr, uint8_t *value) {
    return pmw3610_read(dev, addr, value, 1);
}

static int pmw3610_write_reg(const struct device *dev, uint8_t addr, uint8_t value) {
    const struct pixart_config *cfg = dev->config;
    uint8_t write_buf[] = { addr | SPI_WRITE_BIT, value };
    const struct spi_buf tx_buf = { .buf = write_buf, .len = sizeof(write_buf) };
    const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
    return spi_write_dt(&cfg->spi, &tx);
}

static int pmw3610_write(const struct device *dev, uint8_t reg, uint8_t val) {
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));
    int err = pmw3610_write_reg(dev, reg, val);
    if (unlikely(err != 0)) {
        return err;
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);
    return 0;
}

//////// CPI ////////

static int pmw3610_set_cpi(const struct device *dev, uint32_t cpi,
                           bool swap_xy, bool inv_x, bool inv_y) {
#if IS_ENABLED(CONFIG_PMW3610_ALT_ORIENTATION_180)
    inv_x = true;
    inv_y = true;
#endif

#if CONFIG_PMW3610_ALT_CPI_DIVIDOR > 1
    cpi = cpi / CONFIG_PMW3610_ALT_CPI_DIVIDOR;
    if (cpi < PMW3610_MIN_CPI) {
        cpi = PMW3610_MIN_CPI;
    }
#endif

    if ((cpi > PMW3610_MAX_CPI) || (cpi < PMW3610_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    uint8_t value = 0x00;
    uint8_t cpi_val = cpi / 200;
    value = (value & 0xE0) | (cpi_val & 0x1F);

    LOG_INF("Setting cpi: %d, swap_xy: %s inv_x: %s inv_y: %s",
            cpi, swap_xy ? "yes" : "no", inv_x ? "yes" : "no", inv_y ? "yes" : "no");

#if IS_ENABLED(CONFIG_PMW3610_ALT_SWAP_XY)
    value |= (1 << 7);
#else
    if (swap_xy) { value |= (1 << 7); } else { value &= ~(1 << 7); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_X)
    value |= (1 << 6);
#else
    if (inv_x) { value |= (1 << 6); } else { value &= ~(1 << 6); }
#endif
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_Y)
    value |= (1 << 5);
#else
    if (inv_y) { value |= (1 << 5); } else { value &= ~(1 << 5); }
#endif

    LOG_INF("Setting CPI reg value 0x%x", value);

    uint8_t addr[] = { 0x7F, PMW3610_REG_RES_STEP, 0x7F };
    uint8_t data[] = { 0xFF, value,                0x00 };
    int err = 0;

    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_ENABLE);
    k_sleep(K_USEC(T_CLOCK_ON_DELAY_US));
    for (size_t i = 0; i < sizeof(data); i++) {
        err = pmw3610_write_reg(dev, addr[i], data[i]);
        if (err) {
            LOG_ERR("Burst write failed");
            break;
        }
    }
    pmw3610_write_reg(dev, PMW3610_REG_SPI_CLK_ON_REQ, PMW3610_SPI_CLOCK_CMD_DISABLE);

    return err;
}

//////// Timing ////////

static int pmw3610_set_sample_time(const struct device *dev, uint8_t reg_addr, uint32_t sample_time) {
    uint32_t maxtime = 2550;
    uint32_t mintime = 10;
    if ((sample_time > maxtime) || (sample_time < mintime)) {
        LOG_WRN("Sample time %u out of range", sample_time);
        return -EINVAL;
    }
    uint8_t value = sample_time / mintime;
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change sample time");
    }
    return err;
}

static int pmw3610_set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    uint32_t maxtime, mintime;

    switch (reg_addr) {
    case PMW3610_REG_RUN_DOWNSHIFT:
        maxtime = 8160;
        mintime = 32;
        break;
    case PMW3610_REG_REST1_DOWNSHIFT:
        maxtime = 255 * 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        mintime = 16 * CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS;
        break;
    case PMW3610_REG_REST2_DOWNSHIFT:
        maxtime = 255 * 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        mintime = 128 * CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS;
        break;
    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range (%u - %u)", time, mintime, maxtime);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    uint8_t value = time / mintime;
    int err = pmw3610_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }
    return err;
}

//////// Performance ////////

static int pmw3610_set_performance(const struct device *dev, bool enabled) {
    const struct pixart_config *config = dev->config;
    int err = 0;

    if (config->force_awake) {
        uint8_t value;
        err = pmw3610_read_reg(dev, PMW3610_REG_PERFORMANCE, &value);
        if (err) {
            LOG_ERR("Can't read performance %d", err);
            return err;
        }

        uint8_t perf = config->force_awake_4ms_mode ? 0x0d : (value & 0x0F);
        if (enabled) {
            perf |= 0xF0;
        }
        if (perf != value) {
            err = pmw3610_write(dev, PMW3610_REG_PERFORMANCE, perf);
            if (err) {
                LOG_ERR("Can't write performance %d", err);
                return err;
            }
        }
    }
    return err;
}

//////// IRQ ////////

static int pmw3610_set_interrupt(const struct device *dev, const bool en) {
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
    return ret;
}

//////// Async init ////////

static int pmw3610_async_init_power_up(const struct device *dev) {
    int ret = pmw3610_write_reg(dev, PMW3610_REG_POWER_UP_RESET, PMW3610_POWERUP_CMD_RESET);
    return ret < 0 ? ret : 0;
}

static int pmw3610_async_init_clear_ob1(const struct device *dev) {
    return pmw3610_write(dev, PMW3610_REG_OBSERVATION, 0x00);
}

static int pmw3610_async_init_check_ob1(const struct device *dev) {
    uint8_t value;
    int err = pmw3610_read_reg(dev, PMW3610_REG_OBSERVATION, &value);
    if (err) {
        LOG_ERR("Can't do self-test");
        return err;
    }
    if ((value & 0x0F) != 0x0F) {
        LOG_ERR("Failed self-test (0x%x)", value);
        return -EINVAL;
    }

    uint8_t product_id = 0x01;
    err = pmw3610_read_reg(dev, PMW3610_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }
    if (product_id != PMW3610_PRODUCT_ID) {
        LOG_ERR("Incorrect product id 0x%x (expecting 0x%x)!", product_id, PMW3610_PRODUCT_ID);
        return -EIO;
    }
    return 0;
}

static int pmw3610_async_init_configure(const struct device *dev) {
    int err = 0;
    const struct pixart_config *config = dev->config;

    for (uint8_t reg = 0x02; (reg <= 0x05) && !err; reg++) {
        uint8_t buf[1];
        err = pmw3610_read_reg(dev, reg, buf);
    }

    if (!err) err = pmw3610_set_performance(dev, true);
    if (!err) err = pmw3610_set_cpi(dev, config->cpi, config->swap_xy, config->inv_x, config->inv_y);
    if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT,  CONFIG_PMW3610_ALT_RUN_DOWNSHIFT_TIME_MS);
    if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, CONFIG_PMW3610_ALT_REST1_DOWNSHIFT_TIME_MS);
    if (!err) err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, CONFIG_PMW3610_ALT_REST2_DOWNSHIFT_TIME_MS);
    if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, CONFIG_PMW3610_ALT_REST1_SAMPLE_TIME_MS);
    if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, CONFIG_PMW3610_ALT_REST2_SAMPLE_TIME_MS);
    if (!err) err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, CONFIG_PMW3610_ALT_REST3_SAMPLE_TIME_MS);

    if (err) {
        LOG_ERR("Config the sensor failed");
    }
    return err;
}

static void pmw3610_async_init(struct k_work *work) {
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("PMW3610 async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("PMW3610 initialization failed in step %d", data->async_init_step);
    } else {
        data->async_init_step++;
        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true;
            LOG_INF("PMW3610 initialized");
            pmw3610_set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

//////// Arrows helper ////////

static void pmw3610_send_arrow_key(uint32_t keycode) {
    struct zmk_keycode_state_changed *ev;

    ev = new_zmk_keycode_state_changed();
    if (ev) {
        ev->usage_page         = HID_USAGE_KEY;
        ev->keycode            = keycode;
        ev->implicit_modifiers = 0;
        ev->explicit_modifiers = 0;
        ev->state              = true;
        ev->timestamp          = k_uptime_get();
        ZMK_EVENT_RAISE(ev);
    }

    k_sleep(K_MSEC(10));

    ev = new_zmk_keycode_state_changed();
    if (ev) {
        ev->usage_page         = HID_USAGE_KEY;
        ev->keycode            = keycode;
        ev->implicit_modifiers = 0;
        ev->explicit_modifiers = 0;
        ev->state              = false;
        ev->timestamp          = k_uptime_get();
        ZMK_EVENT_RAISE(ev);
    }
}

//////// Report data ////////

static int pmw3610_report_data(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    uint8_t buf[PMW3610_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int err = pmw3610_read(dev, PMW3610_REG_MOTION_BURST, buf, PMW3610_BURST_SIZE);
    if (err) {
        return err;
    }

#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

    int16_t x = TOINT16((buf[PMW3610_X_L_POS] + ((buf[PMW3610_XY_H_POS] & 0xF0) << 4)), 12);
    int16_t y = TOINT16((buf[PMW3610_Y_L_POS] + ((buf[PMW3610_XY_H_POS] & 0x0F) << 8)), 12);
    LOG_DBG("x/y: %d/%d", x, y);

#ifdef CONFIG_PMW3610_ALT_SMART_ALGORITHM
    int16_t shutter = ((int16_t)(buf[PMW3610_SHUTTER_H_POS] & 0x01) << 8)
                    + buf[PMW3610_SHUTTER_L_POS];
    if (data->sw_smart_flag && shutter < 45) {
        pmw3610_write(dev, 0x32, 0x00);
        data->sw_smart_flag = false;
    }
    if (!data->sw_smart_flag && shutter > 45) {
        pmw3610_write(dev, 0x32, 0x80);
        data->sw_smart_flag = true;
    }
#endif

#if CONFIG_PMW3610_ALT_MOVEMENT_THRESHOLD > 0
    if (abs(x) < CONFIG_PMW3610_ALT_MOVEMENT_THRESHOLD) x = 0;
    if (abs(y) < CONFIG_PMW3610_ALT_MOVEMENT_THRESHOLD) y = 0;
    if (x == 0 && y == 0) return 0;
#endif

    /* ======================================================
     * SCROLL LAYER: emit WHEEL/HWHEEL instead of REL_X/Y
     * ====================================================== */
    if (pmw3610_layer_match(config->scroll_layers, config->scroll_layers_len)) {
        data->scroll_dx += x;
        data->scroll_dy += y;
        data->snipe_dx = 0;
        data->snipe_dy = 0;
        data->arrows_dx = 0;
        data->arrows_dy = 0;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;

        bool scrolled = false;

        if (abs(data->scroll_dx) >= CONFIG_PMW3610_ALT_SCROLL_TICK) {
            int16_t hwheel = (int16_t)(data->scroll_dx / CONFIG_PMW3610_ALT_SCROLL_TICK);
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_SCROLL_X)
            hwheel = -hwheel;
#endif
            input_report_rel(dev, INPUT_REL_HWHEEL, hwheel, false, K_FOREVER);
            data->scroll_dx %= CONFIG_PMW3610_ALT_SCROLL_TICK;
            scrolled = true;
        }

        if (abs(data->scroll_dy) >= CONFIG_PMW3610_ALT_SCROLL_TICK) {
            int16_t wheel = (int16_t)(data->scroll_dy / CONFIG_PMW3610_ALT_SCROLL_TICK);
#if IS_ENABLED(CONFIG_PMW3610_ALT_INVERT_SCROLL_Y)
            wheel = -wheel;
#endif
            input_report_rel(dev, INPUT_REL_WHEEL, wheel, false, K_FOREVER);
            data->scroll_dy %= CONFIG_PMW3610_ALT_SCROLL_TICK;
            scrolled = true;
        }

        if (scrolled) {
            /* sync event */
            input_report_rel(dev, INPUT_REL_WHEEL, 0, true, K_FOREVER);
        }
        return 0;
    }

    /* ======================================================
     * SNIPE LAYER: divide movement, carry over remainder
     * ====================================================== */
    if (pmw3610_layer_match(config->snipe_layers, config->snipe_layers_len)) {
        data->scroll_dx = 0;
        data->scroll_dy = 0;
        data->arrows_dx = 0;
        data->arrows_dy = 0;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;

        data->snipe_dx += x;
        data->snipe_dy += y;

        int16_t rx = (int16_t)(data->snipe_dx / CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR);
        int16_t ry = (int16_t)(data->snipe_dy / CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR);
        data->snipe_dx %= CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR;
        data->snipe_dy %= CONFIG_PMW3610_ALT_SNIPE_CPI_DIVIDOR;

        bool have_x = rx != 0;
        bool have_y = ry != 0;

        if (have_x) {
            input_report(dev, config->evt_type, config->x_input_code, rx, !have_y, K_FOREVER);
        }
        if (have_y) {
            input_report(dev, config->evt_type, config->y_input_code, ry, true, K_FOREVER);
        }
        return 0;
    }

    /* ======================================================
     * ARROWS LAYER: convert movement to arrow key presses
     * Whichever axis has larger displacement wins.
     * Diagonal input is suppressed.
     * ====================================================== */
    if (pmw3610_layer_match(config->arrows_layers, config->arrows_layers_len)) {
        data->scroll_dx = 0;
        data->scroll_dy = 0;
        data->snipe_dx  = 0;
        data->snipe_dy  = 0;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;

        data->arrows_dx += x;
        data->arrows_dy += y;

        int tick = config->arrows_tick;

        if (abs(data->arrows_dx) >= tick || abs(data->arrows_dy) >= tick) {
            uint32_t keycode;
            if (abs(data->arrows_dx) >= abs(data->arrows_dy)) {
                keycode = data->arrows_dx > 0
                    ? HID_USAGE_KEY_KEYBOARD_RIGHT_ARROW
                    : HID_USAGE_KEY_KEYBOARD_LEFT_ARROW;
            } else {
                keycode = data->arrows_dy > 0
                    ? HID_USAGE_KEY_KEYBOARD_DOWN_ARROW
                    : HID_USAGE_KEY_KEYBOARD_UP_ARROW;
            }
            data->arrows_dx = 0;
            data->arrows_dy = 0;

            pmw3610_send_arrow_key(keycode);
        }
        return 0;
    }

    /* ======================================================
     * NORMAL CURSOR
     * ====================================================== */
    data->snipe_dx  = 0;
    data->snipe_dy  = 0;
    data->arrows_dx = 0;
    data->arrows_dy = 0;

    /* 2-sample accumulation: Dist版 POLLING_RATE_125_SW 相当 */
    int64_t curr_time = k_uptime_get();
    if (data->last_poll_time == 0 || curr_time - data->last_poll_time > 128) {
        data->last_poll_time = curr_time;
        data->last_x = x;
        data->last_y = y;
        return 0;
    } else {
        x += data->last_x;
        y += data->last_y;
        data->last_poll_time = 0;
        data->last_x = 0;
        data->last_y = 0;
    }

    bool have_x = x != 0;
    bool have_y = y != 0;

    if (have_x) {
        input_report(dev, config->evt_type, config->x_input_code, x, !have_y, K_FOREVER);
    }
    if (have_y) {
        input_report(dev, config->evt_type, config->y_input_code, y, true, K_FOREVER);
    }

    return 0;
}

//////// Callbacks ////////

static void pmw3610_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;
    pmw3610_set_interrupt(dev, false);
    k_work_submit(&data->trigger_work);
}

static void pmw3610_work_callback(struct k_work *work) {
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;
    pmw3610_report_data(dev);
    pmw3610_set_interrupt(dev, true);
}

static int pmw3610_init_irq(const struct device *dev) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }

    gpio_init_callback(&data->irq_gpio_cb, pmw3610_gpio_callback, BIT(config->irq_gpio.pin));
    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }
    return err;
}

static int pmw3610_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("%s is not ready", config->spi.bus->name);
        return -ENODEV;
    }

    data->dev = dev;
    data->sw_smart_flag = false;
    data->scroll_dx = 0;
    data->scroll_dy = 0;
    data->snipe_dx     = 0;
    data->snipe_dy     = 0;
    data->arrows_dx    = 0;
    data->arrows_dy    = 0;
    data->last_poll_time = 0;
    data->last_x       = 0;
    data->last_y       = 0;

    k_work_init(&data->trigger_work, pmw3610_work_callback);

    err = pmw3610_init_irq(dev);
    if (err) {
        return err;
    }

    k_work_init_delayable(&data->init_work, pmw3610_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

//////// Sensor API ////////

static int pmw3610_alt_attr_set(const struct device *dev, enum sensor_channel chan,
                                enum sensor_attribute attr, const struct sensor_value *val) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    if (unlikely(chan != SENSOR_CHAN_ALL)) return -ENOTSUP;
    if (unlikely(!data->ready)) {
        LOG_DBG("Device is not initialized yet");
        return -EBUSY;
    }

    switch ((uint32_t)attr) {
    case PMW3610_ALT_ATTR_CPI:
        err = pmw3610_set_cpi(dev, PMW3610_SVALUE_TO_CPI(*val),
                              config->swap_xy, config->inv_x, config->inv_y);
        break;
    case PMW3610_ALT_ATTR_RUN_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_RUN_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST1_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST1_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST2_DOWNSHIFT_TIME:
        err = pmw3610_set_downshift_time(dev, PMW3610_REG_REST2_DOWNSHIFT, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST1_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST1_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST2_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST2_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;
    case PMW3610_ALT_ATTR_REST3_SAMPLE_TIME:
        err = pmw3610_set_sample_time(dev, PMW3610_REG_REST3_RATE, PMW3610_SVALUE_TO_TIME(*val));
        break;
    default:
        LOG_ERR("Unknown attribute");
        err = -ENOTSUP;
    }
    return err;
}

static const struct sensor_driver_api pmw3610_driver_api = {
    .attr_set = pmw3610_alt_attr_set,
};

//////// Device instantiation ////////

#define PMW3610_SPI_MODE (SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
                          SPI_MODE_CPHA | SPI_TRANSFER_MSB)

#define PMW3610_LAYERS_DEFINE(n, prop)                                              \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop),                                    \
        (static const uint8_t prop##_##n[] = DT_INST_PROP(n, prop);),              \
        ())

#define PMW3610_LAYERS_PTR(n, prop)                                                 \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop), (prop##_##n), (NULL))

#define PMW3610_LAYERS_LEN(n, prop)                                                 \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, prop),                                    \
        (DT_INST_PROP_LEN(n, prop)), (0))

#define PMW3610_DEFINE(n)                                                           \
    PMW3610_LAYERS_DEFINE(n, scroll_layers)                                         \
    PMW3610_LAYERS_DEFINE(n, snipe_layers)                                          \
    PMW3610_LAYERS_DEFINE(n, arrows_layers)                                         \
    static struct pixart_data data##n;                                              \
    static const struct pixart_config config##n = {                                 \
        .spi = SPI_DT_SPEC_INST_GET(n, PMW3610_SPI_MODE, 0),                       \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                           \
        .cpi = DT_PROP(DT_DRV_INST(n), cpi),                                       \
        .swap_xy = DT_PROP(DT_DRV_INST(n), swap_xy),                               \
        .inv_x = DT_PROP(DT_DRV_INST(n), invert_x),                                \
        .inv_y = DT_PROP(DT_DRV_INST(n), invert_y),                                \
        .evt_type = DT_PROP(DT_DRV_INST(n), evt_type),                             \
        .x_input_code = DT_PROP(DT_DRV_INST(n), x_input_code),                     \
        .y_input_code = DT_PROP(DT_DRV_INST(n), y_input_code),                     \
        .force_awake = DT_PROP(DT_DRV_INST(n), force_awake),                       \
        .force_awake_4ms_mode = DT_PROP(DT_DRV_INST(n), force_awake_4ms_mode),     \
        .scroll_layers = PMW3610_LAYERS_PTR(n, scroll_layers),                     \
        .scroll_layers_len = PMW3610_LAYERS_LEN(n, scroll_layers),                 \
        .snipe_layers = PMW3610_LAYERS_PTR(n, snipe_layers),                       \
        .snipe_layers_len = PMW3610_LAYERS_LEN(n, snipe_layers),                   \
        .arrows_layers = PMW3610_LAYERS_PTR(n, arrows_layers),                     \
        .arrows_layers_len = PMW3610_LAYERS_LEN(n, arrows_layers),                 \
        .arrows_tick = DT_PROP_OR(DT_DRV_INST(n), arrows_tick, 10),               \
    };                                                                              \
    DEVICE_DT_INST_DEFINE(n, pmw3610_init, NULL, &data##n, &config##n,             \
                          POST_KERNEL, CONFIG_INPUT_PMW3610_INIT_PRIORITY,          \
                          &pmw3610_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3610_DEFINE)

#define GET_PMW3610_DEV(node_id) DEVICE_DT_GET(node_id),

static const struct device *pmw3610_devs[] = {
    DT_FOREACH_STATUS_OKAY(pixart_pmw3610_alt, GET_PMW3610_DEV)
};

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        LOG_WRN("NO EVENT, leaving early");
        return 0;
    }
    bool enable = state_ev->state == ZMK_ACTIVITY_ACTIVE ? 1 : 0;
    for (size_t i = 0; i < ARRAY_SIZE(pmw3610_devs); i++) {
        pmw3610_set_performance(pmw3610_devs[i], enable);
    }
    return 0;
}

ZMK_LISTENER(zmk_pmw3610_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_pmw3610_idle_sleeper, zmk_activity_state_changed);
