#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pixart_data {
    const struct device          *dev;
    bool                         sw_smart_flag;

    struct gpio_callback         irq_gpio_cb;
    struct k_work                trigger_work;

    struct k_work_delayable      init_work;
    int                          async_init_step;

    bool                         ready;
    int                          err;

    /* scroll mode accumulation */
    int32_t                      scroll_dx;
    int32_t                      scroll_dy;

    /* snipe mode accumulation (remainder carry-over) */
    int32_t                      snipe_dx;
    int32_t                      snipe_dy;
};

struct pixart_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq_gpio;
    uint16_t cpi;
    bool swap_xy;
    bool inv_x;
    bool inv_y;
    uint8_t evt_type;
    uint8_t x_input_code;
    uint8_t y_input_code;
    bool force_awake;
    bool force_awake_4ms_mode;
    const uint8_t *scroll_layers;
    size_t scroll_layers_len;
    const uint8_t *snipe_layers;
    size_t snipe_layers_len;
};

#ifdef __cplusplus
}
#endif
