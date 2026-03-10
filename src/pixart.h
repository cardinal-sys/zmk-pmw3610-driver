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

    /* arrows mode accumulation */
    int32_t                      arrows_dx;
    int32_t                      arrows_dy;

    /* inertia scroll: velocity in fixed-point (*256) */
    int32_t                      inertia_vx;
    int32_t                      inertia_vy;
    struct k_work_delayable      inertia_work;

    /* flick detection: ring buffer of recent raw deltas */
    int16_t                      flick_hist_x[4];
    int16_t                      flick_hist_y[4];
    uint8_t                      flick_idx;

    /* 2-sample accumulation (POLLING_RATE_125_SW equivalent) */
    int64_t                      last_poll_time;
    int16_t                      last_x;
    int16_t                      last_y;
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
    const uint8_t *arrows_layers;
    size_t arrows_layers_len;
    int arrows_tick;
    bool scroll_inertia;
    int  scroll_inertia_decay;       /* slow-speed decay % (default 75) */
    int  scroll_inertia_decay_fast;  /* high-speed decay % (default 92) */
    int  scroll_inertia_fast_spd;    /* speed threshold for fast decay (default 6) */
    int  scroll_inertia_slow_spd;    /* speed threshold for slow decay (default 2) */
    int  scroll_inertia_tick_ms;
    int  scroll_flick_threshold;     /* avg raw delta/sample to detect flick (default 6) */
    int  scroll_flick_boost;         /* velocity multiplier *256 on flick (default 512=2x) */
};

#ifdef __cplusplus
}
#endif
