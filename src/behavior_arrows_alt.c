#define DT_DRV_COMPAT zmk_behavior_arrows_alt

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include "pixart.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int on_arrows_alt_binding_pressed(struct zmk_behavior_binding *binding,
                                          struct zmk_behavior_binding_event event) {
    pmw3610_arrows_alt_mode_set(true);
    zmk_keymap_layer_activate((uint8_t)binding->param1);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_arrows_alt_binding_released(struct zmk_behavior_binding *binding,
                                           struct zmk_behavior_binding_event event) {
    zmk_keymap_layer_deactivate((uint8_t)binding->param1);
    pmw3610_arrows_alt_mode_set(false);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_arrows_alt_driver_api = {
    .binding_pressed  = on_arrows_alt_binding_pressed,
    .binding_released = on_arrows_alt_binding_released,
};

#define ARROWS_ALT_INST(n)                                          \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL,              \
                            POST_KERNEL,                             \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,    \
                            &behavior_arrows_alt_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ARROWS_ALT_INST)
