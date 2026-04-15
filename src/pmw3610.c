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
#include <string.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include "pmw3610.h"

/* Linux input keycodes for arrow keys */
#define ARROWS_KEY_UP    103
#define ARROWS_KEY_DOWN  108
#define ARROWS_KEY_LEFT  105
#define ARROWS_KEY_RIGHT 106

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3610, CONFIG_PMW3610_ALT_LOG_LEVEL);