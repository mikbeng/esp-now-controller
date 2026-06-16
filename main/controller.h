/**
 * @file controller.h
 * @brief ESP-NOW controller header
 *
 * @author Mikael Bengtsson
 * @date 2025-06-11
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"


// Function declarations
bool controller_booted_from_gpio_wake(void);
void controller_enter_deep_sleep_until_wake(void);
esp_err_t controller_init(void);
void controller_start(void);
void controller_stop(void);

#endif /* CONTROLLER_H */