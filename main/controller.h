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
esp_err_t controller_init(void);
void controller_start(void);
void controller_stop(void);

#endif /* CONTROLLER_H */