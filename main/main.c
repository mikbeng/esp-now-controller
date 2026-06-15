/**
 * @file main.c
 * @brief Controller main application
 *
 * @author Mikael Bengtsson
 * @date 2025-06-11
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "controller.h"

static const char *TAG = "main";

void app_main(void)
{
    // Initialize ESP-ON hold pin
    init_esp_on_hold_pin();

    ESP_LOGI(TAG, "Starting controller application");
    
    // Initialize controller
    esp_err_t ret = controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize controller");
        return;
    }
    
    // Start controller
    controller_start();
    
    // Main loop
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}