/**
 * @file main.c
 * @brief Controller main application
 *
 * @author Mikael Bengtsson
 * @date 2025-06-11
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "controller.h"

static const char *TAG = "main";

void app_main(void)
{
    if (!controller_booted_from_gpio_wake()) {
        ESP_LOGI(TAG, "Cold boot — entering deep sleep");
        controller_enter_deep_sleep_until_wake();
    }

    ESP_LOGI(TAG, "GPIO wake — starting controller application");

    esp_err_t ret = controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize controller");
        return;
    }

    controller_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
