/**
 * @file controller.c
 * @brief ESP-NOW controller implementation
 *
 * @author Mikael Bengtsson
 * @date 2025-06-11
 */

#include "controller.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include "nvs_flash.h"

#define TAG "controller"

// User LED
#define LED_USER_PIN  0

// Button GPIO pins - adjust these to match your hardware
#define BTN_UP_PIN     17
#define BTN_DOWN_PIN   19
#define BTN_LEFT_PIN   23
#define BTN_RIGHT_PIN  22

#define ESP_ON_HOLD_PIN 21

// Button bit positions
#define BTN_UP_BIT     0
#define BTN_DOWN_BIT   1
#define BTN_LEFT_BIT   2
#define BTN_RIGHT_BIT  3

// ESP-NOW data structure
typedef struct {
    uint8_t buttons;  // Bitmap of button states
    uint8_t reserved; // Reserved for future use
} espnow_input_data_t;

// Global variables
//static uint8_t receiver_mac[6] = {0xE8, 0x9F, 0x6D, 0x21, 0x8F, 0xEC};  // Display's MAC address
static uint8_t receiver_mac[6] = {0x98, 0xA3, 0x16, 0x8E, 0x8B, 0x04};  // Test receiver's MAC address 98:A3:16:8E:8B:04
static bool is_paired = false;
static esp_now_peer_info_t peer_info;
static bool is_running = false;

// Function declarations
static void init_buttons(void);
static void init_user_led(void);
static void init_wifi(void);
static void init_espnow(void);
static void send_button_states(void);
static void controller_task(void *pvParameters);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

esp_err_t controller_init(void) {
    ESP_LOGI(TAG, "Initializing controller");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize buttons and user LED
    init_buttons();
    init_user_led();

    
    // Initialize WiFi and ESP-NOW
    init_wifi();
    init_espnow();
    
    // Print MAC address for debugging
    uint8_t mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    ESP_LOGI(TAG, "Controller MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    ESP_LOGI(TAG, "Controller initialized");
    return ESP_OK;
}

void controller_start(void) {
    if (is_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Starting controller");
    
    // Set running flag before creating task
    is_running = true;
    
    // Start controller task
    TaskHandle_t task_handle = NULL;
    BaseType_t result = xTaskCreate(
        controller_task,           // Task function
        "controller_task",        // Task name
        8192,                     // Stack size
        NULL,                     // Task parameters
        10,                       // Task priority
        &task_handle              // Task handle
    );
    
    if (result != pdPASS || task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create controller task");
        is_running = false;
        return;
    }
}

void controller_stop(void) {
    if (!is_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping controller");
    is_running = false;
    
    // Cleanup ESP-NOW
    esp_now_deinit();

    // Turn off user LED
    gpio_set_level(LED_USER_PIN, 0);
    
    // Cleanup WiFi
    esp_wifi_stop();
    esp_wifi_deinit();
}

static void init_buttons(void) {
    // Configure button pins as inputs with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_UP_PIN) | (1ULL << BTN_DOWN_PIN) |
                        (1ULL << BTN_LEFT_PIN) | (1ULL << BTN_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void init_esp_on_hold_pin(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ESP_ON_HOLD_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ESP_ON_HOLD_PIN, 1);
}

static void init_user_led(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_USER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_USER_PIN, 0);
}
static void init_wifi(void) {
    // Initialize WiFi in APSTA mode for better ESP-NOW reliability
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    
    // Configure AP
    wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP-NOW-Controller",
            .password = "",
            .ssid_len = 0,
            .channel = 1,
            .authmode = WIFI_AUTH_OPEN,
            .ssid_hidden = 1,
            .max_connection = 0,
            .beacon_interval = 100
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));
    
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "Message sent successfully");
    } else {
        ESP_LOGW(TAG, "Message send failed");
    }
}

static void init_espnow(void) {
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    
    // Register send callback with our callback function
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
}

static void send_button_states(void) {
    static uint8_t last_button_state = 0;
    uint8_t current_button_state = 0;
    
    // Read current button states
    if (gpio_get_level(BTN_UP_PIN) == 0) current_button_state |= (1 << BTN_UP_BIT);
    if (gpio_get_level(BTN_DOWN_PIN) == 0) current_button_state |= (1 << BTN_DOWN_BIT);
    if (gpio_get_level(BTN_LEFT_PIN) == 0) current_button_state |= (1 << BTN_LEFT_BIT);
    if (gpio_get_level(BTN_RIGHT_PIN) == 0) current_button_state |= (1 << BTN_RIGHT_BIT);
    
    // Only act if button state has changed
    if (current_button_state != last_button_state) {
        ESP_LOGI(TAG, "Buttons: UP=%d DOWN=%d LEFT=%d RIGHT=%d (0x%02x)",
                 (current_button_state >> BTN_UP_BIT) & 1,
                 (current_button_state >> BTN_DOWN_BIT) & 1,
                 (current_button_state >> BTN_LEFT_BIT) & 1,
                 (current_button_state >> BTN_RIGHT_BIT) & 1,
                 current_button_state);

        if (is_paired) {
            espnow_input_data_t data = {0};
            data.buttons = current_button_state;

            esp_err_t result = esp_now_send(peer_info.peer_addr, (uint8_t *)&data, sizeof(data));
            if (result != ESP_OK) {
                ESP_LOGE(TAG, "Error sending data: %d", result);
                is_paired = false;
                gpio_set_level(LED_USER_PIN, 0);
            }
        }

        last_button_state = current_button_state;
    }
}

static void controller_task(void *pvParameters) {
    // Set up peer info
    memcpy(peer_info.peer_addr, receiver_mac, 6);
    peer_info.channel = 1;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;
    
    uint32_t last_pair_attempt = 0;
    const uint32_t PAIR_RETRY_INTERVAL = 5000; // 5 seconds between retries
    
    while (is_running) {
        if (!is_paired) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Only attempt pairing every PAIR_RETRY_INTERVAL
            if (current_time - last_pair_attempt >= PAIR_RETRY_INTERVAL) {
                // Remove existing peer if any
                esp_now_del_peer(peer_info.peer_addr);
                
                // Try to add peer
                esp_err_t result = esp_now_add_peer(&peer_info);
                if (result == ESP_OK) {
                    is_paired = true;
                    gpio_set_level(LED_USER_PIN, 1);
                    ESP_LOGI(TAG, "Successfully paired with display");
                } else {
                    gpio_set_level(LED_USER_PIN, 0);
                    ESP_LOGW(TAG, "Failed to pair (error %d), will retry in %ld seconds", 
                            result, PAIR_RETRY_INTERVAL/1000);
                }
                last_pair_attempt = current_time;
            }
        }

        send_button_states();
        
        vTaskDelay(20 / portTICK_PERIOD_MS);  // 50Hz update rate
    }
    
    vTaskDelete(NULL);
}