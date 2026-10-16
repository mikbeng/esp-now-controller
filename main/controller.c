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
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_system.h"

#define TAG "controller"

// User LED
#define LED_USER_PIN  0

// Button GPIO pins - adjust these to match your hardware
#define BTN_UP_PIN     17
#define BTN_DOWN_PIN   19
#define BTN_LEFT_PIN   20
#define BTN_RIGHT_PIN  18

#define BTN_A_PIN      22
#define BTN_B_PIN      23

#define BTN_START_WAKE_PIN    1
#define BTN_SEL_PIN           16

#define VBAT_MON_PIN 2

// Set to 1 to run the app on SW/USB reset without GPIO wake (useful for idf.py flash/monitor).
#define DEEP_SLEEP_ALLOW_DEBUG_RESET  1

#define WIFI_CHANNEL       1
#define BTN_DEBOUNCE_MS    15
#define PAIR_RETRY_MS      5000
#define IDLE_DEEP_SLEEP_MS (2 * 60 * 1000)
// TX power in 0.25 dBm steps: 80=20 dBm (max), 52=13 dBm, 40=10 dBm. Lower reduces peak current.
#define WIFI_TX_POWER      52

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
static uint8_t receiver_mac[6] = {0xE8, 0x9F, 0x6D, 0x21, 0x8F, 0xEC};  // Display's MAC address
//static uint8_t receiver_mac[6] = {0x98, 0xA3, 0x16, 0x8F, 0x0B, 0xF0};  // Test receiver's MAC address 98:A3:16:8F:0B:F0
static bool is_paired = false;
static esp_now_peer_info_t peer_info;
static bool is_running = false;
static TaskHandle_t controller_task_handle = NULL;
static TickType_t last_button_activity_ticks = 0;

// Function declarations
static void init_buttons(void);
static void try_pair_peer(void);
static uint8_t read_button_state(void);
static void button_isr_handler(void *arg);
static void init_user_led(void);
static void init_wifi(void);
static void init_espnow(void);
static void send_button_states(void);
static TickType_t get_idle_wait_ticks(void);
static void controller_task(void *pvParameters);
static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

static void configure_gpio_sleep_input(gpio_num_t gpio_num)
{
    ESP_ERROR_CHECK(gpio_reset_pin(gpio_num));
    ESP_ERROR_CHECK(gpio_sleep_sel_en(gpio_num));
    ESP_ERROR_CHECK(gpio_sleep_set_direction(gpio_num, GPIO_MODE_DISABLE));
    ESP_ERROR_CHECK(gpio_sleep_set_pull_mode(gpio_num, GPIO_FLOATING));
}

bool controller_booted_from_gpio_wake(void)
{
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
        return true;
    }

#if DEEP_SLEEP_ALLOW_DEBUG_RESET
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_SW || reason == ESP_RST_USB) {
        return true;
    }
#endif

    return false;
}

void controller_enter_deep_sleep_until_wake(void)
{
    gpio_config_t wake_conf = {
        .pin_bit_mask = (1ULL << BTN_START_WAKE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&wake_conf);

    while (gpio_get_level(BTN_START_WAKE_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(esp_deep_sleep_enable_gpio_wakeup(
        1ULL << BTN_START_WAKE_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH));

    static const gpio_num_t button_pins[] = {
        BTN_UP_PIN, BTN_DOWN_PIN, BTN_LEFT_PIN, BTN_RIGHT_PIN,
        BTN_A_PIN, BTN_B_PIN, BTN_SEL_PIN, VBAT_MON_PIN,
    };
    for (size_t i = 0; i < sizeof(button_pins) / sizeof(button_pins[0]); i++) {
        configure_gpio_sleep_input(button_pins[i]);
    }

    gpio_set_level(LED_USER_PIN, 0);
    ESP_ERROR_CHECK(gpio_hold_en(LED_USER_PIN));

    ESP_LOGI(TAG, "Entering deep sleep, wake on GPIO%d high", BTN_START_WAKE_PIN);
    esp_deep_sleep_start();
}

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

static void IRAM_ATTR button_isr_handler(void *arg)
{
    (void)arg;
    BaseType_t wakeup = pdFALSE;
    if (controller_task_handle != NULL) {
        vTaskNotifyGiveFromISR(controller_task_handle, &wakeup);
    }
    if (wakeup) {
        portYIELD_FROM_ISR();
    }
}

static void init_buttons(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_UP_PIN) | (1ULL << BTN_DOWN_PIN) |
                        (1ULL << BTN_LEFT_PIN) | (1ULL << BTN_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_UP_PIN, button_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_DOWN_PIN, button_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_LEFT_PIN, button_isr_handler, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_RIGHT_PIN, button_isr_handler, NULL));
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
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(WIFI_TX_POWER));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_LOGI(TAG, "WiFi TX power: %.1f dBm", WIFI_TX_POWER * 0.25f);
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

static uint8_t read_button_state(void)
{
    uint8_t state = 0;
    if (gpio_get_level(BTN_UP_PIN) == 1) state |= (1 << BTN_UP_BIT);
    if (gpio_get_level(BTN_DOWN_PIN) == 1) state |= (1 << BTN_DOWN_BIT);
    if (gpio_get_level(BTN_LEFT_PIN) == 1) state |= (1 << BTN_LEFT_BIT);
    if (gpio_get_level(BTN_RIGHT_PIN) == 1) state |= (1 << BTN_RIGHT_BIT);
    return state;
}

static void try_pair_peer(void)
{
    esp_now_del_peer(peer_info.peer_addr);

    esp_err_t result = esp_now_add_peer(&peer_info);
    if (result == ESP_OK) {
        is_paired = true;
        gpio_set_level(LED_USER_PIN, 1);
        ESP_LOGI(TAG, "Successfully paired with display");
    } else {
        gpio_set_level(LED_USER_PIN, 0);
        ESP_LOGW(TAG, "Failed to pair (error %d), will retry in %d seconds",
                 result, PAIR_RETRY_MS / 1000);
    }
}

static void send_button_states(void) {
    static uint8_t last_button_state = 0;
    uint8_t current_button_state = read_button_state();

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

static TickType_t get_idle_wait_ticks(void)
{
    TickType_t elapsed = xTaskGetTickCount() - last_button_activity_ticks;
    TickType_t limit = pdMS_TO_TICKS(IDLE_DEEP_SLEEP_MS);

    if (elapsed >= limit) {
        return 0;
    }

    TickType_t remaining = limit - elapsed;
    if (!is_paired) {
        TickType_t pair_ticks = pdMS_TO_TICKS(PAIR_RETRY_MS);
        if (remaining > pair_ticks) {
            remaining = pair_ticks;
        }
    }
    return remaining;
}

static void controller_task(void *pvParameters) {
    (void)pvParameters;
    controller_task_handle = xTaskGetCurrentTaskHandle();
    last_button_activity_ticks = xTaskGetTickCount();

    memcpy(peer_info.peer_addr, receiver_mac, 6);
    peer_info.channel = WIFI_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;

    while (is_running) {
        TickType_t wait_ticks = get_idle_wait_ticks();
        if (wait_ticks == 0) {
            gpio_set_level(LED_USER_PIN, 0);
            ESP_LOGI(TAG, "Idle for %d s, entering deep sleep", IDLE_DEEP_SLEEP_MS / 1000);
            controller_enter_deep_sleep_until_wake();
        }

        uint32_t notified = ulTaskNotifyTake(pdTRUE, wait_ticks);

        if (notified > 0) {
            last_button_activity_ticks = xTaskGetTickCount();
            vTaskDelay(pdMS_TO_TICKS(BTN_DEBOUNCE_MS));
        }

        if (!is_paired) {
            try_pair_peer();
        }

        send_button_states();
    }

    controller_task_handle = NULL;
    vTaskDelete(NULL);
}