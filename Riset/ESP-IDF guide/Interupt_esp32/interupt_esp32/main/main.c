#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"

// Definisi GPIO untuk LED RGB
#define LED_R_GPIO 2
#define LED_G_GPIO 4
#define LED_B_GPIO 5

void led_rgb_task(void *pvParameter)
{
    // Inisialisasi GPIO
    gpio_pad_select_gpio(LED_R_GPIO);
    gpio_set_direction(LED_R_GPIO, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_G_GPIO);
    gpio_set_direction(LED_G_GPIO, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(LED_B_GPIO);
    gpio_set_direction(LED_B_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Merah
        gpio_set_level(LED_R_GPIO, 1);
        gpio_set_level(LED_G_GPIO, 0);
        gpio_set_level(LED_B_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        // Hijau
        gpio_set_level(LED_R_GPIO, 0);
        gpio_set_level(LED_G_GPIO, 1);
        gpio_set_level(LED_B_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        // Biru
        gpio_set_level(LED_R_GPIO, 0);
        gpio_set_level(LED_G_GPIO, 0);
        gpio_set_level(LED_B_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    // Initialize serial debugging
    const char *TAG = "DEBUG";
    ESP_LOGI(TAG, "ESP32 Serial Debugging Started");
    // Buat task untuk blink LED RGB
    xTaskCreate(led_rgb_task, "led_rgb_task", 2048, NULL, 5, NULL);
    int counter = 0;
    while (1) {
        ESP_LOGI(TAG, "Counter value: %d", counter++);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 1 second
    }
}