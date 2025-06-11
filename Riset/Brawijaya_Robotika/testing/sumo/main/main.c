#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ps3.h"  // Pastikan huruf kecil!

void app_main() {
    // Inisialisasi library PS3
    if (ps3Init() == ESP_OK) {
        printf("PS3 Controller Connected!\n");
    } else {
        printf("Failed to initialize PS3 Controller.\n");
    }

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay 1 detik
    }
}