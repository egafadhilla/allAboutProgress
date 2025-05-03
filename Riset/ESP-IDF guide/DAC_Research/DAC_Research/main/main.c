/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

 #include <inttypes.h>
 #include <string.h>
 #include "freertos/FreeRTOS.h"
 #include "driver/dac_oneshot.h"

void app_main() {
    // Konfigurasi channel DAC
    uint8_t value = 0;
    dac_oneshot_config_t config = {
        .chan_id = DAC_CHAN_0,  // GPIO25 (Channel 1)
    };
    
    // Inisialisasi DAC
    dac_oneshot_handle_t handle;
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&config, &handle));
    
    // Set nilai DAC (0-255)
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(handle, 128));  // Output ~1.65V
    
    // Tetap aktif sampai dihapus
    while(1) {
        value+=17;
        dac_oneshot_output_voltage(handle, (value%255));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Cleanup (opsional)
    ESP_ERROR_CHECK(dac_oneshot_del_channel(handle));
}