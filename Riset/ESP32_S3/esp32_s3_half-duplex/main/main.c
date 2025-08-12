#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dynamixel_ax12a.h"
#include "driver/uart.h"

// Adjust these to your wiring
#define DXL_UART_NUM   UART_NUM_1
#define DXL_GPIO_DATA  17   // Example: connect to Dynamixel Data via proper half-duplex interface
#define DXL_GPIO_DIR   18   // Direction pin to enable TX driver (HIGH=TX, LOW=RX)

static const char *TAG = "APP";

void app_main(void)
{
	dxl_port_t port = {
		.uart_num = DXL_UART_NUM,
		.gpio_signal = DXL_GPIO_DATA,
		.gpio_dir = DXL_GPIO_DIR,
		.baudrate = DXL_BAUD_DEFAULT, // 1 Mbps default for AX-12A
		.tx_buffer_size = 256,
		.rx_buffer_size = 256,
		.timeout_ms = 100,
	};

	if (dxl_init(&port) != ESP_OK) {
		ESP_LOGE(TAG, "DXL init failed");
		return;
	}

	uint8_t mdl_l=0, mdl_h=0, fw=0;
	esp_err_t err = dxl_ping(&port, 1 /* default ID */, &mdl_l, &mdl_h, &fw);
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "Ping OK: model=%u (0x%02X%02X) fw=%u", (uint16_t)(mdl_h<<8|mdl_l), mdl_h, mdl_l, fw);
	} else {
		ESP_LOGW(TAG, "Ping failed, check wiring/baud/id");
	}

	// Enable torque
	ax_torque_enable(&port, 1, true);
	// Optional: turn LED on
	ax_led_set(&port, 1, true);
	// Set speed (optional)
	ax_set_moving_speed(&port, 1, 200);

	// Sweep a couple positions
	const uint16_t poses[] = { 200, 512, 800 };
	for (int i = 0; i < 6; ++i) {
		uint16_t p = poses[i % 3];
		ax_set_goal_position(&port, 1, p);
		vTaskDelay(pdMS_TO_TICKS(800));
		uint16_t now=0;
		if (ax_get_present_position(&port, 1, &now) == ESP_OK) {
			ESP_LOGI(TAG, "Present pos=%u", now);
		}
	}

	// Turn LED off and torque off
	ax_led_set(&port, 1, false);
	ax_torque_enable(&port, 1, false);

	// Keep running idle
	while (1) {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}