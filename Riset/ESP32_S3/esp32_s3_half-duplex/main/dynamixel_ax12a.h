#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Protocol 1.0 for Dynamixel AX-12A
// Half-duplex UART on one wire with external 5V TTL level and a direction/enable pin (DE/RE).
// This driver uses ESP-IDF uart driver and a GPIO to control TX enable.

// Default AX-12A parameters
#define DXL_PROTOCOL_VER       1
#define DXL_BAUD_DEFAULT       1000000  // 1 Mbps default for AX-12A

// Common instructions (Protocol 1.0)
#define DXL_INST_PING          0x01
#define DXL_INST_READ          0x02
#define DXL_INST_WRITE         0x03
#define DXL_INST_REG_WRITE     0x04
#define DXL_INST_ACTION        0x05
#define DXL_INST_RESET         0x06
#define DXL_INST_SYNC_WRITE    0x83

// AX-12A Control Table (partial)
#define AX_ID_REG_ID               0x03
#define AX_ID_REG_BAUD_RATE        0x04
#define AX_ID_REG_CW_LIMIT_L       0x06
#define AX_ID_REG_CCW_LIMIT_L      0x08
#define AX_ID_REG_TORQUE_ENABLE    0x18
#define AX_ID_REG_LED              0x19
#define AX_ID_REG_GOAL_POSITION_L  0x1E
#define AX_ID_REG_MOVING_SPEED_L   0x20
#define AX_ID_REG_PRESENT_POSITION_L 0x24

// Handle/config
typedef struct {
    int uart_num;         // UART_NUM_0..2
    int gpio_signal;      // The single-wire bus pin (connected to Dynamixel Data)
    int gpio_dir;         // Direction control pin for TX enable (HIGH = TX drive, LOW = RX Hi-Z)
    int baudrate;         // UART baud
    int tx_buffer_size;   // UART TX buffer bytes
    int rx_buffer_size;   // UART RX buffer bytes
    int timeout_ms;       // Read timeout
} dxl_port_t;

// Public API
esp_err_t dxl_init(dxl_port_t *port);
void dxl_deinit(const dxl_port_t *port);

// Basic protocol helpers
esp_err_t dxl_ping(const dxl_port_t *port, uint8_t id, uint8_t *model_low, uint8_t *model_high, uint8_t *fw);
esp_err_t dxl_read_data(const dxl_port_t *port, uint8_t id, uint8_t addr, uint8_t len, uint8_t *out);
esp_err_t dxl_write_data(const dxl_port_t *port, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t len);

// Convenience helpers for AX-12A
esp_err_t ax_set_id(const dxl_port_t *port, uint8_t old_id, uint8_t new_id);
esp_err_t ax_set_baud(const dxl_port_t *port, uint8_t id, uint32_t baud);
esp_err_t ax_torque_enable(const dxl_port_t *port, uint8_t id, bool enable);
esp_err_t ax_led_set(const dxl_port_t *port, uint8_t id, bool on);
esp_err_t ax_set_goal_position(const dxl_port_t *port, uint8_t id, uint16_t position_0_1023);
esp_err_t ax_set_moving_speed(const dxl_port_t *port, uint8_t id, uint16_t speed_0_1023);
esp_err_t ax_get_present_position(const dxl_port_t *port, uint8_t id, uint16_t *position_out);
esp_err_t ax_set_goal_angle_deg(const dxl_port_t *port, uint8_t id, float degrees_0_to_300);

#ifdef __cplusplus
}
#endif
