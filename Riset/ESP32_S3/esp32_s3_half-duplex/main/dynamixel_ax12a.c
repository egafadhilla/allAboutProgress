#include "dynamixel_ax12a.h"
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h" // ets_delay_us
#include "esp_timer.h"

static const char *TAG = "DXL";

static inline void dir_tx(const dxl_port_t *p, bool en)
{
    gpio_set_level(p->gpio_dir, en ? 1 : 0);
}

static uint8_t dxl_checksum(const uint8_t *pkt, size_t len)
{
    // For Protocol 1.0, checksum = ~ (ID + LENGTH + INSTRUCTION + PARAMS)
    uint16_t sum = 0;
    for (size_t i = 2; i < len; ++i) { // start at ID index
        sum += pkt[i];
    }
    return (uint8_t)(~(sum & 0xFF));
}

static esp_err_t dxl_txrx(const dxl_port_t *p, const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_max, size_t *rx_len)
{
    // Drive TX
    dir_tx(p, true);
    // small guard time before sending
    esp_rom_delay_us(2);

    int written = uart_write_bytes(p->uart_num, (const char *)tx, tx_len);
    if (written != (int)tx_len) {
        ESP_LOGE(TAG, "uart_write_bytes %d/%d", written, (int)tx_len);
        dir_tx(p, false);
        return ESP_FAIL;
    }
    uart_wait_tx_done(p->uart_num, pdMS_TO_TICKS(20));

    // Switch to RX (release bus)
    dir_tx(p, false);

    // Read response (if any)
    if (!rx || rx_max == 0) {
        if (rx_len) *rx_len = 0;
        return ESP_OK;
    }

    // Wait for header 0xFF 0xFF
    uint8_t b;
    int64_t start = esp_timer_get_time();
    int state = 0;
    while ((esp_timer_get_time() - start) / 1000 < p->timeout_ms) {
        int r = uart_read_bytes(p->uart_num, &b, 1, pdMS_TO_TICKS(5));
        if (r == 1) {
            if (state == 0 && b == 0xFF) state = 1;
            else if (state == 1 && b == 0xFF) { state = 2; break; }
            else state = 0;
        }
    }
    if (state != 2) {
        ESP_LOGW(TAG, "No header");
        return ESP_ERR_TIMEOUT;
    }
    // Header found; read rest until checksum
    uint8_t hdr[3]; // ID, LENGTH, ERROR
    int r = uart_read_bytes(p->uart_num, hdr, 3, pdMS_TO_TICKS(p->timeout_ms));
    if (r != 3) return ESP_ERR_TIMEOUT;
    uint8_t id = hdr[0];
    uint8_t length = hdr[1]; // includes error + params + checksum
    uint8_t error = hdr[2];
    size_t param_len = length - 2; // params only
    if (2 + 3 + param_len + 1 > rx_max) {
        // Not enough space; still drain
        ESP_LOGW(TAG, "RX buffer small");
    }

    size_t total_to_read = param_len + 1; // params + checksum
    size_t got = uart_read_bytes(p->uart_num, rx, total_to_read, pdMS_TO_TICKS(p->timeout_ms));
    if (got != total_to_read) return ESP_ERR_TIMEOUT;

    // Build a full packet for checksum verify
    uint8_t tmp[6 + 64];
    size_t build_len = 0;
    tmp[build_len++] = 0xFF;
    tmp[build_len++] = 0xFF;
    tmp[build_len++] = id;
    tmp[build_len++] = length;
    tmp[build_len++] = error;
    memcpy(&tmp[build_len], rx, param_len);
    build_len += param_len;
    uint8_t cs = rx[param_len]; // last byte in rx
    uint8_t calc = dxl_checksum(tmp, build_len);
    if (cs != calc) {
        ESP_LOGW(TAG, "Bad checksum exp=%02X got=%02X", calc, cs);
        return ESP_ERR_INVALID_CRC;
    }

    if (error != 0) {
        ESP_LOGW(TAG, "DXL error: 0x%02X", error);
    }

    if (rx_len) *rx_len = param_len; // only params length
    // Shift params to front for caller convenience
    memmove(rx, rx, param_len);
    return ESP_OK;
}

esp_err_t dxl_init(dxl_port_t *p)
{
    uart_config_t cfg = {
        .baud_rate = p->baudrate > 0 ? p->baudrate : DXL_BAUD_DEFAULT,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(p->uart_num, p->rx_buffer_size, p->tx_buffer_size, 0, NULL, 0));

    // Use same GPIO for TX and RX to emulate single-wire; TXD and RXD both connected to the same pin via external circuit (typical 74HC125/126 or transistor pair)
    // If using a ready-made half-duplex adapter, set TXD to the data pin and RXD to the same pin; ensure external circuitry handles direction.
    ESP_ERROR_CHECK(uart_param_config(p->uart_num, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(p->uart_num, p->gpio_signal, p->gpio_signal, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Direction pin
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << p->gpio_dir,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    dir_tx(p, false); // start in RX

    // Clear buffers
    uart_flush(p->uart_num);

    if (p->timeout_ms <= 0) p->timeout_ms = 100; // default

    ESP_LOGI(TAG, "DXL init UART%d pin=%d dir=%d baud=%d", p->uart_num, p->gpio_signal, p->gpio_dir, cfg.baud_rate);
    return ESP_OK;
}

void dxl_deinit(const dxl_port_t *p)
{
    uart_driver_delete(p->uart_num);
}

esp_err_t dxl_ping(const dxl_port_t *p, uint8_t id, uint8_t *model_low, uint8_t *model_high, uint8_t *fw)
{
    uint8_t pkt[6 + 2];
    size_t len = 0;
    pkt[len++] = 0xFF; pkt[len++] = 0xFF; // header
    pkt[len++] = id;
    pkt[len++] = 2; // LENGTH = 2 (instruction + checksum)
    pkt[len++] = DXL_INST_PING;
    {
        uint8_t cs = dxl_checksum(pkt, len);
        pkt[len++] = cs;
    }

    uint8_t resp[16]; size_t rlen = 0;
    esp_err_t err = dxl_txrx(p, pkt, len, resp, sizeof(resp), &rlen);
    if (err != ESP_OK) return err;
    // Response params: model_l, model_h, firmware
    if (rlen >= 3) {
        if (model_low) *model_low = resp[0];
        if (model_high) *model_high = resp[1];
        if (fw) *fw = resp[2];
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t dxl_read_data(const dxl_port_t *p, uint8_t id, uint8_t addr, uint8_t len_to_read, uint8_t *out)
{
    uint8_t pkt[6 + 4];
    size_t len = 0;
    pkt[len++] = 0xFF; pkt[len++] = 0xFF;
    pkt[len++] = id;
    pkt[len++] = 4; // LENGTH = 4 (instruction + addr + len + checksum)
    pkt[len++] = DXL_INST_READ;
    pkt[len++] = addr;
    pkt[len++] = len_to_read;
    {
        uint8_t cs = dxl_checksum(pkt, len);
        pkt[len++] = cs;
    }

    size_t rlen = 0;
    esp_err_t err = dxl_txrx(p, pkt, len, out, len_to_read + 4, &rlen); // allow margin
    if (err != ESP_OK) return err;
    if (rlen == len_to_read) return ESP_OK;
    return ESP_FAIL;
}

esp_err_t dxl_write_data(const dxl_port_t *p, uint8_t id, uint8_t addr, const uint8_t *data, uint8_t dlen)
{
    uint8_t pkt[6 + 16];
    size_t len = 0;
    pkt[len++] = 0xFF; pkt[len++] = 0xFF;
    pkt[len++] = id;
    pkt[len++] = (uint8_t)(3 + dlen); // LENGTH = 3 + N (instruction + addr + params + checksum)
    pkt[len++] = DXL_INST_WRITE;
    pkt[len++] = addr;
    memcpy(&pkt[len], data, dlen);
    len += dlen;
    {
        uint8_t cs = dxl_checksum(pkt, len);
        pkt[len++] = cs;
    }

    // AX generally replies with a status packet (unless Status Return Level is 1/2 specifics). We'll read but ignore params.
    uint8_t resp[8]; size_t rlen = 0;
    return dxl_txrx(p, pkt, len, resp, sizeof(resp), &rlen);
}

// Convenience wrappers
esp_err_t ax_set_id(const dxl_port_t *p, uint8_t old_id, uint8_t new_id)
{
    return dxl_write_data(p, old_id, AX_ID_REG_ID, &new_id, 1);
}

esp_err_t ax_set_baud(const dxl_port_t *p, uint8_t id, uint32_t baud)
{
    // AX-12A uses baud = 2000000 / (value + 1)
    uint8_t val;
    if (baud == 0) return ESP_ERR_INVALID_ARG;
    uint32_t v = (2000000 / baud) - 1;
    if (v > 255) return ESP_ERR_INVALID_ARG;
    val = (uint8_t)v;
    return dxl_write_data(p, id, AX_ID_REG_BAUD_RATE, &val, 1);
}

esp_err_t ax_torque_enable(const dxl_port_t *p, uint8_t id, bool enable)
{
    uint8_t v = enable ? 1 : 0;
    return dxl_write_data(p, id, AX_ID_REG_TORQUE_ENABLE, &v, 1);
}

esp_err_t ax_led_set(const dxl_port_t *p, uint8_t id, bool on)
{
    uint8_t v = on ? 1 : 0;
    return dxl_write_data(p, id, AX_ID_REG_LED, &v, 1);
}

esp_err_t ax_set_goal_position(const dxl_port_t *p, uint8_t id, uint16_t position)
{
    uint8_t v[2] = { (uint8_t)(position & 0xFF), (uint8_t)((position >> 8) & 0x03) };
    return dxl_write_data(p, id, AX_ID_REG_GOAL_POSITION_L, v, 2);
}

esp_err_t ax_set_moving_speed(const dxl_port_t *p, uint8_t id, uint16_t speed)
{
    uint8_t v[2] = { (uint8_t)(speed & 0xFF), (uint8_t)((speed >> 8) & 0x03) };
    return dxl_write_data(p, id, AX_ID_REG_MOVING_SPEED_L, v, 2);
}

esp_err_t ax_get_present_position(const dxl_port_t *p, uint8_t id, uint16_t *position_out)
{
    uint8_t buf[2];
    esp_err_t err = dxl_read_data(p, id, AX_ID_REG_PRESENT_POSITION_L, 2, buf);
    if (err != ESP_OK) return err;
    *position_out = (uint16_t)(buf[0] | ((buf[1] & 0x03) << 8));
    return ESP_OK;
}

esp_err_t ax_set_goal_angle_deg(const dxl_port_t *p, uint8_t id, float degrees)
{
    if (degrees < 0) degrees = 0;
    if (degrees > 300.0f) degrees = 300.0f;
    // 0..300 deg maps to 0..1023
    uint16_t pos = (uint16_t)((degrees / 300.0f) * 1023.0f + 0.5f);
    return ax_set_goal_position(p, id, pos);
}
