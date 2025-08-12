## ESP32-S3 Half-duplex UART for Dynamixel AX-12A (ESP-IDF 5.4.0)

This example shows how to talk to a Dynamixel AX-12A servo using a single-wire, half-duplex UART bus on the ESP32-S3 with ESP-IDF 5.4.0.

It implements a minimal Protocol 1.0 driver and demonstrates ping, torque enable, LED, set goal position, and read present position.

### Hardware

- ESP32-S3 board
- Dynamixel AX-12A (TTL 5V)
- Power supply 12V for the servo (shared ground with ESP32)
- Half-duplex interface between ESP32 and Dynamixel bus. Options:
	- A ready-made TTL half-duplex adapter for Dynamixel.
	- Or build with a buffer/driver (e.g., 74HC125/126 or a transistor + resistors) so that TX and RX share the same Data line safely. Avoid connecting ESP32 pins directly to the 5V bus.

Default pins in code:

- Data: GPIO17 (both TX and RX mapped to same pin)
- Direction (TX enable): GPIO18 (HIGH=TX drive, LOW=RX Hi-Z)
- UART used: UART1

Adjust these in `main/main.c` to match your wiring.

### Build and flash

Requires ESP-IDF 5.4.0. Set the target to esp32s3 and flash:

```powershell
idf.py set-target esp32s3 ; idf.py build ; idf.py -p COMx flash monitor

Replace COMx with your port.

### Notes

- AX-12A default: ID=1, baud=1,000,000 bps.
- Ensure ground is shared between ESP32 and the servo power supply.
- Status Return Level on AX-12A normally returns status for all instructions; this code reads and validates the response.
- If you need another baud or ID, use the helper functions in `dynamixel_ax12a.h` (e.g., `ax_set_baud`, `ax_set_id`). Changing these requires torque off.

### Safety

Provide proper 5V TTL interface and isolation to avoid damaging the ESP32 (which is 3.3V) and ensure the servo has adequate power.
