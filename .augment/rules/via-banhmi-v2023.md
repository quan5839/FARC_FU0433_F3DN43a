---
type: "agent_requested"
description: "Technical summary and pinout for the VIA "banh mi" v2023 board, for reference when working with hardware, pin mapping, or board-specific code. globs:"
---
# VIA "banh mi" v2023 Board – Technical Overview

- **Processor:** ESP32-WROVER-IE-N4R8 
- **CPU:** Xtensa dual-core 32-bit LX6, 240MHz (overclockable to 256MHz)
- **WIFI** IEEE 802.11 b/g/n, up to 150 Mbps, ~20.5 dBm transmit power, –98 dBm receive sensitivity
- **Bluetooth:** Bluetooth v4.2 + EDR (classic) and BLE support
- **Antenna:** External antenna via I-PEX/U.FL connector (no on-board PCB antenna)
- **SPI Flash:** 4MB
- **PSRAM:** 8 MB PSRAM
- **USB:** USB-C (programming, power, serial)
- **Power:** 5V via USB-C or VIN pin, 3.3V LDO regulator onboard
- **GPIOs:** 21 user-accessible pins (3.3V logic)
- **Analog Inputs:** 6 (ADC1: 32, 33, 34, 35, 36, 39)
- **PWM Outputs:** All GPIOs support PWM
- **I2C:** Default SDA (21), SCL (22)
- **SPI:** Default MOSI (23), MISO (19), SCK (18), CS (5)
- **UART:** Default TX (1), RX (3)
- **Special Pins:**
  - BOOT (GPIO0): Hold LOW to enter bootloader (black button on board)
  - EN (Reset): Active LOW
- **Onboard Peripherals:**
  - RGB LED (WS2812, GPIO27)
  - White side button SW1 (location: side of board)
- **Expansion:** Standard VIA pin headers, Grove connectors

- **Pinout Reference:**
  - See [VIA documentation](mdc:https:/via.makerviet.org/vi/docs/2_get_started_with_via_hardware/1_gioi-thieu-mach-via-b) for full pinout diagram and details.

- **Notes:**
  - All GPIOs are 3.3V only (do not apply 5V).
  - Use BOOT button for flashing firmware.
  - For Arduino, select "ESP32 Dev Module" as the board.

## Power Board Integration

The VIA "banh mi" v2023 typically uses a PCA9685-based power board for motor and servo control:
See `.augment/rules/via-banhmi-v2023.md` for controller board details, GPIO pins, and channel

### VIA "banh mi" v2023 Channel Limitations

**IMPORTANT**: Not all PCA9685 channels are available for all purposes on the VIA power board:

- **Servo Channels**: Only 2-7 available (6 servo ports)
- **Motor Channels**: Only 8-15 available (8 motor ports)
- **Channels 0-1**: May be unused or reserved
- **Total**: 6 servo + 8 motor = 14 usable channels out of 16

### Validation Requirements
Code must validate channel usage:
```cpp
// Correct validation for VIA "banh mi" v2023
bool isValidServoChannel(uint8_t channel) {
    return (channel >= 2 && channel <= 7);  // Only 2-7
}

bool isValidMotorChannel(uint8_t channel) {
    return (channel >= 8 && channel <= 15); // Only 8-15
}
```
