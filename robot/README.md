# Robot Project

Arduino-based robot with VIA "banh mi" v2023 board (ESP32-WROOM-32E) and PCA9685 motor control.

## Quick Start

### Fast Development Commands
```bash
# Syntax check (~7s)
arduino-cli compile --fqbn esp32:esp32:esp32 --verify --quiet .

# Full compile (~17s) 
arduino-cli compile --fqbn esp32:esp32:esp32 --optimize-for-debug .

# Upload to robot
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/cu.usbserial-* .
```

### Development Workflow
1. **Code** - Make changes to robot code
2. **Check** - Fast syntax validation with `--verify --quiet`
3. **Compile** - Full compilation when ready to test
4. **Upload** - Deploy to robot with auto-port detection

## Features
- **PS2 Controller**: Wireless input with connection monitoring
- **Drive System**: Dual motor drive with precision/normal modes
- **Outtake System**: Dual motor outtake with automatic sequences
- **Servo Control**: 4 servos with safety validation
- **State Machine**: Manual control + autonomous sequences with timeout protection
- **Safety Systems**: Limit switch protection, input validation, error recovery
- **User Feedback**: LED and buzzer status indicators

## Code Quality
- **PWM Utilities**: Centralized calculations, no code duplication
- **Direct Function Calls**: Simplified hardware control without validation wrappers
- **Constants**: Named constants instead of magic numbers
- **Function Decomposition**: Small, focused, maintainable functions
- **Error Handling**: Robust safety monitoring and automatic recovery

## Hardware
- **Board**: VIA "banh mi" v2023 (ESP32-WROOM-32E)
- **Motor Control**: PCA9685 at 50Hz for servo compatibility
- **Available IO**: IO32, IO2, IO0, IO25, IO39, IO36
- **Sensors**: Single limit switch (expandable)
- **Feedback**: RGB LED (GPIO27), buzzer (GPIO25)

## Memory Usage
- **Flash**: ~340KB (25% of ESP32)
- **RAM**: ~22.5KB (7% of ESP32)
- **Performance**: 10ms loop timing maintained

## Configuration
All tuning parameters are in `src/config.h`:
- Motor speeds and ramping
- Servo angles and timing
- Controller deadzone and sensitivity
- Safety timeouts and limits
