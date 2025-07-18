#line 1 "/Users/admin/Documents/Arduino/robot/README.md"
# Robot Project

Arduino-based robot with VIA "banh mi" v2023 board (ESP32-WROOM-32E) and PCA9685 motor control.

## Quick Start

### Lightning-Fast Development
```bash
./lightning.sh check     # Syntax check (~7s)
./lightning.sh compile   # Full compile (~17s)
./lightning.sh upload    # Deploy to robot
./lightning.sh help      # Show all commands
```

### Development Workflow
1. **Code** - Make changes to robot code
2. **Check** - `./lightning.sh check` for rapid syntax validation
3. **Compile** - `./lightning.sh compile` when ready to test
4. **Upload** - `./lightning.sh upload` to deploy to robot

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
- **Input Validation**: Hardware protection from invalid parameters
- **Constants**: Named constants instead of magic numbers
- **Function Decomposition**: Small, focused, maintainable functions
- **Error Handling**: Robust validation and automatic recovery

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
