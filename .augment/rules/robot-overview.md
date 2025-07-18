---
type: "always_apply"
description: "Comprehensive overview of the Arduino robot project architecture, features, utilities, and development practices. Use when discussing any aspect of the robot codebase."
---
# Arduino Robot Project Overview

## Hardware Platform
- **Board**: VIA "banh mi" v2023 (ESP32-WROVER-IE-N4R8)
- **Motor Control**: PCA9685 power board at 50Hz (servo compatibility priority)
- **Drivetrain**: 6-wheel drive (2 middle friction wheels + 4 corner omni wheels)
- **Available IO Pins**: IO32, IO2, IO0, IO25, IO39, IO36 (accessible on power board)
- **Input**: PS2 controller with wireless connection monitoring
- **Sensors**: Single limit switch with debouncing (limited sensor setup)
- **Feedback**: Onboard RGB LED (GPIO27) only (no buzzer hardware present)

## Architecture
- **Main Entry**: `robot.ino` (Arduino setup/loop functions)
- **Modular Design**:
  - Controller layer: PS2 controller input handling with debouncing
  - Robot logic: State machine with timeout protection
  - Hardware abstraction: Motors, servos, PCA9685 driver
  - Configuration: Centralized settings in `src/config.h`
  - Utilities: PWM calculations, input validation, constants
  - System monitoring: Health checks and user feedback

## Key Features
- **Input**: PS2 controller with joystick and button mapping, connection monitoring
- **Drive System**: 6-wheel drive (2 middle friction + 4 corner omni wheels) with FTC-style electromagnetic braking
- **Motor Control**: Two-stage deceleration (ramp down 75ms then brake), 100ms direction change braking, 4095 PWM range with 55% speed cap
- **Brake Logic**: Thresholds calculated from actual max driving speed (2252 PWM) not theoretical max, based on motor PWM not controller input
- **Outtake System**: Dual motor outtake with automatic sequences and safety limits
- **Servo Control**: 4 servos for intake/outtake arm positioning with validation
- **State Machine**: Manual control, automatic outtake, timed sequences with timeout protection
- **Safety**: Limit switch protection, input validation, state timeouts, system health monitoring
- **User Feedback**: LED and buzzer status indicators for system state

## Utility Classes
### PWM Utilities (`src/utils/pwm_utils.h`)
- Centralized PWM calculations to eliminate code duplication
- `getMaxPWM()` / `getPrecisionPWM()` - Standard PWM values (uses DRIVE_MAX_SPEED_PERCENT/DRIVE_PRECISION_SPEED_PERCENT)
- `calculateTurnOffset()` - Turn calculation (uses TURN_SENSITIVITY_PERCENT)
- `scaleJoystickToPWM()` - Input scaling with validation
- `applyDeadzone()` - Joystick deadzone processing

### Input Validation (`src/utils/validation.h`)
- Prevent crashes and hardware damage from invalid inputs
- `setServoAngleSafe()` - Safe servo control with validation
- `setMotorSpeedSafe()` - Motor speed validation and clamping
- Parameter validation and automatic bounds correction

### Constants (`src/utils/constants.h`)
- Named constants replacing magic numbers
- `MOTOR_STOP_SPEED`, `SERVO_MIN_ANGLE`, `TIMEOUT_SAFETY_MARGIN_MS`
- Organized by category (PWM, servo, timing, safety)

## Code Organization
### Robot Class Structure
- **Main Control**: `handleDriveInput()`, `handleOuttakeInput()`, `handleServoInput()`
- **Helper Functions**: `calculateDriveMotorSpeeds()`, `processJoystickInputs()`, `stopAllMotors()`
- **State Management**: `isStateTimedOut()`, `handleStateTimeout()`, `getStateTimeoutMs()`
- **Safety**: Input validation, timeout protection, error recovery

### Development Principles
- **DRY**: No code duplication, centralized utilities
- **Single Responsibility**: Each function has one clear purpose
- **Defensive Programming**: All inputs validated, automatic recovery
- **Maintainability**: Small functions, named constants, clear separation

## Development Guidelines
### When Adding New Features
1. Use existing utility functions (PWMUtils, Validation, Constants)
2. Add validation for all new parameters
3. Keep functions small and focused (5-10 lines)
4. Add timeout protection for new states
5. Use named constants instead of magic numbers

### When Modifying Code
1. Maintain decomposed function structure
2. Use validation utilities for hardware control
3. Test with compilation after changes
4. Preserve error handling patterns
5. Update constants file for new values

### Best Practices
- **Hardware Control**: Use direct `setServoAngle()` and `PWMUtils::clampPWM()` functions
- **PWM Calculations**: Use `PWMUtils` functions instead of direct calculations
- **Constants**: Define in `config` namespace, avoid raw numbers
- **Error Handling**: Provide clear error messages and automatic recovery
- **State Machine**: Include timeout handling for autonomous states

## Technical Specifications
- **Memory Usage**: ~340KB flash (25%), ~22.5KB RAM (7%)
- **Compatibility**: ESP32-WROOM-32E, PCA9685 at 50Hz, PS2X library

## Development Notes
- **Compilation**: Always compile after major changes to catch errors early
- **Arduino CLI**: Use `arduino-cli compile --fqbn esp32:esp32:esp32 --verify --quiet .` for fast syntax checks
- **Linting**: Arduino.h/uint8_t linter errors can be ignored - code compiles correctly
- **Testing**: Hardware-dependent - requires physical robot for full validation
- **Configuration**: All tuning parameters centralized in `src/config.h`
- **Limitations**: Limited sensors (no motor encoders), 50Hz PCA9685 constraint
  - **PWM Frequency**: PCA9685 max 1,526Hz vs optimal 1,600Hz for motors
  - **Performance Trade-off**: 50Hz servo compatibility vs motor responsiveness
- **User Preference**: Code-focused improvements over hardware-dependent features

## Compilation Commands
- **Fast Syntax Check**: `arduino-cli compile --fqbn esp32:esp32:esp32 --verify --quiet .`
- **Full Compilation**: `arduino-cli compile --fqbn esp32:esp32:esp32 --optimize-for-debug .`
- **Memory Usage**: Program: ~342KB (26%), RAM: ~22.5KB (7%)
- **Development Workflow**: Rapid iteration with syntax validation

## Related Documentation
- **VIA Board Specifications**: See `.augment/rules/via-banhmi-v2023.md` for controller board details, GPIO pins, and channel limitations
- **Power Board Configuration**: See `.augment/rules/power-board.md` for complete power board architecture with 1x PCA9685 + 4x TA6586 ICs, electromagnetic braking, and IO connections
- **Development Guidelines**: This file contains the primary robot architecture and development practices