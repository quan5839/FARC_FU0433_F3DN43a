#include "robot.h"
#include "../hal/servos.h"
#include "../hal/pca9685_driver.h"
#include "../hal/esp32_servo.h"
#include "../controller/PS2_controller.h"
#include "../utils/pwm_utils.h"
#include "../utils/safety_monitor.h"
#include <Arduino.h>
// Note: motor.h and state_machine.h are included via robot.h

// Performance optimization pragmas for main robot control
#pragma GCC optimize("O3")
#pragma GCC optimize("fast-math")

// --- Robot class implementation ---

Robot::Robot()
    : driveLeft(config::CHAN_DRIVE_L_FWD, config::CHAN_DRIVE_L_REV, config::tuning::INVERT_DRIVE_LEFT),
      driveRight(config::CHAN_DRIVE_R_FWD, config::CHAN_DRIVE_R_REV, config::tuning::INVERT_DRIVE_RIGHT),
      outtakeLeft(config::CHAN_OUTTAKE_L_FWD, config::CHAN_OUTTAKE_L_REV, config::constants::MOTOR_NOT_INVERTED),
      outtakeRight(config::CHAN_OUTTAKE_R_FWD, config::CHAN_OUTTAKE_R_REV, config::tuning::INVERT_OUTTAKE_RIGHT) {}

void Robot::init() {
    // Initialize safety monitoring
    SafetyMonitor::init();

    initPCA9685();
    // pinMode(config::LIMIT_SWITCH_PIN, INPUT_PULLUP);

    // Initialize state machine
    stateMachine.init();

    // Initialize servos (setServoAngle already constrains angles to 0-180)
    setServoAngle(config::INTAKE_SERVO_CHANNEL, config::tuning::INTAKE_ARM_CLOSE_ANGLE);

    // Initialize ESP32 direct servo control for outtake (GPIO 18, back pin 12)
    // ESP32Servo::init(config::OUTTAKE_SERVO_GPIO_PIN, config::servo::LEDC_CHANNEL);
    // ESP32Servo::setAngle(config::OUTTAKE_SERVO_GPIO_PIN, config::tuning::OUTTAKE_ARM_CLOSE_ANGLE);

    // SERVO PIN TEST: Output servo signals on multiple pins for testing
    const int test_pins[] = {0, 2, 4, 5, 16, 17, 18, 19, 23, 25, 26, 33};
    const int num_pins = sizeof(test_pins) / sizeof(test_pins[0]);

    DEBUG_PRINTLN("=== SERVO PIN TEST MODE ===");
    for (int i = 0; i < num_pins; i++) {
        int pin = test_pins[i];
        if (ledcAttach(pin, 50, 16)) {
            ledcWrite(pin, config::servo::LEDC_MIN_DUTY); // 90° center position
            DEBUG_PRINT("GPIO ");
            DEBUG_PRINT(pin);
            DEBUG_PRINTLN(" - SERVO ACTIVE");
        }
    }

    DEBUG_PRINTLN("Robot initialization complete");
}

void Robot::loop() {
    // Performance-optimized main loop with branch prediction
    SafetyMonitor::update();

    // Most likely path: system is healthy
    if (UNLIKELY(!SafetyMonitor::isSystemSafe() || !isSystemHealthy())) {
        handleSystemError();
        return;
    }

    // Most likely path: controller is connected
    if (UNLIKELY(!isConnected())) {
        setRobotState(config::IDLE);
    }

    // Most likely path: no timeout
    if (UNLIKELY(stateMachine.isStateTimedOut())) {
        stateMachine.handleTimeout();
    }

    // --- State Machine Logic ---
    const config::RobotState currentState = stateMachine.getCurrentState();
    switch (currentState) {
        case config::IDLE:
            stopAllMotors();
            break;
        case config::MANUAL_CONTROL:
            // Manual control is handled by processControllerInput()
            break;
        case config::AUTOMATIC_OUTTAKE_REVERSE: {
            const bool limitSwitchTriggered = readLimitSwitch();
            if (limitSwitchTriggered) {
                setRobotState(config::MANUAL_CONTROL);
                break;
            }
            const int outtakePWM = PWMUtils::getOuttakePWM();
            outtakeLeft.setSpeed(-outtakePWM);
            outtakeRight.setSpeed(-outtakePWM);
            if (stateMachine.getStateElapsedTime() > config::tuning::OUTTAKE_TIMEOUT_MS || limitSwitchTriggered) {
                setRobotState(config::MANUAL_CONTROL);
            }
            break;
        }
        case config::TIMED_OUTTAKE_FORWARD: {
            const int outtakePWM = PWMUtils::getOuttakePWM();
            outtakeLeft.setSpeed(outtakePWM);
            outtakeRight.setSpeed(outtakePWM);
            if (stateMachine.getStateElapsedTime() > config::tuning::OUTTAKE_FORWARD_TIMEOUT_MS) {
                setRobotState(config::MANUAL_CONTROL);
            }
            break;
        }
    }

    // --- Update all motors ---
    driveLeft.update();
    driveRight.update();
    outtakeLeft.update();
    outtakeRight.update();
}

void Robot::processControllerInput(const ControllerState& controllerState) {
    handleDriveInput(controllerState);
    handleOuttakeInput(controllerState);
    handleServoInput(controllerState);
    handleSpecialCommands(controllerState);
}

void Robot::handleDriveInput(const ControllerState& controllerState) {
    int leftPWM, rightPWM;
    calculateDriveMotorSpeeds(controllerState, leftPWM, rightPWM);
    setDriveMotorSpeeds(leftPWM, rightPWM);
}

void Robot::handleOuttakeInput(const ControllerState& controllerState) {
    int outtakeMotorCommand = 0;  // Use literal 0 instead of constant lookup

    if (controllerState.l1_pressed) {
        outtakeMotorCommand = PWMUtils::getOuttakePWM();
    } else if (controllerState.l2_pressed) {
        outtakeMotorCommand = -PWMUtils::getOuttakePWM();
    }

    // Safety check: prevent reverse motion if limit switch is triggered
    if (readLimitSwitch() && outtakeMotorCommand < 0) {
        outtakeMotorCommand = 0;
    }

    setOuttakeMotorSpeeds(outtakeMotorCommand);
}

void Robot::handleServoInput(const ControllerState& controllerState) {
    if (controllerState.r2_pressed) {
        handleServoToggle(config::OUTTAKE_SERVO_CHANNEL, outtakeArmToggled,
                         config::tuning::OUTTAKE_ARM_OPEN_ANGLE, config::tuning::OUTTAKE_ARM_CLOSE_ANGLE);
    }
    if (controllerState.circle_pressed) {
        handleServoToggle(config::INTAKE_SERVO_CHANNEL, intakeArmToggled,
                         config::tuning::INTAKE_ARM_OPEN_ANGLE, config::tuning::INTAKE_ARM_CLOSE_ANGLE);
    }
}

void Robot::handleSpecialCommands(const ControllerState& controllerState) {
    if (controllerState.start_pressed) {
        config::tuning::INVERT_OUTTAKE_RIGHT = !config::tuning::INVERT_OUTTAKE_RIGHT;
    }
    if (controllerState.pad_right_pressed) {
        config::tuning::LIMIT_SWITCH_DISABLED = !config::tuning::LIMIT_SWITCH_DISABLED;
    }
    if (controllerState.pad_down_pressed) {
        if (!config::tuning::LIMIT_SWITCH_DISABLED) {
            automaticOuttakeReverse();
        }
    }
    if (controllerState.pad_up_pressed && readLimitSwitch()) {
        timedOuttakeForward();
    }
}

void Robot::automaticOuttakeReverse() {
    setRobotState(config::AUTOMATIC_OUTTAKE_REVERSE);
    ESP32Servo::setAngle(config::servo::LEDC_CHANNEL, config::tuning::OUTTAKE_ARM_CLOSE_ANGLE);
    setServoAngle(config::INTAKE_SERVO_CHANNEL, config::tuning::INTAKE_ARM_CLOSE_ANGLE);
}

void Robot::timedOuttakeForward() {
    setRobotState(config::TIMED_OUTTAKE_FORWARD);
}

void Robot::setRobotState(config::RobotState newState) {
    stateMachine.setState(newState);
}

config::RobotState Robot::getRobotState() const {
    return stateMachine.getCurrentState();
}

bool Robot::readLimitSwitch() {
    if (config::tuning::LIMIT_SWITCH_DISABLED) {
        return false;
    }
    int reading = digitalRead(config::LIMIT_SWITCH_PIN);
    if (reading != limitSwitchState.lastRawReading) {
        limitSwitchState.lastDebounceTime = millis();
    }
    if ((millis() - limitSwitchState.lastDebounceTime) > config::tuning::DEBOUNCE_DELAY_MS) {
        limitSwitchState.lastState = reading;
    }
    limitSwitchState.lastRawReading = reading;
    return limitSwitchState.lastState == config::constants::LIMIT_SWITCH_TRIGGERED;
}

// Helper function implementations (PWM utilities now handle input processing)

// Decomposed helper functions for better code organization
void Robot::calculateDriveMotorSpeeds(const ControllerState& controllerState, int& leftPWM, int& rightPWM) {
    // Determine speed mode (precision vs normal)
    int topDuty = controllerState.r1_pressed ? PWMUtils::getPrecisionPWM() : PWMUtils::getMaxPWM();

    // Process joystick inputs
    int processedY, processedX;
    processJoystickInputs(controllerState, processedY, processedX);

    // Calculate motor speeds using selected drive style
    if (config::tuning::USE_CHEESY_DRIVE) {
        // Cheesy Drive: maintains forward speed while turning
        PWMUtils::calculateCheesyDrive(processedY, processedX, leftPWM, rightPWM, topDuty);
    } else {
        // Traditional Differential Drive
        PWMUtils::calculateDifferentialDrive(processedY, processedX, leftPWM, rightPWM, topDuty);
    }
}

void Robot::processJoystickInputs(const ControllerState& controllerState, int& processedY, int& processedX) {
    // Apply deadzone to joystick inputs for better control
    processedY = PWMUtils::applyDeadzone(controllerState.left_joystick_y, config::ps2::JOYSTICK_DEADZONE);
    processedX = PWMUtils::applyDeadzone(controllerState.right_joystick_x, config::ps2::JOYSTICK_DEADZONE);
}

void Robot::setDriveMotorSpeeds(int leftPWM, int rightPWM) {
    // Apply PWM clamping for safety
    int safeLeftPWM = PWMUtils::clampPWM(leftPWM);
    int safeRightPWM = PWMUtils::clampPWM(rightPWM);

    // Detect if this is a turning movement
    bool is_turning = detectTurningMovement(safeLeftPWM, safeRightPWM);

    // Set motor speeds with turn context (electromagnetic braking always enabled)
    driveLeft.setSpeed(safeLeftPWM, is_turning);
    driveRight.setSpeed(safeRightPWM, is_turning);
}

void Robot::setOuttakeMotorSpeeds(int speed) {
    // Apply PWM clamping for safety
    int safeSpeed = PWMUtils::clampPWM(speed);
    outtakeLeft.setSpeed(safeSpeed);
    outtakeRight.setSpeed(safeSpeed);
}

void Robot::stopAllMotors() {
    // Use literal 0 for better performance than constant lookup
    driveLeft.setSpeed(0);
    driveRight.setSpeed(0);
    outtakeLeft.setSpeed(0);
    outtakeRight.setSpeed(0);
}

void Robot::brakeAllMotors() {
    // Apply FTC-style active braking to all motors
    driveLeft.brake();
    driveRight.brake();
    outtakeLeft.brake();
    outtakeRight.brake();
}

void Robot::coastAllMotors() {
    // Coast all motors to stop (no active braking)
    driveLeft.coast();
    driveRight.coast();
    outtakeLeft.coast();
    outtakeRight.coast();
}

bool Robot::detectTurningMovement(int leftPWM, int rightPWM) {
    // Performance-optimized turning detection with fast math
    const int speed_difference = FAST_ABS(leftPWM - rightPWM);

    // Use bit shifting for division by 2 (faster than division)
    const int average_speed = (FAST_ABS(leftPWM) + FAST_ABS(rightPWM)) >> 1;

    // Use pre-computed multiplier for percentage conversion
    const int difference_percent = (speed_difference * config::performance::PWM_TO_PERCENT_MULTIPLIER);
    const int average_percent = (average_speed * config::performance::PWM_TO_PERCENT_MULTIPLIER);

    // Use config turn threshold directly
    int turn_threshold = config::tuning::TURN_DETECTION_THRESHOLD;

    // Detect turning based on:
    // 1. Significant speed difference between motors
    // 2. Motors moving in opposite directions (differential steering)
    // 3. One motor significantly faster than the other during forward/reverse
    bool significant_difference = difference_percent > turn_threshold;
    bool opposite_directions = (leftPWM > 0 && rightPWM < 0) || (leftPWM < 0 && rightPWM > 0);
    bool coordinated_turn = average_percent > 10 && significant_difference; // Avoid false positives at very low speeds

    return opposite_directions || (coordinated_turn && significant_difference);
}

bool Robot::handleServoToggle(uint8_t channel, bool& toggleState, int openAngle, int closeAngle) {
    toggleState = !toggleState;
    int targetAngle = toggleState ? openAngle : closeAngle;

    // Use ESP32 servo for outtake, PCA9685 for others
    if (channel == config::OUTTAKE_SERVO_CHANNEL) {
        ESP32Servo::setAngle(config::OUTTAKE_SERVO_GPIO_PIN, targetAngle);
    } else {
        setServoAngle(channel, targetAngle);
    }
    return true;
}

// Enhanced Error Handling & Recovery Implementation
void Robot::handleSystemError() {
    _consecutive_errors++;
    _last_error_time = millis();

    ERROR_PRINT("System error detected. Consecutive errors: ");
    ERROR_PRINTLN(_consecutive_errors);

    if (_consecutive_errors >= 5) { // Max consecutive errors before emergency stop
        ERROR_PRINTLN("CRITICAL: Maximum consecutive errors reached - entering emergency stop");
        _emergency_stop_active = true;

        // Emergency stop all systems
        stopAllMotors();
        hal_pca9685_emergency_stop();
        setRobotState(config::IDLE);

        // Try to recover PCA9685 communication
        if (hal_pca9685_recover()) {
            DEBUG_PRINTLN("PCA9685 recovery successful - resetting error counter");
            resetErrorCounter();
            _emergency_stop_active = false;
        } else {
            ERROR_PRINTLN("PCA9685 recovery failed - system remains in emergency stop");
        }
    } else {
        // Minor error - try to continue with degraded functionality
        DEBUG_PRINTLN("Minor system error - attempting to continue");
        setRobotState(config::IDLE); // Go to safe state temporarily
    }
}

void Robot::resetErrorCounter() {
    _consecutive_errors = 0;
    _last_error_time = 0;
    DEBUG_PRINTLN("Error counter reset - system health restored");
}

bool Robot::isSystemHealthy() {
    // If in emergency stop, system is not healthy
    if (_emergency_stop_active) {
        return false;
    }

    // Check if too many recent errors
    if (_consecutive_errors >= config::constants::MAX_CONSECUTIVE_ERRORS) { // Max consecutive errors threshold
        return false;
    }

    // Check PCA9685 communication health
    if (!hal_pca9685_is_connected()) {
        ERROR_PRINTLN("PCA9685 communication failure detected");
        return false;
    }

    // If we've had recent errors but not too many, check if enough time has passed to reset
    if (_consecutive_errors > 0 && (millis() - _last_error_time) > 5000) {
        resetErrorCounter(); // Reset after 5 seconds of stable operation
    }

    return true;
}