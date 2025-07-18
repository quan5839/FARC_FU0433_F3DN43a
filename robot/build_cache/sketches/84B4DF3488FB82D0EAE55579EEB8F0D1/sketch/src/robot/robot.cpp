#line 1 "/Users/admin/Documents/Arduino/robot/src/robot/robot.cpp"
#include "robot.h"
#include "src/hal/motor.h"
#include "src/hal/servos.h"
#include "src/hal/pca9685_driver.h"
#include "src/controller/PS2_controller.h"
#include "src/utils/pwm_utils.h"
#include "src/utils/validation.h"
#include "src/utils/safety_monitor.h"
#include "src/utils/config_validator.h"
#include "state_machine.h"
#include <Arduino.h>

// --- Robot class implementation ---

Robot::Robot()
    : driveLeft(config::CHAN_DRIVE_L_FWD, config::CHAN_DRIVE_L_REV, config::tuning::invertDriveLeft),
      driveRight(config::CHAN_DRIVE_R_FWD, config::CHAN_DRIVE_R_REV, config::tuning::invertDriveRight),
      outtakeLeft(config::CHAN_OUTTAKE_L_FWD, config::CHAN_OUTTAKE_L_REV, config::constants::MOTOR_NOT_INVERTED),
      outtakeRight(config::CHAN_OUTTAKE_R_FWD, config::CHAN_OUTTAKE_R_REV, config::tuning::invertOuttakeRight) {}

void Robot::init() {
    // Validate configuration before starting
    if (!ConfigValidator::validateAll()) {
        Serial.println("CRITICAL: Configuration validation failed!");
        while(1) delay(1000); // Halt system
    }

    // Initialize safety monitoring
    SafetyMonitor::init();

    initPCA9685();
    pinMode(config::LIMIT_SWITCH_PIN, INPUT_PULLUP);

    // Initialize state machine
    stateMachine.init();

    // Initialize servos with validation
    Validation::setServoAngleSafe(config::INTAKE_SERVO_CHANNEL, config::tuning::INTAKE_ARM_CLOSE_ANGLE);
    Validation::setServoAngleSafe(config::OUTTAKE_SERVO_CHANNEL, config::tuning::OUTTAKE_ARM_CLOSE_ANGLE);

    Serial.println("Robot initialization complete");
}

void Robot::loop() {
    // Update safety monitoring
    SafetyMonitor::update();
    SafetyMonitor::feedWatchdog();

    // Check system health first
    if (!SafetyMonitor::isSystemSafe() || !isSystemHealthy()) {
        handleSystemError();
        return;
    }

    if (!isConnected()) {
        setRobotState(config::IDLE);
    }

    // Check for state timeouts to prevent getting stuck
    if (stateMachine.isStateTimedOut()) {
        stateMachine.handleTimeout();
    }

    // --- State Machine Logic ---
    switch (stateMachine.getCurrentState()) {
        case config::IDLE:
            stopAllMotors();
            break;
        case config::MANUAL_CONTROL:
            // Manual control is handled by processControllerInput()
            break;
        case config::AUTOMATIC_OUTTAKE_REVERSE:
            if (readLimitSwitch()) {
                setRobotState(config::MANUAL_CONTROL);
                break;
            }
            outtakeLeft.setSpeed(-PWMUtils::getOuttakePWM());
            outtakeRight.setSpeed(-PWMUtils::getOuttakePWM());
            if (stateMachine.getStateElapsedTime() > config::tuning::OUTTAKE_TIMEOUT_MS || readLimitSwitch()) {
                setRobotState(config::MANUAL_CONTROL);
            }
            break;
        case config::TIMED_OUTTAKE_FORWARD:
            outtakeLeft.setSpeed(PWMUtils::getOuttakePWM());
            outtakeRight.setSpeed(PWMUtils::getOuttakePWM());
            if (stateMachine.getStateElapsedTime() > config::tuning::OUTTAKE_FORWARD_TIMEOUT_MS) {
                setRobotState(config::MANUAL_CONTROL);
            }
            break;
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
    int outtakeMotorCommand = config::constants::MOTOR_STOP_SPEED;

    if (controllerState.l1_pressed) {
        outtakeMotorCommand = PWMUtils::getOuttakePWM();
    } else if (controllerState.l2_pressed) {
        outtakeMotorCommand = -PWMUtils::getOuttakePWM();
    }

    // Safety check: prevent reverse motion if limit switch is triggered
    if (readLimitSwitch() && outtakeMotorCommand < config::constants::MOTOR_STOP_SPEED) {
        outtakeMotorCommand = config::constants::MOTOR_STOP_SPEED;
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
        config::tuning::invertOuttakeRight = !config::tuning::invertOuttakeRight;
    }
    if (controllerState.pad_right_pressed) {
        config::tuning::limitSwitchDisabled = !config::tuning::limitSwitchDisabled;
    }
    if (controllerState.pad_down_pressed) {
        if (!config::tuning::limitSwitchDisabled) {
            automaticOuttakeReverse();
        }
    }
    if (controllerState.pad_up_pressed && readLimitSwitch()) {
        timedOuttakeForward();
    }
}

void Robot::automaticOuttakeReverse() {
    setRobotState(config::AUTOMATIC_OUTTAKE_REVERSE);
    setServoAngle(config::OUTTAKE_SERVO_CHANNEL, config::tuning::OUTTAKE_ARM_CLOSE_ANGLE);
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
    if (config::tuning::limitSwitchDisabled) {
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
    // Apply validation
    int safeLeftPWM = Validation::setMotorSpeedSafe(leftPWM);
    int safeRightPWM = Validation::setMotorSpeedSafe(rightPWM);

    // Detect if this is a turning movement
    bool is_turning = detectTurningMovement(safeLeftPWM, safeRightPWM);

    // Set motor speeds with turn context (electromagnetic braking always enabled)
    driveLeft.setSpeed(safeLeftPWM, is_turning);
    driveRight.setSpeed(safeRightPWM, is_turning);
}

void Robot::setOuttakeMotorSpeeds(int speed) {
    // Apply validation and set motor speeds
    int safeSpeed = Validation::setMotorSpeedSafe(speed);
    outtakeLeft.setSpeed(safeSpeed);
    outtakeRight.setSpeed(safeSpeed);
}

void Robot::stopAllMotors() {
    driveLeft.setSpeed(config::constants::MOTOR_STOP_SPEED);
    driveRight.setSpeed(config::constants::MOTOR_STOP_SPEED);
    outtakeLeft.setSpeed(config::constants::MOTOR_STOP_SPEED);
    outtakeRight.setSpeed(config::constants::MOTOR_STOP_SPEED);
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
    // Calculate the difference between left and right motor speeds
    int speed_difference = abs(leftPWM - rightPWM);

    // Calculate the average speed to determine if we're moving
    int average_speed = (abs(leftPWM) + abs(rightPWM)) / 2;

    // Convert to percentage for threshold comparison
    int difference_percent = (speed_difference * 100) / config::motor::MAX_PWM;
    int average_percent = (average_speed * 100) / config::motor::MAX_PWM;

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

    if (Validation::setServoAngleSafe(channel, targetAngle)) {
        return true;
    }
    return false;
}

// Enhanced Error Handling & Recovery Implementation
void Robot::handleSystemError() {
    _consecutive_errors++;
    _last_error_time = millis();

    Serial.print("System error detected. Consecutive errors: ");
    Serial.println(_consecutive_errors);

    if (_consecutive_errors >= config::constants::MAX_CONSECUTIVE_ERRORS) {
        Serial.println("CRITICAL: Maximum consecutive errors reached - entering emergency stop");
        _emergency_stop_active = true;

        // Emergency stop all systems
        stopAllMotors();
        hal_pca9685_emergency_stop();
        setRobotState(config::IDLE);

        // Try to recover PCA9685 communication
        if (hal_pca9685_recover()) {
            Serial.println("PCA9685 recovery successful - resetting error counter");
            resetErrorCounter();
            _emergency_stop_active = false;
        } else {
            Serial.println("PCA9685 recovery failed - system remains in emergency stop");
        }
    } else {
        // Minor error - try to continue with degraded functionality
        Serial.println("Minor system error - attempting to continue");
        setRobotState(config::IDLE); // Go to safe state temporarily
    }
}

void Robot::resetErrorCounter() {
    _consecutive_errors = 0;
    _last_error_time = 0;
    Serial.println("Error counter reset - system health restored");
}

bool Robot::isSystemHealthy() {
    // If in emergency stop, system is not healthy
    if (_emergency_stop_active) {
        return false;
    }

    // Check if too many recent errors
    if (_consecutive_errors >= config::constants::MAX_CONSECUTIVE_ERRORS) {
        return false;
    }

    // Check PCA9685 communication health
    if (!hal_pca9685_is_connected()) {
        Serial.println("PCA9685 communication failure detected");
        return false;
    }

    // If we've had recent errors but not too many, check if enough time has passed to reset
    if (_consecutive_errors > 0 && (millis() - _last_error_time) > 5000) {
        resetErrorCounter(); // Reset after 5 seconds of stable operation
    }

    return true;
}