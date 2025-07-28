#include "robot.h"
#include "../hal/servos.h"
#include "../hal/pca9685_driver.h"
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

    // Initialize IMU sensor
    if (config::imu::ENABLE_IMU) {
        if (IMU::init()) {
            DEBUG_PRINTLN("IMU initialized successfully");
        } else {
            ERROR_PRINTLN("IMU initialization failed - continuing without IMU");
        }
    }
    // Initialize limit switch with pull-up resistor
    // This ensures disconnected switches read HIGH (safe, not triggered)
    pinMode(config::LIMIT_SWITCH_PIN, INPUT_PULLUP);

    // Initialize state machine
    stateMachine.init();

    // Initialize servos (setServoAngle already constrains angles to 0-180)
    setServoAngle(config::INTAKE_SERVO_CHANNEL, config::tuning::INTAKE_ARM_CLOSE_ANGLE);

    // Initialize WS2812B LED strip
    initLEDStrip();
}

void Robot::loop() {
    // Performance-optimized main loop with branch prediction
    SafetyMonitor::update();

    // Update IMU sensor readings
    if (config::imu::ENABLE_IMU) {
        static unsigned long last_imu_update = 0;
        if (millis() - last_imu_update >= config::imu::UPDATE_INTERVAL_MS) {
            IMU::update();
            last_imu_update = millis();
        }
    }



    // Check for controller safety shutdown (separate from system errors)
    if (UNLIKELY(SafetyMonitor::isControllerSafetyShutdownActive())) {
        // Controller safety shutdown - clean stop without error handling
        stopAllMotors();
        setRobotState(config::IDLE);
        return;
    }

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

    // --- Update LED strip ---
    updateLEDStrip();
}

void Robot::processControllerInput(const ControllerState& controllerState) {
    // Check controller input safety and update monitoring
    SafetyMonitor::checkControllerInputSafety(controllerState);

    // If controller safety shutdown is active, don't process any input
    // (This is also checked in main loop, but kept here for safety)
    if (SafetyMonitor::isControllerSafetyShutdownActive()) {
        return; // Input processing blocked - main loop handles stopping
    }

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

    // Safety check: prevent reverse motion if limit switch is triggered (unless overridden)
    if (!config::tuning::LIMIT_SWITCH_DISABLED && readLimitSwitch() && outtakeMotorCommand < 0) {
        outtakeMotorCommand = 0;
        DEBUG_PRINTLN("Limit switch blocking downward motion - use Right D-pad to override");
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


    if (controllerState.pad_right_pressed) {
        config::tuning::LIMIT_SWITCH_DISABLED = !config::tuning::LIMIT_SWITCH_DISABLED;
        DEBUG_PRINT("Limit switch override: ");
        DEBUG_PRINTLN(config::tuning::LIMIT_SWITCH_DISABLED ? "DISABLED (override active)" : "ENABLED (safety active)");
    }
    if (controllerState.pad_down_pressed) {
        if (!config::tuning::LIMIT_SWITCH_DISABLED) {
            automaticOuttakeReverse();
        }
    }
    if (controllerState.pad_up_pressed && readLimitSwitch()) {
        timedOuttakeForward();
    }



    // IMU test (L2 + SELECT)
    if (controllerState.l2_pressed && controllerState.select_pressed) {
        DEBUG_PRINTLN("Manual IMU test triggered via controller (L2 + SELECT)");
        testIMU();
    }

    // LED strip hardware test (R1 + SELECT)
    if (controllerState.r1_pressed && controllerState.select_pressed) {
        DEBUG_PRINTLN("LED strip hardware test triggered via controller (R1 + SELECT)");
        testLEDStripHardware();
    }

    // Reset controller input safety (R2 + SELECT)
    if (controllerState.r2_pressed && controllerState.select_pressed) {
        DEBUG_PRINTLN("Controller input safety reset triggered via controller (R2 + SELECT)");
        SafetyMonitor::resetControllerInputSafety();
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
    if (config::tuning::LIMIT_SWITCH_DISABLED) {
        return false;
    }

    // Simple, reliable limit switch reading with debouncing
    int reading = digitalRead(config::LIMIT_SWITCH_PIN);

    // Debouncing logic
    if (reading != limitSwitchState.lastRawReading) {
        limitSwitchState.lastDebounceTime = millis();
    }

    if ((millis() - limitSwitchState.lastDebounceTime) > config::tuning::DEBOUNCE_DELAY_MS) {
        limitSwitchState.lastState = reading;
    }

    limitSwitchState.lastRawReading = reading;

    // With INPUT_PULLUP:
    // - Connected switch pressed = LOW (triggered)
    // - Connected switch open = HIGH (not triggered)
    // - Disconnected switch = HIGH (safe, not triggered)
    return limitSwitchState.lastState == config::constants::LIMIT_SWITCH_TRIGGERED;
}





// I2C monitoring functions removed

// Helper function implementations
void Robot::calculateDriveMotorSpeeds(const ControllerState& controllerState, int& leftPWM, int& rightPWM) {
    // Determine speed mode (precision vs normal)
    int topDuty = controllerState.r1_pressed ? PWMUtils::getPrecisionPWM() : PWMUtils::getMaxPWM();

    // Process joystick inputs
    int processedY, processedX;
    processJoystickInputs(controllerState, processedY, processedX);

    // Calculate motor speeds using selected drive style
    PWMUtils::calculateCheesyDrive(processedY, processedX, leftPWM, rightPWM, topDuty);
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

    // Apply holding power when no outtake buttons are pressed
    if (safeSpeed == 0) {
        // Use small forward power to hold position and prevent rolling back
        int holdPower = PWMUtils::percentageToPWM(config::tuning::OUTTAKE_HOLD_POWER_PERCENT);
        outtakeLeft.setSpeed(holdPower, false);
        outtakeRight.setSpeed(holdPower, false);
    } else {
        // Normal speed control when moving
        outtakeLeft.setSpeed(safeSpeed, false);
        outtakeRight.setSpeed(safeSpeed, false);
    }
}

void Robot::stopAllMotors() {
    // Use literal 0 for better performance than constant lookup
    // Enable braking for better stopping control on all motors
    driveLeft.setSpeed(0, false);
    driveRight.setSpeed(0, false);
    outtakeLeft.setSpeed(0, false);
    outtakeRight.setSpeed(0, false);
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
    // Detect if the robot is turning based on differential motor speeds
    int pwmDifference = abs(leftPWM - rightPWM);
    return pwmDifference > config::tuning::TURN_DETECTION_THRESHOLD;
}
    #endif

    #if !COMPETITION_MODE
    // Test MPU6050 performance (if enabled)
    if (config::imu::ENABLE_IMU) {
        Serial.println("Testing MPU6050...");

        // Switch to MPU6050 I2C speed for testing
        Wire.setClock(config::imu::I2C_CLOCK_SPEED);
        Serial.print("MPU6050 I2C speed: ");
        Serial.print(config::imu::I2C_CLOCK_SPEED / 1000);
        Serial.println("kHz");

        // Reset counters
        total_time = 0;
        min_time = ULONG_MAX;
        max_time = 0;
        successful_transactions = 0;

        for (int i = 0; i < TEST_ITERATIONS; i++) {
            const unsigned long start = micros();
            Wire.beginTransmission(config::imu::I2C_ADDRESS);
            uint8_t error = Wire.endTransmission();
            const unsigned long duration = micros() - start;

            if (error == 0) {
                successful_transactions++;
                total_time += duration;
                if (duration < min_time) min_time = duration;
                if (duration > max_time) max_time = duration;
            }

            delayMicroseconds(100);
        }

        if (successful_transactions > 0) {
            const float avg_time = (float)total_time / successful_transactions;
            const float success_rate = (float)successful_transactions / TEST_ITERATIONS * 100;

            Serial.print("MPU6050 Results:");
            Serial.print(" Success Rate: ");
            Serial.print(success_rate, 1);
            Serial.println("%");
            Serial.print("  Average Time: ");
            Serial.print(avg_time, 1);
            Serial.println("μs");
            Serial.print("  Min Time: ");
            Serial.print(min_time);
            Serial.println("μs");
            Serial.print("  Max Time: ");
            Serial.print(max_time);
            Serial.println("μs");
        } else {
            Serial.println("MPU6050: NO SUCCESSFUL TRANSACTIONS!");
        }

        // Restore PCA9685 I2C speed after MPU6050 testing
        Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);
        Serial.print("I2C speed restored to ");
        Serial.print(config::pca9685::I2C_CLOCK_SPEED / 1000);
        Serial.println("kHz for PCA9685");
    }

    // I2C Bus Speed Analysis
    Serial.println("--- I2C Bus Analysis ---");
    Serial.print("PCA9685 Speed: ");
    Serial.print(config::pca9685::I2C_CLOCK_SPEED / 1000);
    Serial.println(" kHz");
    if (config::imu::ENABLE_IMU) {
        Serial.print("MPU6050 Speed: ");
        Serial.print(config::imu::I2C_CLOCK_SPEED / 1000);
        Serial.println(" kHz");
    }
    Serial.println("VIA v2023 Board: 1kΩ pull-ups (optimal for 1MHz)");
    Serial.print("Pull-up Recommendation: ");
    if (config::pca9685::I2C_CLOCK_SPEED >= 1000000) {
        Serial.println("OPTIMAL (1kΩ for 1MHz)");
    } else {
        Serial.println("CONSERVATIVE (could use 1MHz)");
    }

    Serial.println("============================");
    #endif
}

void Robot::scanI2CBus() {
    #if !COMPETITION_MODE
    Serial.println("=== I2C Bus Scanner ===");
    Serial.println("Scanning I2C bus for devices...");

    int devices_found = 0;

    // Test both I2C speeds
    const long speeds[] = {400000, 1000000};
    const char* speed_names[] = {"400kHz", "1MHz"};

    for (int speed_idx = 0; speed_idx < 2; speed_idx++) {
        Wire.setClock(speeds[speed_idx]);
        Serial.print("\nScanning at ");
        Serial.print(speed_names[speed_idx]);
        Serial.println(":");

        constexpr uint8_t I2C_MIN_ADDRESS = 1;
        constexpr uint8_t I2C_MAX_ADDRESS = 127;
        for (uint8_t address = I2C_MIN_ADDRESS; address < I2C_MAX_ADDRESS; address++) {
            Wire.beginTransmission(address);
            uint8_t error = Wire.endTransmission();

            if (error == 0) {
                Serial.print("Device found at address 0x");
                if (address < 16) Serial.print("0");
                Serial.print(address, HEX);

                // Identify known devices
                if (address == config::pca9685::I2C_ADDRESS) {
                    Serial.print(" (PCA9685 Motor Controller)");
                } else if (address == config::imu::I2C_ADDRESS) {
                    Serial.print(" (MPU6050 IMU - CONFIGURED ADDRESS)");
                } else if (address == config::imu::I2C_ADDRESS_ALT) {
                    Serial.print(" (MPU6050 IMU - Alternative Address)");
                } else {
                    Serial.print(" (Unknown Device)");
                }
                Serial.println();
                devices_found++;
            }

            delay(1); // Small delay between scans
        }
    }

    // Restore PCA9685 speed
    Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);

    Serial.print("\nScan complete. Found ");
    Serial.print(devices_found);
    Serial.println(" device(s).");

    if (devices_found == 0) {
        Serial.println("No I2C devices found. Check wiring and pull-up resistors.");
    }

    Serial.println("=======================");
    #endif
}


// Helper function implementations
void Robot::calculateDriveMotorSpeeds(const ControllerState& controllerState, int& leftPWM, int& rightPWM) {
    // Determine speed mode (precision vs normal)
    int topDuty = controllerState.r1_pressed ? PWMUtils::getPrecisionPWM() : PWMUtils::getMaxPWM();

    // Process joystick inputs
    int processedY, processedX;
    processJoystickInputs(controllerState, processedY, processedX);

    // Calculate motor speeds using selected drive style
    PWMUtils::calculateCheesyDrive(processedY, processedX, leftPWM, rightPWM, topDuty);
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

    // Apply holding power when no outtake buttons are pressed
    if (safeSpeed == 0) {
        // Use small forward power to hold position and prevent rolling back
        int holdPower = PWMUtils::percentageToPWM(config::tuning::OUTTAKE_HOLD_POWER_PERCENT);
        outtakeLeft.setSpeed(holdPower, false);
        outtakeRight.setSpeed(holdPower, false);
    } else {
        // Normal speed control when moving
        outtakeLeft.setSpeed(safeSpeed, false);
        outtakeRight.setSpeed(safeSpeed, false);
    }
}

void Robot::stopAllMotors() {
    // Use literal 0 for better performance than constant lookup
    // Enable braking for better stopping control on all motors
    driveLeft.setSpeed(0, false);
    driveRight.setSpeed(0, false);
    outtakeLeft.setSpeed(0, false);
    outtakeRight.setSpeed(0, false);
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
    constexpr int MIN_SPEED_FOR_TURN_DETECTION = 10; // Minimum speed percentage to detect turns
    bool coordinated_turn = average_percent > MIN_SPEED_FOR_TURN_DETECTION && significant_difference;

    return opposite_directions || (coordinated_turn && significant_difference);
}

bool Robot::handleServoToggle(uint8_t channel, bool& toggleState, int openAngle, int closeAngle) {
    toggleState = !toggleState;
    int targetAngle = toggleState ? openAngle : closeAngle;

    // Use PCA9685 for all servos
    setServoAngle(channel, targetAngle);
    return true;
}

// Enhanced Error Handling & Recovery Implementation
void Robot::handleSystemError() {
    _consecutive_errors++;
    _last_error_time = millis();

    ERROR_PRINT("System error detected. Consecutive errors: ");
    ERROR_PRINTLN(_consecutive_errors);

    if (_consecutive_errors >= config::constants::MAX_CONSECUTIVE_ERRORS) {
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

// WS2812B LED Strip Control Functions
void Robot::initLEDStrip() {
    if (WS2812BStrip::init()) {
        DEBUG_PRINTLN("WS2812B LED strip initialized successfully");

        // Quick initialization test without blocking delays
        DEBUG_PRINTLN("LED strip quick test - brief flash");
        WS2812BStrip::powerOn();
        WS2812BStrip::setSolidColor(0, 255, 0); // Green to indicate successful init
        WS2812BStrip::show();
        delay(100); // Very brief flash - won't disrupt PS2 controller

        // Clear and prepare for normal operation
        DEBUG_PRINTLN("LED strip ready for normal operation");
        WS2812BStrip::clear();
        WS2812BStrip::show();
    } else {
        ERROR_PRINTLN("WS2812B LED strip initialization failed");
    }
}

void Robot::updateLEDStrip() {
    // Update LED strip based on current robot state
    config::RobotState currentState = stateMachine.getCurrentState();

    // Enable power for active states, disable for off states
    bool should_be_powered = false;

    switch (currentState) {
        case config::IDLE:
            WS2812BStrip::setRobotStatus(1); // Idle - blue breathing
            should_be_powered = true;
            break;
        case config::MANUAL_CONTROL:
            WS2812BStrip::setRobotStatus(2); // Manual - solid green
            should_be_powered = true;
            break;
        case config::AUTOMATIC_OUTTAKE_REVERSE:
        case config::TIMED_OUTTAKE_FORWARD:
            WS2812BStrip::setRobotStatus(3); // Auto - rainbow
            should_be_powered = true;
            break;
        default:
            WS2812BStrip::setRobotStatus(0); // Off
            should_be_powered = false;
            break;
    }

    // Check for controller safety shutdown (different from system errors)
    if (SafetyMonitor::isControllerSafetyShutdownActive()) {
        WS2812BStrip::setRobotStatus(5); // Controller safety - orange/yellow warning
        should_be_powered = true; // Show safety warning
    }
    // Check for system error conditions
    else if (!isSystemHealthy() || !SafetyMonitor::isSystemSafe()) {
        WS2812BStrip::setRobotStatus(4); // Error - red blinking
        should_be_powered = true; // Show error even if system is unhealthy
    }

    // Control power based on state
    if (should_be_powered && !WS2812BStrip::isPowerOn()) {
        WS2812BStrip::powerOn();
    } else if (!should_be_powered && WS2812BStrip::isPowerOn()) {
        WS2812BStrip::powerOff();
    }

    // Update the strip
    WS2812BStrip::update();
}

void Robot::enableLEDStrip() {
    WS2812BStrip::powerOn();
    DEBUG_PRINTLN("LED strip power enabled");
}

void Robot::disableLEDStrip() {
    WS2812BStrip::powerOff();
    DEBUG_PRINTLN("LED strip power disabled");
}

// IMU Testing Functions
void Robot::testIMU() {
    DEBUG_PRINTLN("=== IMU TEST START ===");

    if (!config::imu::ENABLE_IMU) {
        DEBUG_PRINTLN("IMU is disabled in config - enable it to test");
        return;
    }

    // Check if IMU is connected
    if (!IMU::isConnected()) {
        ERROR_PRINTLN("IMU not connected! Check wiring:");
        ERROR_PRINTLN("- SDA: GPIO 21");
        ERROR_PRINTLN("- SCL: GPIO 22");
        ERROR_PRINTLN("- VCC: 3.3V or 5V");
        ERROR_PRINTLN("- GND: Ground");
        ERROR_PRINTLN("- I2C Address: 0x68 (or 0x69 if AD0 is high)");
        return;
    }

    DEBUG_PRINTLN("IMU connected successfully!");
    DEBUG_PRINTLN("Testing IMU for 10 seconds...");
    DEBUG_PRINTLN("Try moving and rotating the robot to see readings change");
    DEBUG_PRINTLN("");

    // Reset heading for testing
    IMU::resetHeading();

    // Test for 10 seconds
    unsigned long test_start = millis();
    unsigned long last_print = 0;

    while (millis() - test_start < 10000) { // 10 seconds
        // Update IMU readings
        if (IMU::update()) {
            // Print readings every 500ms
            if (millis() - last_print >= 500) {
                printIMUReadings();
                last_print = millis();
            }
        } else {
            ERROR_PRINTLN("Failed to read IMU data");
        }

        delay(10); // Small delay
    }

    DEBUG_PRINTLN("=== IMU TEST COMPLETE ===");
}

void Robot::printIMUReadings() {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temperature;
    float heading;

    // Get all IMU readings
    IMU::getAcceleration(accel_x, accel_y, accel_z);
    IMU::getRotation(gyro_x, gyro_y, gyro_z);
    temperature = IMU::getTemperature();
    heading = IMU::getHeading();

    // Print formatted readings
    DEBUG_PRINT("Accel(m/s²): X=");
    DEBUG_PRINT(accel_x);
    DEBUG_PRINT(" Y=");
    DEBUG_PRINT(accel_y);
    DEBUG_PRINT(" Z=");
    DEBUG_PRINT(accel_z);

    DEBUG_PRINT(" | Gyro(°/s): X=");
    DEBUG_PRINT(gyro_x * 180.0 / PI); // Convert rad/s to deg/s
    DEBUG_PRINT(" Y=");
    DEBUG_PRINT(gyro_y * 180.0 / PI);
    DEBUG_PRINT(" Z=");
    DEBUG_PRINT(gyro_z * 180.0 / PI);

    DEBUG_PRINT(" | Temp: ");
    DEBUG_PRINT(temperature);
    DEBUG_PRINT("°C");

    DEBUG_PRINT(" | Heading: ");
    DEBUG_PRINT(heading);
    DEBUG_PRINT("°");

    // Movement detection
    if (IMU::isMoving()) {
        DEBUG_PRINT(" [MOVING]");
    }
    if (IMU::isTurning()) {
        DEBUG_PRINT(" [TURNING]");
    }

    DEBUG_PRINTLN("");
}

void Robot::testLEDStripHardware() {
    DEBUG_PRINTLN("Starting LED strip hardware test...");
    WS2812BStrip::directHardwareTest();
    DEBUG_PRINTLN("LED strip hardware test complete");
}

