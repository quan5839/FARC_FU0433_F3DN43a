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
    : driveLeft(config::CHAN_DRIVE_L_FWD, config::CHAN_DRIVE_L_REV, config::tuning::INVERT_DRIVE_LEFT, MotorType::DRIVE),
      driveRight(config::CHAN_DRIVE_R_FWD, config::CHAN_DRIVE_R_REV, config::tuning::INVERT_DRIVE_RIGHT, MotorType::DRIVE),
      outtakeLeft(config::CHAN_OUTTAKE_L_FWD, config::CHAN_OUTTAKE_L_REV, config::constants::MOTOR_NOT_INVERTED, MotorType::OUTTAKE),
      outtakeRight(config::CHAN_OUTTAKE_R_FWD, config::CHAN_OUTTAKE_R_REV, config::tuning::INVERT_OUTTAKE_RIGHT, MotorType::OUTTAKE),
      selectButtonOuttakeActive(false), holdingPowerDisabled(false), reverseHoldModeActive(false), l2PressedDuringOverride(false), controlsLockedToOuttake(false) {}

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
    setServoAngle(config::FRUIT_SERVO_CHANNEL, config::tuning::FRUIT_SERVO_CLOSE_ANGLE);

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

    // Periodic I2C health monitoring
    static unsigned long last_i2c_check = 0;
    if (millis() - last_i2c_check >= config::performance::I2C_HEALTH_CHECK_INTERVAL_MS) {
        performI2CHealthCheck();
        last_i2c_check = millis();
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
                // When limit switch is triggered, activate reverse hold mode
                DEBUG_PRINTLN("Limit switch triggered - activating reverse hold mode");
                reverseHoldModeActive = true;
                selectButtonOuttakeActive = false;  // Stop automatic outtake
                setRobotState(config::MANUAL_CONTROL);
                break;
            }
            const int outtakePWM = PWMUtils::getOuttakePWM();
            outtakeLeft.setSpeed(-outtakePWM);
            outtakeRight.setSpeed(-outtakePWM);
            if (stateMachine.getStateElapsedTime() > config::tuning::OUTTAKE_TIMEOUT_MS) {
                // Timeout reached - stop automatic outtake but keep holding power disabled
                selectButtonOuttakeActive = false;
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
    // Force drive motors to stop if controls are locked
    if (controlsLockedToOuttake) {
        driveLeft.setSpeed(0);
        driveRight.setSpeed(0);
    }

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
    // Skip drive control if controls are locked to outtake only
    if (controlsLockedToOuttake) {
        // Stop drive motors when controls are locked
        DEBUG_PRINTLN("Drive control locked - stopping motors");
        setDriveMotorSpeeds(0, 0);
        return;
    }

    int leftPWM, rightPWM;
    calculateDriveMotorSpeeds(controllerState, leftPWM, rightPWM);
    setDriveMotorSpeeds(leftPWM, rightPWM);
}

void Robot::handleOuttakeInput(const ControllerState& controllerState) {
    int outtakeMotorCommand = 0;  // Use literal 0 instead of constant lookup

    if (controllerState.l1_pressed) {
        outtakeMotorCommand = PWMUtils::getOuttakePWM();
        // Clear any reverse hold modes when actively controlling
        reverseHoldModeActive = false;
        l2PressedDuringOverride = false;
    } else if (controllerState.l2_pressed) {
        outtakeMotorCommand = -PWMUtils::getOuttakePWM();
        // Track L2 press during limit switch override
        if (config::tuning::LIMIT_SWITCH_DISABLED) {
            l2PressedDuringOverride = true;
        }
        // Clear reverse hold mode when actively controlling
        reverseHoldModeActive = false;
    } else {
        // No buttons pressed - check if we should activate reverse hold
        if (l2PressedDuringOverride && config::tuning::LIMIT_SWITCH_DISABLED) {
            // L2 was released after being pressed during override - activate reverse hold
            DEBUG_PRINTLN("L2 released during override - activating reverse hold mode");
            reverseHoldModeActive = true;
            l2PressedDuringOverride = false;
        }
    }

    // Safety check: prevent reverse motion if limit switch is triggered (unless overridden)
    if (!config::tuning::LIMIT_SWITCH_DISABLED && readLimitSwitch() && outtakeMotorCommand < 0) {
        outtakeMotorCommand = 0;
        DEBUG_PRINTLN("Limit switch blocking downward motion - use Right D-pad to override");
    }

    setOuttakeMotorSpeeds(outtakeMotorCommand);
}

void Robot::handleServoInput(const ControllerState& controllerState) {
    // Skip servo control if controls are locked to outtake only
    if (controlsLockedToOuttake) {
        return;
    }

    if (controllerState.r2_pressed) {
        handleServoToggle(config::BALL_SERVO_CHANNEL, ballServoToggled,
                         config::tuning::BALL_SERVO_OPEN_ANGLE, config::tuning::BALL_SERVO_CLOSE_ANGLE);
    }
    if (controllerState.circle_pressed) {
        handleServoToggle(config::FRUIT_SERVO_CHANNEL, fruitServoToggled,
                         config::tuning::FRUIT_SERVO_OPEN_ANGLE, config::tuning::FRUIT_SERVO_CLOSE_ANGLE);
    }

    // Continuous servo control for fruit intake
    handleFruitIntakeServos(controllerState);
}

void Robot::handleSpecialCommands(const ControllerState& controllerState) {

    // SELECT button: Toggle holding power and automatic outtake (when pressed alone)
    if (controllerState.select_pressed && !controllerState.start_pressed &&
        !controllerState.l1_pressed && !controllerState.l2_pressed &&
        !controllerState.r1_pressed && !controllerState.r2_pressed) {

        if (selectButtonOuttakeActive || holdingPowerDisabled || reverseHoldModeActive || controlsLockedToOuttake) {
            // Stop automatic outtake reverse and/or restore normal holding power and unlock controls
            DEBUG_PRINTLN("SELECT: Stopping automatic outtake - restoring normal holding power and unlocking controls");
            selectButtonOuttakeActive = false;
            holdingPowerDisabled = false;  // Re-enable normal holding power
            reverseHoldModeActive = false;  // Disable reverse hold mode
            l2PressedDuringOverride = false;  // Reset override tracking
            controlsLockedToOuttake = false;  // Unlock all controls
            setRobotState(config::MANUAL_CONTROL);
        } else if (!selectButtonOuttakeActive && stateMachine.getCurrentState() == config::MANUAL_CONTROL) {
            // Lock controls to outtake only and disable holding power when SELECT is pressed
            controlsLockedToOuttake = true;
            holdingPowerDisabled = true;
            DEBUG_PRINTLN("SELECT: Locking controls to outtake only and disabling holding power");

            // Start automatic outtake reverse only if limit switch safety allows
            if (!config::tuning::LIMIT_SWITCH_DISABLED) {
                DEBUG_PRINTLN("SELECT: Starting automatic outtake reverse");
                selectButtonOuttakeActive = true;
                automaticOuttakeReverse();
            } else {
                DEBUG_PRINTLN("SELECT: Limit switch disabled - holding power off but no automatic outtake");
            }
        }
    }

    // I2C health check (START only, no other buttons)
    if (controllerState.start_pressed && !controllerState.l1_pressed && !controllerState.select_pressed) {
        performI2CHealthCheck();
    }
    if (controllerState.pad_right_pressed) {
        config::tuning::LIMIT_SWITCH_DISABLED = !config::tuning::LIMIT_SWITCH_DISABLED;
        DEBUG_PRINT("Limit switch override: ");
        DEBUG_PRINTLN(config::tuning::LIMIT_SWITCH_DISABLED ? "DISABLED (override active)" : "ENABLED (safety active)");

        // Reset override tracking when limit switch override is disabled
        if (!config::tuning::LIMIT_SWITCH_DISABLED) {
            l2PressedDuringOverride = false;
        }
    }
    if (controllerState.pad_down_pressed) {
        if (!config::tuning::LIMIT_SWITCH_DISABLED) {
            automaticOuttakeReverse();
        }
    }
    if (controllerState.pad_up_pressed && readLimitSwitch()) {
        timedOuttakeForward();
    }

    // Performance test (START + SELECT)
    if (controllerState.start_pressed && controllerState.select_pressed) {
        testI2CPerformance();
    }

    // I2C bus scan (L1 + SELECT)
    if (controllerState.l1_pressed && controllerState.select_pressed) {
        scanI2CBus();
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
    setServoAngle(config::BALL_SERVO_CHANNEL, config::tuning::BALL_SERVO_CLOSE_ANGLE);
    setServoAngle(config::FRUIT_SERVO_CHANNEL, config::tuning::FRUIT_SERVO_CLOSE_ANGLE);
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

void Robot::performI2CHealthCheck() {
    const unsigned long start_time = micros();
    i2cHealthState.total_transactions++;

    #if !COMPETITION_MODE
    Serial.println("=== I2C Health Check ===");
    #endif

    // Test PCA9685 connection and response time
    const unsigned long pca_start = micros();
    Wire.beginTransmission(config::pca9685::I2C_ADDRESS);
    uint8_t pca_error = Wire.endTransmission();
    const unsigned long pca_time = micros() - pca_start;

    if (pca_error == 0) {
        i2cHealthState.pca9685_connected = true;
        i2cHealthState.pca9685_last_success = millis();
        i2cHealthState.pca9685_response_time_us = pca_time;
        #if !COMPETITION_MODE
        Serial.print("PCA9685: CONNECTED (");
        Serial.print(pca_time);
        Serial.println("μs response)");
        #endif
    } else {
        i2cHealthState.pca9685_connected = false;
        i2cHealthState.pca9685_error_count++;
        i2cHealthState.failed_transactions++;
        #if !COMPETITION_MODE
        Serial.print("PCA9685: DISCONNECTED (error ");
        Serial.print(pca_error);
        Serial.println(")");
        #endif
    }

    // // Test MPU6050 connection and response time (if enabled)
    // if (config::imu::ENABLE_IMU) {
    //     // Switch to MPU6050 I2C speed for testing
    //     Wire.setClock(config::imu::I2C_CLOCK_SPEED);

    //     const unsigned long mpu_start = micros();
    //     Wire.beginTransmission(config::imu::I2C_ADDRESS);
    //     uint8_t mpu_error = Wire.endTransmission();
    //     const unsigned long mpu_time = micros() - mpu_start;

    //     // Restore PCA9685 I2C speed
    //     Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);

    //     if (mpu_error == 0) {
    //         i2cHealthState.mpu6050_connected = true;
    //         i2cHealthState.mpu6050_last_success = millis();
    //         i2cHealthState.mpu6050_response_time_us = mpu_time;
    //         #if !COMPETITION_MODE
    //         Serial.print("MPU6050: CONNECTED (");
    //         Serial.print(mpu_time);
    //         Serial.println("μs response)");
    //         #endif
    //     } else {
    //         i2cHealthState.mpu6050_connected = false;
    //         i2cHealthState.mpu6050_error_count++;
    //         i2cHealthState.failed_transactions++;
    //         #if !COMPETITION_MODE
    //         Serial.print("MPU6050: DISCONNECTED (error ");
    //         Serial.print(mpu_error);
    //         Serial.println(")");
    //         #endif
    //     }
    // } else {
    //     #if !COMPETITION_MODE
    //     Serial.println("MPU6050: DISABLED");
    //     #endif
    // }

    // Update bus health status
    const float error_rate = (float)i2cHealthState.failed_transactions / i2cHealthState.total_transactions;
    i2cHealthState.bus_healthy = (error_rate < config::performance::I2C_MAX_ERROR_RATE);
    i2cHealthState.i2c_speed_hz = config::pca9685::I2C_CLOCK_SPEED;
    i2cHealthState.last_health_check = millis();

    // Check for slow response times
    if (i2cHealthState.pca9685_connected &&
        i2cHealthState.pca9685_response_time_us > config::performance::I2C_RESPONSE_TIMEOUT_US) {
        #if !COMPETITION_MODE
        Serial.print("WARNING: PCA9685 slow response (");
        Serial.print(i2cHealthState.pca9685_response_time_us);
        Serial.println("μs > 1000μs threshold)");
        #endif
    }

    if (config::imu::ENABLE_IMU && i2cHealthState.mpu6050_connected &&
        i2cHealthState.mpu6050_response_time_us > config::performance::I2C_RESPONSE_TIMEOUT_US) {
        #if !COMPETITION_MODE
        Serial.print("WARNING: MPU6050 slow response (");
        Serial.print(i2cHealthState.mpu6050_response_time_us);
        Serial.println("μs > 1000μs threshold)");
        #endif
    }

    const unsigned long total_time = micros() - start_time;
    #if !COMPETITION_MODE
    Serial.print("I2C Health Check completed in ");
    Serial.print(total_time);
    Serial.println("μs");
    Serial.println("========================");
    #endif
}

void Robot::printI2CDiagnostics() const {
    #if !COMPETITION_MODE
    Serial.println("=== I2C Diagnostics ===");

    // I2C Bus Configuration
    Serial.print("PCA9685 I2C Speed: ");
    Serial.print(config::pca9685::I2C_CLOCK_SPEED / 1000);
    Serial.println(" kHz");
    if (config::imu::ENABLE_IMU) {
        Serial.print("MPU6050 I2C Speed: ");
        Serial.print(config::imu::I2C_CLOCK_SPEED / 1000);
        Serial.println(" kHz");
    }
    Serial.println("SDA Pin: GPIO 21, SCL Pin: GPIO 22 (VIA v2023 defaults)");

    // Bus Health
    Serial.print("Bus Health: ");
    Serial.println(i2cHealthState.bus_healthy ? "HEALTHY" : "UNHEALTHY");
    Serial.print("Total Transactions: ");
    Serial.println(i2cHealthState.total_transactions);
    Serial.print("Failed Transactions: ");
    Serial.println(i2cHealthState.failed_transactions);

    if (i2cHealthState.total_transactions > 0) {
        float error_rate = (float)i2cHealthState.failed_transactions / i2cHealthState.total_transactions * 100;
        Serial.print("Error Rate: ");
        Serial.print(error_rate, 1); // One decimal place
        Serial.println("%");
    }

    // PCA9685 Status
    Serial.println("--- PCA9685 Motor Controller ---");
    Serial.print("Address: 0x");
    Serial.println(config::pca9685::I2C_ADDRESS, HEX);
    Serial.print("Status: ");
    Serial.println(i2cHealthState.pca9685_connected ? "CONNECTED" : "DISCONNECTED");

    if (i2cHealthState.pca9685_connected) {
        Serial.print("Response Time: ");
        Serial.print(i2cHealthState.pca9685_response_time_us);
        Serial.println("μs");
        Serial.print("Last Success: ");
        Serial.print(millis() - i2cHealthState.pca9685_last_success);
        Serial.println("ms ago");
    }

    Serial.print("Error Count: ");
    Serial.println(i2cHealthState.pca9685_error_count);

    // MPU6050 Status
    Serial.println("--- MPU6050 IMU Sensor ---");
    Serial.print("Address: 0x");
    Serial.println(config::imu::I2C_ADDRESS, HEX);
    Serial.print("Enabled: ");
    Serial.println(config::imu::ENABLE_IMU ? "YES" : "NO");

    if (config::imu::ENABLE_IMU) {
        Serial.print("Status: ");
        Serial.println(i2cHealthState.mpu6050_connected ? "CONNECTED" : "DISCONNECTED");

        if (i2cHealthState.mpu6050_connected) {
            Serial.print("Response Time: ");
            Serial.print(i2cHealthState.mpu6050_response_time_us);
            Serial.println("μs");
            Serial.print("Last Success: ");
            Serial.print(millis() - i2cHealthState.mpu6050_last_success);
            Serial.println("ms ago");
        }

        Serial.print("Error Count: ");
        Serial.println(i2cHealthState.mpu6050_error_count);
    }

    Serial.println("=======================");
    #endif
}

void Robot::testI2CPerformance() {
    #if !COMPETITION_MODE
    Serial.println("=== I2C Performance Test ===");
    Serial.println("Testing I2C speed and stability...");

    const int TEST_ITERATIONS = 100;
    unsigned long total_time = 0;
    unsigned long min_time = ULONG_MAX;
    unsigned long max_time = 0;
    int successful_transactions = 0;

    // Test PCA9685 performance
    Serial.print("Testing PCA9685 (");
    Serial.print(TEST_ITERATIONS);
    Serial.println(" transactions)...");

    for (int i = 0; i < TEST_ITERATIONS; i++) {
        const unsigned long start = micros();
        Wire.beginTransmission(config::pca9685::I2C_ADDRESS);
        uint8_t error = Wire.endTransmission();
        const unsigned long duration = micros() - start;

        if (error == 0) {
            successful_transactions++;
            total_time += duration;
            if (duration < min_time) min_time = duration;
            if (duration > max_time) max_time = duration;
        }

        // Small delay between transactions
        delayMicroseconds(100);
    }

    // Calculate statistics
    if (successful_transactions > 0) {
        const float avg_time = (float)total_time / successful_transactions;
        const float success_rate = (float)successful_transactions / TEST_ITERATIONS * 100;

        Serial.print("PCA9685 Results:");
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

        // Performance analysis
        if (avg_time > config::performance::I2C_RESPONSE_TIMEOUT_US) {
            Serial.println("WARNING: Average response time exceeds threshold!");
        }
        if (success_rate < 95.0) {
            Serial.println("WARNING: Low success rate indicates I2C issues!");
        }
        if (max_time > avg_time * 3) {
            Serial.println("WARNING: High response time variance detected!");
        }
    } else {
        Serial.println("PCA9685: NO SUCCESSFUL TRANSACTIONS!");
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
    PWMUtils::calculateDifferentialDrive(processedY, processedX, leftPWM, rightPWM, topDuty);
}

void Robot::processJoystickInputs(const ControllerState& controllerState, int& processedY, int& processedX) {
    // Apply deadzone to joystick inputs for better control
    processedY = PWMUtils::applyDeadzone(controllerState.left_joystick_y, config::ps2::JOYSTICK_DEADZONE);
    processedX = PWMUtils::applyDeadzone(controllerState.right_joystick_x, config::ps2::JOYSTICK_DEADZONE);
}

void Robot::setDriveMotorSpeeds(int leftPWM, int rightPWM) {
    // Apply PWM clamping for safety and set motor speeds
    int safeLeftPWM = PWMUtils::clampPWM(leftPWM);
    int safeRightPWM = PWMUtils::clampPWM(rightPWM);

    // Simple VEX/FTC-style motor control
    driveLeft.setSpeed(safeLeftPWM);
    driveRight.setSpeed(safeRightPWM);
}


void Robot::setOuttakeMotorSpeeds(int speed) {
    // Apply PWM clamping for safety
    int safeSpeed = PWMUtils::clampPWM(speed);

    // Apply braking when no outtake buttons are pressed (like drivetrain)
    if (safeSpeed == 0) {
        // Check for reverse hold mode (SELECT button pressed and limit switch encountered)
        if (reverseHoldModeActive) {
            // Reverse hold mode: apply 20% reverse holding power
            int reverseHoldPower = -PWMUtils::percentageToPWM(config::tuning::OUTTAKE_REVERSE_HOLD_POWER_PERCENT);
            outtakeLeft.setSpeed(reverseHoldPower);
            outtakeRight.setSpeed(reverseHoldPower);
            setServoAngle(config::BALL_SERVO_CHANNEL, config::tuning::BALL_SERVO_CLOSE_ANGLE);
            setServoAngle(config::FRUIT_SERVO_CHANNEL, config::tuning::FRUIT_SERVO_CLOSE_ANGLE);
        } else if (holdingPowerDisabled) {
            // SELECT button mode: no holding power at all
            outtakeLeft.setSpeed(0);
            outtakeRight.setSpeed(0);
        } else if (!config::tuning::LIMIT_SWITCH_DISABLED && readLimitSwitch()) {
            // Limit switch is active - stop motors and close both servos
            outtakeLeft.setSpeed(0);
            outtakeRight.setSpeed(0);
            setServoAngle(config::BALL_SERVO_CHANNEL, config::tuning::BALL_SERVO_CLOSE_ANGLE);
            setServoAngle(config::FRUIT_SERVO_CHANNEL, config::tuning::FRUIT_SERVO_CLOSE_ANGLE);

        } else {
            // Normal operation: apply holding power
            int holdPower = PWMUtils::percentageToPWM(config::tuning::OUTTAKE_HOLD_POWER_PERCENT);
            outtakeLeft.setSpeed(holdPower);
            outtakeRight.setSpeed(holdPower);
        }
    } else {
        // Normal speed control when moving
        outtakeLeft.setSpeed(safeSpeed);
        outtakeRight.setSpeed(safeSpeed);
    }
}

void Robot::stopAllMotors() {
    // Simple VEX/FTC-style motor control - braking handled automatically
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

void Robot::handleFruitIntakeServos(const ControllerState& controllerState) {
    // Handle continuous servo control for fruit intake
    // A button (Square) = down/intake, X button (Cross) = up/outtake

    if (controllerState.square_pressed && controllerState.cross_pressed) {
        // Both buttons pressed - stop both servos
        setServoSpeed(config::FRUIT_INTAKE_LEFT_SERVO_CHANNEL, 0);
        setServoSpeed(config::FRUIT_INTAKE_RIGHT_SERVO_CHANNEL, 0);
    } else if (controllerState.square_pressed) {
        // A button pressed - turn down/intake (servo 4 forward, servo 5 reverse)
        setServoSpeed(config::FRUIT_INTAKE_LEFT_SERVO_CHANNEL, config::tuning::FRUIT_INTAKE_SERVO_SPEED);
        setServoSpeed(config::FRUIT_INTAKE_RIGHT_SERVO_CHANNEL, -config::tuning::FRUIT_INTAKE_SERVO_SPEED);
    } else if (controllerState.cross_pressed) {
        // X button pressed - turn up/outtake (servo 4 reverse, servo 5 forward)
        setServoSpeed(config::FRUIT_INTAKE_LEFT_SERVO_CHANNEL, -config::tuning::FRUIT_INTAKE_SERVO_SPEED);
        setServoSpeed(config::FRUIT_INTAKE_RIGHT_SERVO_CHANNEL, config::tuning::FRUIT_INTAKE_SERVO_SPEED);
    } else {
        // No buttons pressed - stop both servos
        setServoSpeed(config::FRUIT_INTAKE_LEFT_SERVO_CHANNEL, 0);
        setServoSpeed(config::FRUIT_INTAKE_RIGHT_SERVO_CHANNEL, 0);
    }
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

        // Run startup sequence for visual feedback
        WS2812BStrip::startupSequence();

        DEBUG_PRINTLN("LED strip ready for normal operation");
    } else {
        ERROR_PRINTLN("WS2812B LED strip initialization failed");
    }
}

void Robot::updateLEDStrip() {
    // Update LED strip based on current robot state and system conditions
    config::RobotState currentState = stateMachine.getCurrentState();
    bool should_be_powered = false;
    uint8_t led_status = config::led_status::OFF;

    // Priority 1: System errors (highest priority)
    if (!isSystemHealthy() || !SafetyMonitor::isSystemSafe()) {
        // Check specific error types for more informative display
        if (!hal_pca9685_is_connected()) {
            led_status = config::led_status::I2C_ERROR;
        } else {
            // For now, use generic system error - temperature monitoring is internal to SafetyMonitor
            led_status = config::led_status::SYSTEM_ERROR;
        }
        should_be_powered = true;
    }
    // Priority 2: Controller safety shutdown
    else if (SafetyMonitor::isControllerSafetyShutdownActive()) {
        led_status = config::led_status::CONTROLLER_SAFETY;
        should_be_powered = true;
    }
    // Priority 3: Controller connection status
    else if (!isConnected()) {
        led_status = config::led_status::CONTROLLER_DISCONNECTED;
        should_be_powered = true;
    }
    // Priority 4: Limit switch status (when active and not disabled)
    else if (!config::tuning::LIMIT_SWITCH_DISABLED && readLimitSwitch()) {
        led_status = config::led_status::LIMIT_SWITCH_ACTIVE;
        should_be_powered = true;
    }
    // Priority 5: Normal robot state
    else {
        switch (currentState) {
            case config::IDLE:
                led_status = config::led_status::IDLE;
                should_be_powered = true;
                break;
            case config::MANUAL_CONTROL:
                led_status = config::led_status::MANUAL_CONTROL;
                should_be_powered = true;
                break;
            case config::AUTOMATIC_OUTTAKE_REVERSE:
            case config::TIMED_OUTTAKE_FORWARD:
                led_status = config::led_status::AUTOMATIC_MODE;
                should_be_powered = true;
                break;
            default:
                led_status = config::led_status::OFF;
                should_be_powered = false;
                break;
        }
    }

    // Set the LED status
    WS2812BStrip::setRobotStatus(led_status);

    // Control power based on state
    if (should_be_powered && !WS2812BStrip::isPowerOn()) {
        WS2812BStrip::powerOn();
    } else if (!should_be_powered && WS2812BStrip::isPowerOn()) {
        WS2812BStrip::powerOff();
    }

    // Update the strip
    WS2812BStrip::update();
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

