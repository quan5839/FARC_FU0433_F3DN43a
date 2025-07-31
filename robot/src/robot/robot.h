#ifndef ROBOT_H
#define ROBOT_H

#include "../hal/motor.h" // Use relative path for linter compatibility
#include "../hal/imu.h"   // IMU sensor support
#include "../hal/ws2812b_strip.h" // WS2812B LED strip control
// NOTE: Linter errors about includes can be ignored; code compiles with Arduino CLI/IDE.
#include "../config.h"
#include "../controller/controller_state.h"
#include "state_machine.h"
#include <stdint.h> // For uint8_t, etc. (linter compatibility)
#ifndef HIGH
#define HIGH 1
#endif

/**
 * @class Robot
 * @brief Encapsulates all robot state, motors, and high-level logic for the Arduino robot project.
 *
 * - Handles drive, outtake, and servo control.
 * - Implements a state machine for manual and automatic actions.
 * - Provides safety features (limit switch, debouncing).
 */
class Robot {
public:
    /**
     * @brief Construct a new Robot object and initialize hardware members.
     */
    Robot();

    /**
     * @brief Initialize robot hardware and state.
     */
    void init();

    /**
     * @brief Main loop for periodic robot logic.
     */
    void loop();

    /**
     * @brief Process controller input and update robot state.
     * @param controllerState The current state of the controller.
     */
    void processControllerInput(const ControllerState& controllerState);

    /**
     * @brief Start automatic outtake reverse sequence.
     */
    void automaticOuttakeReverse();

    /**
     * @brief Start timed outtake forward sequence.
     */
    void timedOuttakeForward();

    /**
     * @brief Set the robot's current state.
     * @param newState The new state to set.
     */
    void setRobotState(config::RobotState newState);

    /**
     * @brief Get current robot state.
     * @return Current robot state.
     */
    config::RobotState getRobotState() const;

    /**
     * @brief Read the debounced limit switch state.
     * @return true if the limit switch is active, false otherwise.
     */
    bool readLimitSwitch();

    /**
     * @brief Perform comprehensive I2C health diagnostics.
     */
    void performI2CHealthCheck();

    /**
     * @brief Print I2C diagnostic information.
     */
    void printI2CDiagnostics() const;

    /**
     * @brief Test I2C speed and stability with multiple transactions.
     */
    void testI2CPerformance();

    /**
     * @brief Scan I2C bus for connected devices.
     */
    void scanI2CBus();

    /**
     * @brief Initialize WS2812B LED strip
     */
    void initLEDStrip();

    /**
     * @brief Update LED strip based on robot status
     */
    void updateLEDStrip();

    /**
     * @brief Test IMU functionality and print readings
     */
    void testIMU();

    /**
     * @brief Print current IMU readings to serial
     */
    void printIMUReadings();

    /**
     * @brief Test LED strip hardware directly (bypasses power control)
     */
    void testLEDStripHardware();



private:
    void handleDriveInput(const ControllerState& controllerState);
    void handleOuttakeInput(const ControllerState& controllerState);
    void handleServoInput(const ControllerState& controllerState);
    void handleFruitIntakeServos(const ControllerState& controllerState);
    void handleSpecialCommands(const ControllerState& controllerState);

    // Decomposed helper functions for better organization
    void calculateDriveMotorSpeeds(const ControllerState& controllerState, int& leftPWM, int& rightPWM);
    void processJoystickInputs(const ControllerState& controllerState, int& processedY, int& processedX);
    void setDriveMotorSpeeds(int leftPWM, int rightPWM);
    void setOuttakeMotorSpeeds(int speed);
    void stopAllMotors();
    void brakeAllMotors();  // FTC-style active braking
    void coastAllMotors();  // Coast to stop
    bool handleServoToggle(uint8_t channel, bool& toggleState, int openAngle, int closeAngle);

    // State machine
    StateMachine stateMachine;

    // Track if automatic outtake reverse was started by select button
    bool selectButtonOuttakeActive;

    // Track if holding power should be disabled (controlled by SELECT button, independent of limit switch)
    bool holdingPowerDisabled;

    // Track if reverse hold mode is active (SELECT button pressed and limit switch encountered)
    bool reverseHoldModeActive;

    // Track if L2 was pressed during limit switch override mode
    bool l2PressedDuringOverride;

    // Track if controls are locked to outtake only (SELECT button mode)
    bool controlsLockedToOuttake;

    // Error handling and recovery
    int _consecutive_errors = 0;
    unsigned long _last_error_time = 0;
    bool _emergency_stop_active = false;

    void handleSystemError();
    void resetErrorCounter();
    bool isSystemHealthy();



    // Limit switch debouncing state
    struct LimitSwitchState {
        int lastState = HIGH;
        int lastRawReading = HIGH;
        unsigned long lastDebounceTime = 0;
    } limitSwitchState;



    // I2C health monitoring state
    struct I2CHealthState {
        // PCA9685 health
        bool pca9685_connected = false;
        unsigned long pca9685_last_success = 0;
        unsigned long pca9685_error_count = 0;
        unsigned long pca9685_response_time_us = 0;

        // MPU6050 health
        bool mpu6050_connected = false;
        unsigned long mpu6050_last_success = 0;
        unsigned long mpu6050_error_count = 0;
        unsigned long mpu6050_response_time_us = 0;

        // I2C bus health
        unsigned long i2c_speed_hz = 0;
        unsigned long last_health_check = 0;
        bool bus_healthy = true;
        unsigned long total_transactions = 0;
        unsigned long failed_transactions = 0;
    } i2cHealthState;

    Motor driveLeft;
    Motor driveRight;
    Motor outtakeLeft;
    Motor outtakeRight;

    // Servo toggle state
    bool ballServoToggled = false;
    bool fruitServoToggled = false;
};

#endif // ROBOT_H