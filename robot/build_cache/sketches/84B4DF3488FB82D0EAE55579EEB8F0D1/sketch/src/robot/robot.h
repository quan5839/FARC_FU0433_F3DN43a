#line 1 "/Users/admin/Documents/Arduino/robot/src/robot/robot.h"
#ifndef ROBOT_H
#define ROBOT_H

#include "../hal/motor.h" // Use relative path for linter compatibility
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

private:
    void handleDriveInput(const ControllerState& controllerState);
    void handleOuttakeInput(const ControllerState& controllerState);
    void handleServoInput(const ControllerState& controllerState);
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
    bool detectTurningMovement(int leftPWM, int rightPWM);

    // State machine
    StateMachine stateMachine;

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

    Motor driveLeft;
    Motor driveRight;
    Motor outtakeLeft;
    Motor outtakeRight;

    // Servo toggle state
    bool outtakeArmToggled = false;
    bool intakeArmToggled = false;
};

#endif // ROBOT_H