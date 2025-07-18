#line 1 "/Users/admin/Documents/Arduino/robot/src/robot/state_machine.h"
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "../config.h"

/**
 * @class StateMachine
 * @brief Manages robot state transitions, timeouts, and state-specific logic
 * 
 * This class encapsulates all state machine functionality, providing a clean
 * interface for state management and removing complexity from the main Robot class.
 * 
 * Features:
 * - State transition management with logging
 * - Automatic timeout handling for timed states
 * - State-specific timeout configuration
 * - Debug utilities for state monitoring
 */
class StateMachine {
public:
    /**
     * @brief Construct a new StateMachine object
     */
    StateMachine();

    /**
     * @brief Initialize the state machine
     */
    void init();

    /**
     * @brief Get the current robot state
     * @return Current state
     */
    config::RobotState getCurrentState() const;

    /**
     * @brief Set a new robot state with transition logging
     * @param newState The new state to transition to
     */
    void setState(config::RobotState newState);

    /**
     * @brief Check if the current state has timed out
     * @return true if state has exceeded its timeout, false otherwise
     */
    bool isStateTimedOut() const;

    /**
     * @brief Handle state timeout by transitioning to safe state
     */
    void handleTimeout();

    /**
     * @brief Get the elapsed time in the current state
     * @return Milliseconds since state started
     */
    unsigned long getStateElapsedTime() const;

    /**
     * @brief Get the timeout duration for a specific state
     * @param state The state to get timeout for
     * @return Timeout in milliseconds (0 = no timeout)
     */
    unsigned long getStateTimeoutMs(config::RobotState state) const;

    /**
     * @brief Get human-readable name for a state (for debugging)
     * @param state The state to get name for
     * @return State name string
     */
    const char* getStateName(config::RobotState state) const;

    /**
     * @brief Check if a state is a timed automatic state
     * @param state The state to check
     * @return true if state has automatic timeout behavior
     */
    bool isTimedState(config::RobotState state) const;

private:
    config::RobotState _currentState;
    unsigned long _stateStartTime;

    /**
     * @brief Log state transition for debugging
     * @param fromState Previous state
     * @param toState New state
     */
    void logStateTransition(config::RobotState fromState, config::RobotState toState);
};

#endif // STATE_MACHINE_H
