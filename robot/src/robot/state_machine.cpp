#include "state_machine.h"
#include "../config.h"
#include <Arduino.h>

StateMachine::StateMachine() 
    : _currentState(config::IDLE), _stateStartTime(0) {
}

void StateMachine::init() {
    _currentState = config::IDLE;
    _stateStartTime = millis();
}

config::RobotState StateMachine::getCurrentState() const {
    return _currentState;
}

void StateMachine::setState(config::RobotState newState) {
    if (_currentState != newState) {
        logStateTransition(_currentState, newState);
        _currentState = newState;
        _stateStartTime = millis();
    }
}

bool StateMachine::isStateTimedOut() const {
    unsigned long timeoutMs = getStateTimeoutMs(_currentState);
    
    // Only check timeout for states that have timeouts
    if (timeoutMs == 0) {
        return false;
    }
    
    unsigned long elapsed = millis() - _stateStartTime;
    return elapsed > timeoutMs;
}

void StateMachine::handleTimeout() {
    ERROR_PRINT("State timeout: ");
    ERROR_PRINT(getStateName(_currentState));
    ERROR_PRINT(" after ");
    ERROR_PRINT(millis() - _stateStartTime);
    ERROR_PRINTLN("ms");

    // Transition to safe state
    setState(config::MANUAL_CONTROL);
}

unsigned long StateMachine::getStateElapsedTime() const {
    return millis() - _stateStartTime;
}

unsigned long StateMachine::getStateTimeoutMs(config::RobotState state) const {
    switch (state) {
        case config::AUTOMATIC_OUTTAKE_REVERSE:
            return config::tuning::OUTTAKE_TIMEOUT_MS + config::constants::TIMEOUT_SAFETY_MARGIN_MS;
        case config::TIMED_OUTTAKE_FORWARD:
            return config::tuning::OUTTAKE_FORWARD_TIMEOUT_MS + config::constants::TIMEOUT_SAFETY_MARGIN_MS;
        // Homing sequence states have timeouts for safety
        case config::HOMING_INITIAL_CLEARANCE:
        case config::HOMING_FAST_APPROACH:
        case config::HOMING_RETRACTION:
        case config::HOMING_SLOW_APPROACH:
        case config::HOMING_FINAL_POSITION:
            return config::tuning::HOMING_TIMEOUT_MS;
        case config::IDLE:
        case config::MANUAL_CONTROL:
        case config::HOMING_COMPLETE:
        default:
            return 0; // No timeout for these states
    }
}

const char* StateMachine::getStateName(config::RobotState state) const {
    switch (state) {
        case config::IDLE: return "IDLE";
        case config::MANUAL_CONTROL: return "MANUAL_CONTROL";
        case config::AUTOMATIC_OUTTAKE_REVERSE: return "AUTOMATIC_OUTTAKE_REVERSE";
        case config::TIMED_OUTTAKE_FORWARD: return "TIMED_OUTTAKE_FORWARD";
        case config::HOMING_INITIAL_CLEARANCE: return "HOMING_INITIAL_CLEARANCE";
        case config::HOMING_FAST_APPROACH: return "HOMING_FAST_APPROACH";
        case config::HOMING_RETRACTION: return "HOMING_RETRACTION";
        case config::HOMING_SLOW_APPROACH: return "HOMING_SLOW_APPROACH";
        case config::HOMING_FINAL_POSITION: return "HOMING_FINAL_POSITION";
        case config::HOMING_COMPLETE: return "HOMING_COMPLETE";
        default: return "UNKNOWN";
    }
}

bool StateMachine::isTimedState(config::RobotState state) const {
    return getStateTimeoutMs(state) > 0;
}

void StateMachine::logStateTransition(config::RobotState fromState, config::RobotState toState) {
    DEBUG_PRINT("State transition: ");
    DEBUG_PRINT(getStateName(fromState));
    DEBUG_PRINT(" -> ");
    DEBUG_PRINTLN(getStateName(toState));
}
