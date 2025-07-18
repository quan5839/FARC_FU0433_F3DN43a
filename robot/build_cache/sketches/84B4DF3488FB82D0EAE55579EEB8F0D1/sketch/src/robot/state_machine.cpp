#line 1 "/Users/admin/Documents/Arduino/robot/src/robot/state_machine.cpp"
#include "state_machine.h"
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
    Serial.print("State timeout: ");
    Serial.print(getStateName(_currentState));
    Serial.print(" after ");
    Serial.print(millis() - _stateStartTime);
    Serial.println("ms");
    
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
        case config::IDLE:
        case config::MANUAL_CONTROL:
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
        default: return "UNKNOWN";
    }
}

bool StateMachine::isTimedState(config::RobotState state) const {
    return getStateTimeoutMs(state) > 0;
}

void StateMachine::logStateTransition(config::RobotState fromState, config::RobotState toState) {
    Serial.print("State transition: ");
    Serial.print(getStateName(fromState));
    Serial.print(" -> ");
    Serial.println(getStateName(toState));
}
