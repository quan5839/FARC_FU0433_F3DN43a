#include "motor.h"
#include "pca9685_driver.h"
#include "../config.h"
#include <Arduino.h>

// Performance optimization pragmas
#pragma GCC optimize("O3")
#pragma GCC optimize("fast-math")
#pragma GCC optimize("unroll-loops")

// Returns -1, 0, or 1 based on the sign of the input.
template <typename T> constexpr int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Motor::Motor(uint8_t fwd_channel, uint8_t rev_channel, bool inverted) {
    _fwd_channel = fwd_channel;
    _rev_channel = rev_channel;
    _inverted = inverted;
}

void Motor::setSpeed(int pwm, bool is_turning) {
    int safe_pwm = _inverted ? -pwm : pwm;

    // Exit braking mode if new non-zero speed is commanded
    if (_is_braking && safe_pwm != 0) {
        _is_braking = false;
    }

    // Check if we should apply mixed-decay braking
    if (config::tuning::ENABLE_ACTIVE_BRAKING && shouldApplyBraking(_current_pwm, safe_pwm)) {
        _is_braking = true;
        _brake_start_time = millis();
        _last_speed_before_brake = _current_pwm;
        initializeMixedDecayBraking(_current_pwm);
    }

    _target_pwm = safe_pwm;
    _is_turning = is_turning;
}

void Motor::brake() {
    if (config::tuning::ENABLE_ACTIVE_BRAKING) {
        _target_pwm = 0;
        _is_braking = true;
        _brake_start_time = millis();
        _last_speed_before_brake = _current_pwm;
        initializeMixedDecayBraking(_current_pwm);
    } else {
        coast();
    }
}

void Motor::coast() {
    _target_pwm = 0;
    _is_braking = false;
}

void Motor::update() {
    // Two-Stage Deceleration Control:
    // Stage 1: Ramping handles smooth deceleration from high to low speed
    // Stage 2: Braking handles final stop and direction changes

    // Handle mixed-decay braking (Stage 2) - optimized with branch prediction
    if (UNLIKELY(_is_braking)) {
        updateBraking();
    } else {
        // Most common path: ramping for smooth acceleration/deceleration (Stage 1)
        updateRamping();

        if (LIKELY(_current_pwm != _last_sent_pwm)) {
            setMotorSpeed(_current_pwm);
            _last_sent_pwm = _current_pwm;
        }
    }
}



void Motor::updateBraking() {
    if (!_is_braking) return;

    unsigned long brake_duration = millis() - _brake_start_time;

    // Mixed-Decay Braking System:
    // Phase 1: Fast Decay (coasting) - motor terminals disconnected
    // Phase 2: Slow Decay (electromagnetic braking) - motor terminals shorted
    // Phase 3: Hold (low power electromagnetic braking) - position maintenance

    if (brake_duration < _fast_decay_time) {
        // Phase 1: Fast Decay - Coast the motor (both channels off)
        applyFastDecay();
        _current_pwm = 0;
        _last_sent_pwm = 0;

    } else if (brake_duration < _total_brake_time) {
        // Phase 2: Slow Decay - Electromagnetic braking
        int brake_power = calculateBrakePower(_last_speed_before_brake);
        applyElectromagneticBrake(brake_power);
        _current_pwm = 0;
        _last_sent_pwm = 0;

    } else {
        // Phase 3: Hold - Braking complete, switch to holding or stop
        _is_braking = false;
        _current_pwm = 0;

        // Apply low holding power to maintain position
        if (config::tuning::BRAKE_HOLD_POWER_PERCENT > 0) {
            int hold_power = (config::constants::PWM_MAX * config::tuning::BRAKE_HOLD_POWER_PERCENT) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;
            applyElectromagneticBrake(hold_power);
        } else {
            setMotorSpeed(0);
        }
        _last_sent_pwm = 0;
    }
}

void Motor::updateRamping() {
    // Early exit if ramping disabled
    if (!config::tuning::ENABLE_MOTOR_RAMPING) {
        _current_pwm = _target_pwm;
        return;
    }

    // Early exit if already at target
    if (_target_pwm == _current_pwm) {
        return;
    }

    const unsigned long current_time = millis();

    // Initialize timing on first call
    if (!_ramping_initialized) {
        _last_ramp_time = current_time;
        _ramping_initialized = true;
        _current_pwm = _target_pwm; // Start immediately at target on first call
        return;
    }

    // Calculate time delta and update timestamp
    const unsigned long delta_time = current_time - _last_ramp_time;
    _last_ramp_time = current_time;

    // Performance-optimized: cache absolute values using fast math
    const int abs_target = FAST_ABS(_target_pwm);
    const int abs_current = FAST_ABS(_current_pwm);

    // Determine ramp time based on acceleration/deceleration
    const int ramp_time = (abs_target > abs_current) ?
        config::tuning::ACCELERATION_TIME_MS : config::tuning::DECELERATION_TIME_MS;

    // Calculate maximum change allowed in this time step
    const int max_change_per_ms = config::constants::PWM_MAX / ramp_time;
    int max_change = max_change_per_ms * delta_time;

    // Ensure minimum change of 1 to prevent stalling
    if (max_change < 1) {
        max_change = 1;
    }

    // Apply ramping with optimized logic
    const int pwm_difference = _target_pwm - _current_pwm;
    const int abs_difference = FAST_ABS(pwm_difference);

    if (abs_difference <= max_change) {
        // Can reach target in this step
        _current_pwm = _target_pwm;
    } else {
        // Apply limited change toward target
        _current_pwm += (pwm_difference > 0) ? max_change : -max_change;
    }
}

bool Motor::shouldApplyBraking(int current_speed, int target_speed) {
    // Mixed-Decay Braking Trigger Logic:
    // Apply braking for direction changes and stops when moving above threshold speed

    // Cache expensive calculations - these are compile-time constants
    static const int actual_max_pwm = (config::tuning::DRIVE_MAX_SPEED_PERCENT * config::constants::PWM_MAX) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;
    static const int brake_threshold_pwm = (config::tuning::BRAKE_THRESHOLD_PERCENT * actual_max_pwm) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;

    const int abs_current_pwm = abs(_current_pwm);
    const bool moving_fast_enough = abs_current_pwm >= brake_threshold_pwm;

    // Brake for direction changes
    bool reversing = (current_speed > 0 && target_speed < 0) ||
                     (current_speed < 0 && target_speed > 0);

    // Brake for stops
    bool stopping = (target_speed == 0);

    // Apply mixed-decay braking if moving fast enough and (reversing OR stopping)
    return moving_fast_enough && (reversing || stopping);
}

int Motor::calculateBrakePower(int current_speed) {
    // Cache expensive calculation - this is a compile-time constant
    static const int base_brake_power = (config::constants::PWM_MAX * config::tuning::BRAKE_POWER_PERCENT) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;
    static const int min_brake_power = base_brake_power / config::constants::QUARTER_DIVISOR;

    // Scale brake power based on current speed (more speed = more braking)
    const int speed_percent = (abs(current_speed) * config::constants::PERCENT_TO_DECIMAL_DIVISOR) / config::constants::PWM_MAX;
    const int scaled_brake_power = (base_brake_power * speed_percent) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;

    // Ensure minimum and maximum brake power using optimized min/max
    return (scaled_brake_power < min_brake_power) ? min_brake_power :
           (scaled_brake_power > base_brake_power) ? base_brake_power : scaled_brake_power;
}

void Motor::applyElectromagneticBrake(int brake_power) {
    // True FTC-style electromagnetic braking: Short motor terminals
    // Set both forward and reverse channels to same PWM value
    // This creates a current path through the motor, causing electromagnetic braking

    // TA6586: Both inputs HIGH = Brake mode (electromagnetic braking)

    hal_pca9685_set_pwm(_fwd_channel, brake_power);
    hal_pca9685_set_pwm(_rev_channel, brake_power);
}

// Reverse braking removed - using electromagnetic braking only for better FTC authenticity

void Motor::setMotorSpeed(int speed) {
    if (speed > 0) {
        hal_pca9685_set_pwm(_fwd_channel, speed);
        hal_pca9685_set_pwm(_rev_channel, 0);
    } else {
        hal_pca9685_set_pwm(_fwd_channel, 0);
        hal_pca9685_set_pwm(_rev_channel, -speed);
    }
}

void Motor::applyFastDecay() {
    // Fast decay: Disconnect motor terminals (both channels off)
    // This allows the motor to coast freely with minimal resistance
    hal_pca9685_set_pwm(_fwd_channel, 0);
    hal_pca9685_set_pwm(_rev_channel, 0);
}

unsigned long Motor::calculateTotalBrakeTime(int current_speed) {
    // Calculate total brake time based on current motor speed
    // Higher speeds get longer brake times for better control

    const int speed_percent = (abs(current_speed) * config::constants::PERCENT_TO_DECIMAL_DIVISOR) / config::constants::PWM_MAX;

    // Linear interpolation between base time and max time based on speed
    const unsigned long time_range = config::tuning::BRAKE_MAX_TIME_MS - config::tuning::BRAKE_BASE_TIME_MS;
    const unsigned long speed_based_time = (time_range * speed_percent) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;

    return config::tuning::BRAKE_BASE_TIME_MS + speed_based_time;
}

void Motor::initializeMixedDecayBraking(int current_speed) {
    // Calculate timing for mixed-decay braking phases
    _total_brake_time = calculateTotalBrakeTime(current_speed);

    // Calculate phase durations based on percentages
    _fast_decay_time = (_total_brake_time * config::tuning::FAST_DECAY_PERCENT) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;
    _slow_decay_time = (_total_brake_time * config::tuning::SLOW_DECAY_PERCENT) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;

    // Ensure fast + slow decay times don't exceed total time
    if (_fast_decay_time + _slow_decay_time > _total_brake_time) {
        _slow_decay_time = _total_brake_time - _fast_decay_time;
    }
}
