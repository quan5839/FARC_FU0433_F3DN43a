#line 1 "/Users/admin/Documents/Arduino/robot/src/hal/motor.cpp"
#include "motor.h"
#include "pca9685_driver.h"
#include "../config.h"

// Returns -1, 0, or 1 based on the sign of the input.
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

Motor::Motor(uint8_t fwd_channel, uint8_t rev_channel, bool inverted) {
    _fwd_channel = fwd_channel;
    _rev_channel = rev_channel;
    _inverted = inverted;
}

void Motor::setSpeed(int pwm) {
    int safe_pwm = _inverted ? -pwm : pwm;

    // Check if we should apply FTC-style braking
    if (config::tuning::ENABLE_ACTIVE_BRAKING && shouldApplyBraking(_current_pwm, safe_pwm)) {
        _is_braking = true;
        _brake_start_time = millis();
        _last_speed_before_brake = _current_pwm;
    }

    _target_pwm = safe_pwm;
    _is_turning = false;
}

void Motor::setSpeed(int pwm, bool is_turning) {
    int safe_pwm = _inverted ? -pwm : pwm;

    // Check if we should apply FTC-style braking
    if (config::tuning::ENABLE_ACTIVE_BRAKING && shouldApplyBraking(_current_pwm, safe_pwm)) {
        _is_braking = true;
        _brake_start_time = millis();
        _last_speed_before_brake = _current_pwm;
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
    } else {
        coast();
    }
}

void Motor::coast() {
    _target_pwm = 0;
    _is_braking = false;
}

void Motor::update() {
    // Handle FTC-style braking first
    updateBraking();

    // If not braking, apply ramping
    if (!_is_braking) {
        updateRamping();

        if (_current_pwm != _last_sent_pwm) {
            setMotorSpeed(_current_pwm);
            _last_sent_pwm = _current_pwm;
        }
    }
}



void Motor::updateBraking() {
    if (!_is_braking) return;

    unsigned long brake_duration = millis() - _brake_start_time;

    if (brake_duration < config::tuning::BRAKE_TIME_MS) {
        // Apply active braking based on configuration
        int brake_power = calculateBrakePower(_last_speed_before_brake);

        // True FTC-style: Short motor terminals for electromagnetic braking
        applyElectromagneticBrake(brake_power);

        _current_pwm = 0;  // Set internal state to stopped
        _last_sent_pwm = 0;
    } else {
        // Braking time expired, switch to holding or stop
        _is_braking = false;
        _current_pwm = 0;

        // Optional: Apply low holding power to maintain position
        if (config::tuning::BRAKE_HOLD_POWER_PERCENT > 0) {
            int hold_power = (config::motor::MAX_PWM * config::tuning::BRAKE_HOLD_POWER_PERCENT) / 100;
            applyElectromagneticBrake(hold_power);
        } else {
            setMotorSpeed(0);
        }
        _last_sent_pwm = 0;
    }
}

void Motor::updateRamping() {
    if (!config::tuning::ENABLE_MOTOR_RAMPING) {
        // Ramping disabled - set target directly
        _current_pwm = _target_pwm;
        return;
    }

    unsigned long current_time = millis();

    // Initialize timing on first call
    if (!_ramping_initialized) {
        _last_ramp_time = current_time;
        _ramping_initialized = true;
        _current_pwm = _target_pwm; // Start immediately at target on first call
        return;
    }

    // Calculate time delta
    unsigned long delta_time = current_time - _last_ramp_time;
    _last_ramp_time = current_time;

    // If target equals current, no change needed
    if (_target_pwm == _current_pwm) {
        return;
    }

    // Determine if we're accelerating or decelerating
    bool accelerating = abs(_target_pwm) > abs(_current_pwm);
    int ramp_time = accelerating ? config::tuning::ACCELERATION_TIME_MS : config::tuning::DECELERATION_TIME_MS;

    // Calculate maximum change allowed in this time step
    int max_change_per_ms = config::motor::MAX_PWM / ramp_time;
    int max_change = max_change_per_ms * delta_time;

    // Ensure minimum change of 1 to prevent stalling
    if (max_change < 1) {
        max_change = 1;
    }

    // Apply ramping
    int pwm_difference = _target_pwm - _current_pwm;

    if (abs(pwm_difference) <= max_change) {
        // Can reach target in this step
        _current_pwm = _target_pwm;
    } else {
        // Apply limited change toward target
        if (pwm_difference > 0) {
            _current_pwm += max_change;
        } else {
            _current_pwm -= max_change;
        }
    }
}

bool Motor::shouldApplyBraking(int current_speed, int target_speed) {
    // Only brake if:
    // 1. We're moving at significant speed
    // 2. Target is stop (0) or opposite direction
    // 3. Speed is above threshold to avoid unnecessary braking at low speeds

    int current_speed_percent = (abs(current_speed) * 100) / config::motor::MAX_PWM;
    bool moving_fast_enough = current_speed_percent >= config::tuning::BRAKE_THRESHOLD_PERCENT;
    bool stopping = (target_speed == 0);
    bool reversing = (current_speed > 0 && target_speed < 0) || (current_speed < 0 && target_speed > 0);

    return moving_fast_enough && (stopping || reversing);
}

int Motor::calculateBrakePower(int current_speed) {
    // Calculate brake power based on current speed and configuration
    int base_brake_power = (config::motor::MAX_PWM * config::tuning::BRAKE_POWER_PERCENT) / 100;

    // Scale brake power based on current speed (more speed = more braking)
    int speed_percent = (abs(current_speed) * 100) / config::motor::MAX_PWM;
    int scaled_brake_power = (base_brake_power * speed_percent) / 100;

    // Ensure minimum and maximum brake power
    return max(base_brake_power / 4, min(base_brake_power, scaled_brake_power));
}

void Motor::applyElectromagneticBrake(int brake_power) {
    // True FTC-style electromagnetic braking: Short motor terminals
    // Set both forward and reverse channels to same PWM value
    // This creates a current path through the motor, causing electromagnetic braking
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
