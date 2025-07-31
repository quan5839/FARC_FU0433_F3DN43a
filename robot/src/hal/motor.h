#ifndef MOTOR_H
#define MOTOR_H

#include "../config.h"

// Motor types for different brake configurations
enum class MotorType {
    DRIVE,
    OUTTAKE
};

// BrakeMode removed - using electromagnetic braking only for FTC authenticity

/**
 * @class Motor
 * @brief Simple VEX/FTC-style motor controller with automatic braking optimization
 *
 * This class provides easy-to-use motor control similar to VEX and FTC frameworks,
 * with sophisticated mixed-decay braking handled automatically behind the scenes.
 * No need to worry about turning context or braking parameters.
 *
 * Features:
 * - Simple setSpeed(pwm) interface like VEX motor.spin() or FTC motor.setPower()
 * - Automatic mixed-decay braking (fast decay + electromagnetic braking)
 * - Speed-dependent brake timing for optimal stopping performance
 * - Directional inversion support
 * - Smooth acceleration/deceleration ramping
 * - Optimized PWM updates (only when values change)
 */
class Motor {
public:
    Motor(uint8_t fwd_channel, uint8_t rev_channel, bool inverted, MotorType type = MotorType::DRIVE);

    void setSpeed(int pwm);  // Simple VEX/FTC-style interface
    void brake();  // Apply FTC-style active braking
    void coast();  // Coast to stop (disable active braking)
    void update();

    // Getter methods
    int getCurrentPWM() const { return _current_pwm; }

private:
    void updateBraking();
    void updateRamping();
    bool shouldApplyBraking(int current_speed, int target_speed);
    int calculateBrakePower(int current_speed);
    void applyElectromagneticBrake(int brake_power);
    void applyFastDecay();  // Coast motor (both channels off)
    void setMotorPWMDirect(int pwm);  // Direct PWM setting without ramping
    void setMotorSpeed(int speed);  // Low-level motor control (moved from motors.h)

    // Mixed-decay helper functions
    unsigned long calculateTotalBrakeTime(int current_speed);
    void initializeMixedDecayBraking(int current_speed);

    uint8_t _fwd_channel;
    uint8_t _rev_channel;
    bool _inverted;
    MotorType _motor_type;

    int _current_pwm = 0;
    int _target_pwm = 0;
    int _last_sent_pwm = -1;

    // Mixed-decay braking state
    bool _is_braking = false;
    unsigned long _brake_start_time = 0;
    int _last_speed_before_brake = 0;
    unsigned long _total_brake_time = 0;     // Calculated total brake time based on speed
    unsigned long _fast_decay_time = 0;      // Duration of fast decay phase
    unsigned long _slow_decay_time = 0;      // Duration of slow decay phase

    // Motor ramping state
    unsigned long _last_ramp_time = 0;
    bool _ramping_initialized = false;
};

#endif // MOTOR_H
