#line 1 "/Users/admin/Documents/Arduino/robot/src/hal/motor.h"
#ifndef MOTOR_H
#define MOTOR_H

#include "../config.h"

// BrakeMode removed - using electromagnetic braking only for FTC authenticity

/**
 * @class Motor
 * @brief High-performance motor controller with FTC-style electromagnetic braking
 *
 * This class provides precise motor control using electromagnetic braking for
 * immediate stopping and position holding. Optimized for 6-wheel drive robots
 * with high traction requirements.
 *
 * Features:
 * - FTC-style electromagnetic braking (terminal shorting)
 * - Configurable acceleration/deceleration ramping (150ms/75ms)
 * - Position holding capability
 * - Turn-aware braking optimization
 * - Directional inversion support
 * - Optimized PWM updates (only when values change)
 */
class Motor {
public:
    Motor(uint8_t fwd_channel, uint8_t rev_channel, bool inverted);

    void setSpeed(int pwm);
    void setSpeed(int pwm, bool is_turning);  // Enhanced version with turn context
    void brake();  // Apply FTC-style active braking
    void coast();  // Coast to stop (disable active braking)
    void update();

private:
    void updateBraking();
    void updateRamping();
    bool shouldApplyBraking(int current_speed, int target_speed);
    int calculateBrakePower(int current_speed);
    void applyElectromagneticBrake(int brake_power);
    void setMotorPWMDirect(int pwm);  // Direct PWM setting without ramping
    void setMotorSpeed(int speed);  // Low-level motor control (moved from motors.h)

    uint8_t _fwd_channel;
    uint8_t _rev_channel;
    bool _inverted;

    int _current_pwm = 0;
    int _target_pwm = 0;
    int _last_sent_pwm = -1;
    bool _is_turning = false;  // Track if this motor is part of a turning movement

    // FTC-style electromagnetic braking state
    bool _is_braking = false;
    unsigned long _brake_start_time = 0;
    int _last_speed_before_brake = 0;

    // Motor ramping state
    unsigned long _last_ramp_time = 0;
    bool _ramping_initialized = false;
};

#endif // MOTOR_H
