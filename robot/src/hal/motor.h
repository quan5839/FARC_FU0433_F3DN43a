#ifndef MOTOR_H
#define MOTOR_H

#include "../config.h"

// BrakeMode removed - using electromagnetic braking only for FTC authenticity

/**
 * @class Motor
 * @brief High-performance motor controller with mixed-decay braking system
 *
 * This class provides precise motor control using a sophisticated mixed-decay
 * braking system that combines fast decay (coasting) and slow decay (electromagnetic
 * braking) for optimal stopping performance. Optimized for 6-wheel drive robots.
 *
 * Mixed-Decay Braking Features:
 * - PHASE 1: Fast decay (coasting) - configurable percentage of brake time
 * - PHASE 2: Slow decay (electromagnetic braking) - remaining brake time
 * - PHASE 3: Position holding with low power electromagnetic braking
 * - Speed-dependent brake timing (200-800ms based on current motor speed)
 * - Configurable fast/slow decay ratio (default: 30% fast, 70% slow)
 * - Turn-aware braking optimization
 * - Directional inversion support
 * - Optimized PWM updates (only when values change)
 */
class Motor {
public:
    Motor(uint8_t fwd_channel, uint8_t rev_channel, bool inverted);

    void setSpeed(int pwm, bool is_turning = false);  // Consolidated with optional turn context
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

    int _current_pwm = 0;
    int _target_pwm = 0;
    int _last_sent_pwm = -1;
    bool _is_turning = false;  // Track if this motor is part of a turning movement

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
