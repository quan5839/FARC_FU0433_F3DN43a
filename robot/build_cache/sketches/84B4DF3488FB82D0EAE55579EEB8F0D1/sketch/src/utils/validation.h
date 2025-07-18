#line 1 "/Users/admin/Documents/Arduino/robot/src/utils/validation.h"
#ifndef VALIDATION_H
#define VALIDATION_H

#include "../config.h"

/**
 * @brief Input validation utilities to improve robustness and prevent crashes
 * 
 * This class provides static methods for validating inputs and parameters
 * throughout the robot codebase, ensuring safe operation and preventing
 * crashes from invalid data.
 */
class Validation {
public:
    /**
     * @brief Validate servo angle is within acceptable range
     * @param angle Servo angle to validate (0-180 degrees)
     * @return true if angle is valid, false otherwise
     */
    static bool isValidServoAngle(int angle) {
        return (angle >= config::constants::SERVO_MIN_ANGLE && angle <= config::constants::SERVO_MAX_ANGLE);
    }
    
    /**
     * @brief Validate PWM value is within acceptable range
     * @param pwm PWM value to validate
     * @return true if PWM value is valid, false otherwise
     */
    static bool isValidPWM(int pwm) {
        return (pwm >= -config::motor::MAX_PWM && pwm <= config::motor::MAX_PWM);
    }
    
    /**
     * @brief Validate joystick input is within expected range
     * @param joystickValue Joystick input to validate
     * @return true if joystick value is valid, false otherwise
     */
    static bool isValidJoystickInput(int joystickValue) {
        return (joystickValue >= -config::ps2::JOYSTICK_MAX && 
                joystickValue <= config::ps2::JOYSTICK_MAX);
    }
    
    /**
     * @brief Validate servo channel is within VIA banh mi v2023 servo range
     * @param channel Servo channel to validate (2-7 only)
     * @return true if channel is valid, false otherwise
     */
    static bool isValidServoChannel(uint8_t channel) {
        return (channel >= config::channels::SERVO_MIN_CHANNEL &&
                channel <= config::channels::SERVO_MAX_CHANNEL);
    }

    /**
     * @brief Validate motor channel is within VIA banh mi v2023 motor range
     * @param channel Motor channel to validate (8-15 only)
     * @return true if channel is valid, false otherwise
     */
    static bool isValidMotorChannel(uint8_t channel) {
        return (channel >= config::channels::MOTOR_MIN_CHANNEL &&
                channel <= config::channels::MOTOR_MAX_CHANNEL);
    }
    
    /**
     * @brief Validate timeout value is reasonable
     * @param timeoutMs Timeout in milliseconds
     * @return true if timeout is valid, false otherwise
     */
    static bool isValidTimeout(unsigned long timeoutMs) {
        return (timeoutMs >= config::constants::MIN_TIMEOUT_MS && timeoutMs <= config::constants::MAX_TIMEOUT_MS);
    }

    /**
     * @brief Validate percentage value
     * @param percentage Percentage to validate (0-100)
     * @return true if percentage is valid, false otherwise
     */
    static bool isValidPercentage(int percentage) {
        return (percentage >= config::constants::PERCENT_MIN && percentage <= config::constants::PERCENT_MAX);
    }

    /**
     * @brief Clamp servo angle to valid range
     * @param angle Servo angle to clamp
     * @return Clamped servo angle (0-180)
     */
    static int clampServoAngle(int angle) {
        if (angle < config::constants::SERVO_MIN_ANGLE) return config::constants::SERVO_MIN_ANGLE;
        if (angle > config::constants::SERVO_MAX_ANGLE) return config::constants::SERVO_MAX_ANGLE;
        return angle;
    }
    
    /**
     * @brief Clamp PWM value to valid range
     * @param pwm PWM value to clamp
     * @return Clamped PWM value
     */
    static int clampPWM(int pwm) {
        if (pwm > config::motor::MAX_PWM) return config::motor::MAX_PWM;
        if (pwm < -config::motor::MAX_PWM) return -config::motor::MAX_PWM;
        return pwm;
    }
    
    /**
     * @brief Clamp joystick input to valid range
     * @param joystickValue Joystick input to clamp
     * @return Clamped joystick value
     */
    static int clampJoystickInput(int joystickValue) {
        if (joystickValue > config::ps2::JOYSTICK_MAX) return config::ps2::JOYSTICK_MAX;
        if (joystickValue < -config::ps2::JOYSTICK_MAX) return -config::ps2::JOYSTICK_MAX;
        return joystickValue;
    }
    
    /**
     * @brief Clamp percentage to valid range
     * @param percentage Percentage to clamp
     * @return Clamped percentage (0-100)
     */
    static int clampPercentage(int percentage) {
        if (percentage < 0) return 0;
        if (percentage > 100) return 100;
        return percentage;
    }
    
    /**
     * @brief Safe servo angle setting with validation and error reporting
     * @param channel Servo channel
     * @param angle Desired servo angle
     * @return true if successful, false if validation failed
     */
    static bool setServoAngleSafe(uint8_t channel, int angle);
    
    /**
     * @brief Safe motor speed setting with validation and error reporting
     * @param speed Desired motor speed
     * @return Validated and clamped motor speed
     */
    static int setMotorSpeedSafe(int speed);
    
    /**
     * @brief Print validation error message
     * @param paramName Parameter name that failed validation
     * @param value Invalid value
     * @param validRange Description of valid range
     */
    static void printValidationError(const char* paramName, int value, const char* validRange);
};

#endif // VALIDATION_H
