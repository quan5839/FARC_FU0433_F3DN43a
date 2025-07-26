#ifndef PWM_UTILS_H
#define PWM_UTILS_H

#include "../config.h"
#include <cstdlib>  // For abs() function

/**
 * @brief PWM utility class to centralize PWM calculations and eliminate code duplication
 * 
 * This class provides static methods for common PWM calculations used throughout
 * the robot codebase, ensuring consistency and reducing duplication.
 */
class PWMUtils {
public:
    /**
     * @brief Get maximum PWM value for normal driving (pre-calculated for performance)
     * @return PWM value (0-4095) for maximum normal speed
     */
    static constexpr int getMaxPWM() {
        return config::tuning::DRIVE_MAX_SPEED_PWM;
    }

    /**
     * @brief Get precision PWM value for precise driving (pre-calculated for performance)
     * @return PWM value (0-4095) for precision mode speed
     */
    static constexpr int getPrecisionPWM() {
        return config::tuning::DRIVE_PRECISION_PWM;
    }

    /**
     * @brief Get PWM value for outtake motors (pre-calculated for performance)
     * @return PWM value (0-4095) for outtake motor speed
     */
    static constexpr int getOuttakePWM() {
        return config::tuning::OUTTAKE_PWM;
    }
    
    /**
     * @brief Convert percentage to PWM value
     * @param percentage Percentage (0-100)
     * @return PWM value (0-4095)
     */
    static constexpr int percentageToPWM(int percentage) {
        return (config::constants::PWM_MAX * percentage) / config::constants::PERCENT_TO_DECIMAL_DIVISOR;
    }

    /**
     * @brief Validate PWM value is within acceptable range
     * @param pwm PWM value to validate
     * @return true if PWM value is valid, false otherwise
     */
    static bool isValidPWM(int pwm) {
        return (pwm >= -config::constants::PWM_MAX && pwm <= config::constants::PWM_MAX);
    }
    
    /**
     * @brief Clamp PWM value to valid range
     * @param pwm PWM value to clamp
     * @return Clamped PWM value within valid range
     */
    static int clampPWM(int pwm) {
        if (pwm > config::constants::PWM_MAX) return config::constants::PWM_MAX;
        if (pwm < -config::constants::PWM_MAX) return -config::constants::PWM_MAX;
        return pwm;
    }
    
    /**
     * @brief Scale joystick input to PWM value with bounds checking
     * @param joystickValue Joystick input (-128 to +128)
     * @param maxPWM Maximum PWM value to scale to
     * @return Scaled PWM value
     */
    static int scaleJoystickToPWM(int joystickValue, int maxPWM) {
        if (joystickValue == 0 || maxPWM <= 0) return 0;
        
        // Scale joystick input to PWM range
        int scaledPWM = (joystickValue * maxPWM) / config::ps2::JOYSTICK_MAX;
        
        // Clamp to valid range
        return clampPWM(scaledPWM);
    }
    
    /**
     * @brief Apply deadzone to joystick input
     * @param value Joystick input value
     * @param deadzone Deadzone threshold
     * @return Processed joystick value with deadzone applied
     */
    static int applyDeadzone(int value, int deadzone) {
        if (abs(value) < deadzone) {
            return 0;
        }

        // Scale the remaining range to maintain full output range
        if (value > 0) {
            return map(value, deadzone, config::ps2::JOYSTICK_MAX, 0, config::ps2::JOYSTICK_MAX);
        } else {
            return map(value, -config::ps2::JOYSTICK_MAX, -deadzone, -config::ps2::JOYSTICK_MAX, 0);
        }
    }

    /**
     * @brief Calculate motor speeds using Cheesy Drive algorithm
     * @param throttle Forward/backward input (-128 to +128)
     * @param wheel Turning input (-128 to +128)
     * @param leftPWM Output left motor PWM
     * @param rightPWM Output right motor PWM
     * @param maxPWM Maximum PWM value to use
     */
    static void calculateCheesyDrive(int throttle, int wheel, int& leftPWM, int& rightPWM, int maxPWM) {
        // Normalize inputs to -1.0 to +1.0 range
        float throttleNorm = (float)throttle / config::ps2::JOYSTICK_MAX;
        float wheelNorm = (float)wheel / config::ps2::JOYSTICK_MAX;

        // Apply nonlinearity to wheel for better feel
        float nonlinearity = config::tuning::CHEESY_DRIVE_NONLINEARITY / 100.0f;
        float wheelNonlinear = wheelNorm * wheelNorm * wheelNorm * nonlinearity + wheelNorm * (1.0f - nonlinearity);

        // Apply turn sensitivity and cap the maximum turn rate
        float turnSensitivity = config::tuning::TURN_SENSITIVITY_PERCENT / 100.0f;
        float maxTurnRate = turnSensitivity; // Cap turn component at the configured sensitivity level
        float steeringComponent = wheelNonlinear * turnSensitivity;

        // Clamp steering component to prevent excessive turning
        if (steeringComponent > maxTurnRate) steeringComponent = maxTurnRate;
        if (steeringComponent < -maxTurnRate) steeringComponent = -maxTurnRate;

        // Cheesy Drive: maintain forward/backward speed, add/subtract steering
        float leftMotor = throttleNorm + steeringComponent;
        float rightMotor = throttleNorm - steeringComponent;

        // Convert back to PWM values and clamp
        leftPWM = clampPWM((int)(leftMotor * maxPWM));
        rightPWM = clampPWM((int)(rightMotor * maxPWM));
    }
    /**
     * @brief Apply low-pass filter to reduce joystick jitter
     * @param newValue New input value
     * @param oldValue Previous filtered value
     * @param filterStrength Filter strength (0.0-1.0, higher = more filtering)
     * @return Filtered value
     */
    static int applyLowPassFilter(int newValue, int oldValue, float filterStrength) {
        if (filterStrength <= 0.0f) return newValue; // No filtering
        if (filterStrength >= 1.0f) return oldValue;  // Complete filtering (no change)

        return (int)(oldValue * filterStrength + newValue * (1.0f - filterStrength));
    }

    /**
     * @brief Apply input validation and safety limits
     * @param value Input value to validate
     * @param minValue Minimum allowed value
     * @param maxValue Maximum allowed value
     * @return Validated and clamped value
     */
    static int validateInput(int value, int minValue, int maxValue) {
        if (value < minValue) return minValue;
        if (value > maxValue) return maxValue;
        return value;
    }

private:
    // Utility function for mapping values (Arduino-style map function)
    static long map(long x, long in_min, long in_max, long out_min, long out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
};

#endif // PWM_UTILS_H
