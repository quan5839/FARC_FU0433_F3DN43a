#include "esp32_servo.h"
#include "../config.h"

namespace ESP32Servo {

    bool init(uint8_t gpio_pin, uint8_t ledc_channel) {
        // Configure LEDC timer and channel
        if (ledcAttach(gpio_pin, config::servo::LEDC_FREQUENCY, config::servo::LEDC_RESOLUTION) == 0) {
            return false; // Failed to setup LEDC
        }

        // Set initial position to center (90 degrees) - test position
        setAngle(gpio_pin, 90);

        return true;
    }
    
    void setAngle(uint8_t gpio_pin, int angle) {
        // Clamp angle to valid range
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;

        // Convert angle to pulse width (500-2500 microseconds)
        int pulse_us = config::servo::MIN_PULSE +
                      ((angle * (config::servo::MAX_PULSE - config::servo::MIN_PULSE)) / 180);

        setPulseWidth(gpio_pin, pulse_us);
    }

    void setPulseWidth(uint8_t gpio_pin, int pulse_us) {
        // Clamp pulse width to valid range
        if (pulse_us < config::servo::MIN_PULSE) pulse_us = config::servo::MIN_PULSE;
        if (pulse_us > config::servo::MAX_PULSE) pulse_us = config::servo::MAX_PULSE;

        // Convert microseconds to LEDC duty cycle
        // Formula: duty = (pulse_us * max_duty) / period_us
        // Period = 1/50Hz = 20000us, max_duty = 65535 (16-bit)
        uint32_t duty = (pulse_us * 65535UL) / 20000UL;

        // Clamp to valid duty cycle range
        if (duty < config::servo::LEDC_MIN_DUTY) duty = config::servo::LEDC_MIN_DUTY;
        if (duty > config::servo::LEDC_MAX_DUTY) duty = config::servo::LEDC_MAX_DUTY;

        // Set LEDC duty cycle using GPIO pin
        ledcWrite(gpio_pin, duty);
    }

    void disable(uint8_t gpio_pin) {
        // Set duty cycle to 0 (no PWM output)
        ledcWrite(gpio_pin, 0);
    }
}
