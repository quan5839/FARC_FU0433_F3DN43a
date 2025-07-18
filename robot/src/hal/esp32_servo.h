#ifndef ESP32_SERVO_H
#define ESP32_SERVO_H

#include <Arduino.h>

/**
 * @brief ESP32 LEDC-based servo control for direct GPIO pins
 * 
 * This module provides high-frequency servo control using ESP32's LEDC PWM
 * instead of PCA9685. Useful for servos connected to back GPIO pins while
 * keeping power from PCA9685 channels.
 */
namespace ESP32Servo {
    
    /**
     * @brief Initialize ESP32 servo control on specified GPIO pin
     * @param gpio_pin ESP32 GPIO pin number
     * @param ledc_channel LEDC channel to use (0-15)
     * @return true if initialization successful
     */
    bool init(uint8_t gpio_pin, uint8_t ledc_channel);
    
    /**
     * @brief Set servo angle using ESP32 LEDC PWM
     * @param gpio_pin ESP32 GPIO pin number
     * @param angle Servo angle (0-180 degrees)
     */
    void setAngle(uint8_t gpio_pin, int angle);

    /**
     * @brief Set servo pulse width directly
     * @param gpio_pin ESP32 GPIO pin number
     * @param pulse_us Pulse width in microseconds (500-2500)
     */
    void setPulseWidth(uint8_t gpio_pin, int pulse_us);

    /**
     * @brief Disable servo PWM output (servo will float)
     * @param gpio_pin ESP32 GPIO pin number
     */
    void disable(uint8_t gpio_pin);
}

#endif // ESP32_SERVO_H
