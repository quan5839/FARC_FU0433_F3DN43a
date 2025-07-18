#ifndef SERVOS_H
#define SERVOS_H

#include <Arduino.h>

// Function Declarations

/**
 * @brief Sets the angle of a 180-degree servo.
 * @param channel The PCA9685 channel the servo is connected to (e.g., 2-7).
 * @param angle The desired angle, from 0 to 180 degrees.
 */
void setServoAngle(uint8_t channel, int angle);

/**
 * @brief Sets the speed of a continuous rotation servo.
 * @param channel The PCA9685 channel the servo is connected to (e.g., 2-7).
 * @param speed The desired speed, from -100 (full reverse) to 100 (full forward).
 */
void setServoSpeed(uint8_t channel, int speed);

#endif // SERVOS_H
