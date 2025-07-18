#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include <Adafruit_PWMServoDriver.h>

// Function to initialize the PCA9685 board
void initPCA9685();
void hal_pca9685_set_pwm(uint8_t channel, uint16_t value);

// Enhanced error handling functions (proven methods only)
bool hal_pca9685_is_connected();
bool hal_pca9685_recover();
void hal_pca9685_emergency_stop();

#endif // PCA9685_DRIVER_H
