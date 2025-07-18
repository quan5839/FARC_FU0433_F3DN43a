#line 1 "/Users/admin/Documents/Arduino/robot/src/hal/pca9685_driver.h"
#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#include <Adafruit_PWMServoDriver.h>

// Function to initialize the PCA9685 board
void initPCA9685();
void hal_pca9685_set_pwm(uint8_t channel, uint16_t value);

// Enhanced error handling functions
bool hal_pca9685_is_connected();
bool hal_pca9685_recover();
void hal_pca9685_emergency_stop();

#endif // PCA9685_DRIVER_H
