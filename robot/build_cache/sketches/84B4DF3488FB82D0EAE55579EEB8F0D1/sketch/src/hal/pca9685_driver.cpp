#line 1 "/Users/admin/Documents/Arduino/robot/src/hal/pca9685_driver.cpp"
#include "pca9685_driver.h"
#include <Wire.h>
#include <Arduino.h>
#include "../config.h"

// Define the Adafruit_PWMServoDriver object as static
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void initPCA9685()
{
  Wire.begin();
  pwm.begin();
  pwm.setOscillatorFrequency(config::pca9685::OSCILLATOR_FREQ);
  pwm.setPWMFreq(config::pwm::DEFAULT_FREQ); // Use motor frequency as default (most channels are motors)
  Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);
}

void hal_pca9685_set_pwm(uint8_t channel, uint16_t value) {
  // Try to set PWM with error checking
  uint8_t result = pwm.setPWM(channel, 0, value);

  // If communication failed, attempt recovery
  if (result != 0) {
    Serial.print("PCA9685 communication error on channel ");
    Serial.print(channel);
    Serial.println(", attempting recovery...");

    if (hal_pca9685_recover()) {
      // Retry the operation after recovery
      pwm.setPWM(channel, 0, value);
    } else {
      Serial.println("PCA9685 recovery failed - entering emergency stop");
      hal_pca9685_emergency_stop();
    }
  }
}

bool hal_pca9685_is_connected() {
  // Try to set and read back a PWM value to test communication
  // Use channel 0 with a test value
  uint8_t result = pwm.setPWM(0, 0, 0);

  // If setPWM returns 0, communication is working
  return (result == 0);
}

bool hal_pca9685_recover() {
  Serial.println("Attempting PCA9685 recovery...");

  // Reinitialize I2C and PCA9685
  Wire.end();
  delay(100);

  Wire.begin();
  Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);

  // Reinitialize PCA9685
  pwm.begin();
  pwm.setOscillatorFrequency(config::pca9685::OSCILLATOR_FREQ);
  pwm.setPWMFreq(config::pwm::DEFAULT_FREQ);

  delay(50); // Allow time for initialization

  // Test if recovery was successful
  if (hal_pca9685_is_connected()) {
    Serial.println("PCA9685 recovery successful");
    return true;
  } else {
    Serial.println("PCA9685 recovery failed");
    return false;
  }
}

void hal_pca9685_emergency_stop() {
  Serial.println("PCA9685 EMERGENCY STOP - All outputs disabled");

  // Try to set all channels to 0 using direct I2C if possible
  // This is a last-ditch effort to stop all motors/servos
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 0);
  }
}
