#include "pca9685_driver.h"
#include <Wire.h>
#include <Arduino.h>
#include "../config.h"

// Performance optimization pragmas for I2C communication
#pragma GCC optimize("O3")
#pragma GCC optimize("fast-math")

// Define the Adafruit_PWMServoDriver object as static
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void initPCA9685()
{
  // Proven I2C initialization methods (Arduino documentation)
  Wire.begin(21, 22);  // Explicit SDA/SCL pins for VIA board (ESP32 docs)
  Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);  // Standard I2C speed setting
  Wire.setTimeOut(config::performance::I2C_TIMEOUT_MS);  // Arduino docs: 10-1000ms recommended

  pwm.begin();
  pwm.setOscillatorFrequency(config::pca9685::OSCILLATOR_FREQ);
  pwm.setPWMFreq(config::constants::PWM_DEFAULT_FREQ); // Use servo frequency as default (servo compatibility required)

  DEBUG_PRINT("PCA9685 initialized at ");
  DEBUG_PRINT(config::pca9685::I2C_CLOCK_SPEED / 1000);
  DEBUG_PRINTLN("kHz I2C speed");
}

void hal_pca9685_set_pwm(uint8_t channel, uint16_t value) {
  // Input validation
  if (channel > 15) {
    ERROR_PRINT("ERROR: Invalid PCA9685 channel ");
    ERROR_PRINT(channel);
    ERROR_PRINTLN(" (max 15)");
    return;
  }

  if (value > 4095) {
    DEBUG_PRINT("WARNING: PWM value ");
    DEBUG_PRINT(value);
    DEBUG_PRINTLN(" clamped to 4095");
    value = 4095;
  }

  // Set PWM value - setPWM() doesn't return error codes
  pwm.setPWM(channel, 0, value);

  // Note: Error detection would require periodic health checks
  // rather than per-call checking since setPWM() returns void
}

bool hal_pca9685_is_connected() {
  // Test I2C communication by checking if device responds to configured address
  Wire.beginTransmission(config::pca9685::I2C_ADDRESS);
  uint8_t error = Wire.endTransmission();

  // error == 0 means device acknowledged the address
  // error != 0 means no device responded or communication failed
  return (error == 0);
}

bool hal_pca9685_recover() {
  // Standard Arduino I2C recovery method (documented in Wire library)
  DEBUG_PRINTLN("Attempting standard I2C recovery...");

  if (!config::performance::ENABLE_STANDARD_I2C_RECOVERY) {
    return false;
  }

  // Method 1: Standard Wire library recovery (Arduino documentation)
  Wire.end();
  delay(10); // Arduino docs recommend 10ms minimum for I2C bus reset
  Wire.begin(21, 22);
  Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);
  Wire.setTimeOut(config::performance::I2C_TIMEOUT_MS);

  // Test if recovery worked
  if (hal_pca9685_is_connected()) {
    DEBUG_PRINTLN("Standard I2C recovery successful");
    return true;
  }

  // Method 2: Full PCA9685 reinitialization (Adafruit library docs)
  pwm.begin();
  pwm.setOscillatorFrequency(config::pca9685::OSCILLATOR_FREQ);
  pwm.setPWMFreq(config::constants::PWM_DEFAULT_FREQ);
  delay(50); // PCA9685 datasheet recommends 50ms for full initialization

  if (hal_pca9685_is_connected()) {
    DEBUG_PRINTLN("PCA9685 reinitialization successful");
    return true;
  }

  ERROR_PRINTLN("Standard I2C recovery failed");
  return false;
}

void hal_pca9685_emergency_stop() {
  ERROR_PRINTLN("PCA9685 EMERGENCY STOP - All outputs disabled");

  // Try to set all channels to 0 using direct I2C if possible
  // This is a last-ditch effort to stop all motors/servos
  for (int i = 0; i < 16; i++) {
    pwm.setPWM(i, 0, 0);
  }
}

// Removed experimental proactive monitoring - using only proven methods
