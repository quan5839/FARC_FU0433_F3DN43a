#include "servos.h"
#include "pca9685_driver.h"
#include "../config.h"

/**
 * Helper function to convert a pulse in microseconds (us) to a 12-bit PWM value
 *
 * FREQUENCY CONFIGURATION EXPLANATION:
 * - Servos REQUIRE 50Hz for most reliable operation (standard servo frequency)
 * - Motors work best at ~1600Hz but can tolerate 50Hz with some response loss
 * - PCA9685 can only run at ONE frequency for all channels
 * - PRIORITY: Servo reliability > Motor response time
 * - Using 50Hz ensures servo compatibility and prevents servo damage
 *
 * @param pulse_us Desired pulse width in microseconds (typically 500-2500us for servos)
 * @return 12-bit PWM value (0-4095) for the PCA9685
 */
int pulseUsToPwm(int pulse_us) {
  // Calculate the period of one cycle at 50Hz (servo-optimized frequency)
  // Period (us) = 1,000,000 / Frequency (Hz)
  long cycle_us = config::constants::MICROSECONDS_PER_SECOND / config::constants::PWM_DEFAULT_FREQ; // 50Hz = 20000us period

  // Calculate the 12-bit PWM value that corresponds to the desired pulse width
  // Value = (pulse_us * PWM_MAX) / cycle_us
  return (pulse_us * config::constants::PWM_MAX) / cycle_us;
}

void setServoAngle(uint8_t channel, int angle) {
  // Constrain the angle to the valid range
  angle = constrain(angle, config::constants::SERVO_MIN_ANGLE, config::constants::SERVO_MAX_ANGLE);
  // Map the angle (0-180) to the standard pulse width (e.g., 500-2500 us)
  int pulse_us = map(angle, config::constants::SERVO_MIN_ANGLE, config::constants::SERVO_MAX_ANGLE, config::servo::MIN_PULSE, config::servo::MAX_PULSE);
  // Convert the pulse width to the correct PWM value for the high frequency clock
  int pulse_12bit = pulseUsToPwm(pulse_us);
  hal_pca9685_set_pwm(channel, pulse_12bit);
}

void setServoSpeed(uint8_t channel, int speed) {
  // Constrain the speed to the valid range
  speed = constrain(speed, -100, 100);
  // Map the speed (-100 to 100) to the pulse width for continuous servos.
  int pulse_us = map(speed, -100, 100, config::servo::MIN_PULSE, config::servo::MAX_PULSE);
  // Convert the pulse width to the correct PWM value for the high frequency clock
  int pulse_12bit = pulseUsToPwm(pulse_us);
  hal_pca9685_set_pwm(channel, pulse_12bit);
}
