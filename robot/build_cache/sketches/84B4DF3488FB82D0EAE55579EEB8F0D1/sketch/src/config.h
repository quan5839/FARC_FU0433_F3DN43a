#line 1 "/Users/admin/Documents/Arduino/robot/src/config.h"
#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// --- ROBOT CONFIGURATION AND CONSTANTS ---
// This file contains all configuration values, constants, and hardware definitions
namespace config {

  // === UNIVERSAL CONSTANTS ===
  namespace constants {
    // Servo angle limits
    constexpr int SERVO_MIN_ANGLE = 0;
    constexpr int SERVO_MAX_ANGLE = 180;
    constexpr int SERVO_CENTER_ANGLE = 90;

    // PWM values
    constexpr int PWM_STOPPED = 0;
    constexpr int PWM_MIN = -4095;
    constexpr int PWM_MAX = 4095;

    // Joystick values
    constexpr int JOYSTICK_CENTER = 0;
    constexpr int JOYSTICK_MIN = -128;
    constexpr int JOYSTICK_MAX = 128;
    constexpr int JOYSTICK_NEUTRAL = 128; // PS2 controller neutral position

    // Time constants
    constexpr unsigned long ONE_SECOND_MS = 1000;
    constexpr unsigned long HALF_SECOND_MS = 500;
    constexpr unsigned long QUARTER_SECOND_MS = 250;
    constexpr unsigned long ONE_MINUTE_MS = 60000;

    // Communication constants
    constexpr int SERIAL_BAUD_RATE = 115200;
    constexpr int I2C_CLOCK_SPEED = 400000; // 400kHz

    // Digital pin states
    constexpr int DIGITAL_LOW = 0;
    constexpr int DIGITAL_HIGH = 1;

    // Percentage constants
    constexpr int PERCENT_MIN = 0;
    constexpr int PERCENT_MAX = 100;
    constexpr int PERCENT_HALF = 50;

    // Loop timing
    constexpr unsigned long TARGET_LOOP_TIME_MS = 10; // 10ms target loop time

    // Motor control constants
    constexpr int MOTOR_STOP_SPEED = 0;
    constexpr int MOTOR_FULL_FORWARD = PWM_MAX;
    constexpr int MOTOR_FULL_REVERSE = PWM_MIN;

    // Boolean constants for clarity
    constexpr bool FEATURE_ENABLED = true;
    constexpr bool FEATURE_DISABLED = false;
    constexpr bool MOTOR_INVERTED = true;
    constexpr bool MOTOR_NOT_INVERTED = false;

    // Sensor constants
    constexpr int LIMIT_SWITCH_TRIGGERED = 0; // LOW when triggered (INPUT_PULLUP)
    constexpr int LIMIT_SWITCH_NOT_TRIGGERED = 1; // HIGH when not triggered

    // Validation ranges
    constexpr int MIN_TIMEOUT_MS = 100;
    constexpr int MAX_TIMEOUT_MS = 30000; // 30 seconds
    constexpr int MIN_DEADZONE = 0;
    constexpr int MAX_DEADZONE = 50;

    // Safety constants
    constexpr unsigned long TIMEOUT_SAFETY_MARGIN_MS = 1000; // 1 second safety margin
    constexpr int PWM_SAFETY_LIMIT = 4000; // Slightly below max for safety
    constexpr unsigned long WATCHDOG_TIMEOUT_MS = 5000; // 5 second watchdog
    constexpr int MAX_CONSECUTIVE_ERRORS = 5; // Max errors before safety shutdown

    // Watchdog and monitoring
    constexpr bool ENABLE_WATCHDOG = true;               // Enable ESP32 hardware watchdog
    constexpr bool ENABLE_VOLTAGE_MONITORING = false;    // Enable battery voltage monitoring (VIA board has no voltage sensing)
    constexpr bool ENABLE_TEMPERATURE_MONITORING = true; // Enable ESP32 temperature monitoring
    constexpr float MIN_BATTERY_VOLTAGE = 10.5;          // Minimum battery voltage (V)
    constexpr float LOW_BATTERY_WARNING = 11.0;          // Low battery warning voltage (V)
    constexpr float MAX_ESP32_TEMPERATURE = 80.0;        // Maximum ESP32 temperature (°C)

    // Communication timeouts
    constexpr unsigned long I2C_TIMEOUT_MS = 100;
    constexpr unsigned long SERIAL_TIMEOUT_MS = 1000;
    constexpr unsigned long PS2_RETRY_DELAY_MS = 100; // Delay between PS2 connection attempts



    // Configuration constants
    constexpr int CONFIG_VERSION = 1;
    constexpr uint32_t CONFIG_MAGIC_NUMBER = 0xDEADBEEF;
  }

  // === ROBOT CONFIGURATION ===
  // -- Robot States --
  enum RobotState {
    IDLE,
    MANUAL_CONTROL,
    AUTOMATIC_OUTTAKE_REVERSE,
    TIMED_OUTTAKE_FORWARD
  };

  // -- PS2 Controller --
  namespace ps2 {
    constexpr uint8_t DATA_PIN = 12;
    constexpr uint8_t CMD_PIN  = 13;
    constexpr uint8_t SEL_PIN  = 15;
    constexpr uint8_t CLK_PIN  = 14;
    constexpr int LOOP_MS = constants::TARGET_LOOP_TIME_MS;
    constexpr int JOYSTICK_DEADZONE = 1; // Dead zone for joystick inputs (0-128 range)
    constexpr int JOYSTICK_MAX = constants::JOYSTICK_MAX;
  }

  // -- Servos --
  namespace servo {
    constexpr int MIN_PULSE = 500;
    constexpr int MAX_PULSE = 2500;
  }

  // -- PWM FREQUENCY CONFIGURATION --
  namespace pwm {
      constexpr int MOTOR_OPTIMAL_FREQ = 1600; // Optimal frequency for motors (best performance)
      constexpr int SERVO_OPTIMAL_FREQ = 50;   // Standard frequency for servos (REQUIRED for reliability)
      constexpr int COMPROMISE_FREQ = 400;     // Compromise frequency

      // FREQUENCY STRATEGY SELECTION:
      // Servos REQUIRE 50Hz for most reliable operation. Motors can tolerate lower frequencies
      // with some loss in response time, but servos will malfunction at higher frequencies.
      // PRIORITY: Servo reliability > Motor response time
      constexpr int DEFAULT_FREQ = SERVO_OPTIMAL_FREQ; // Use 50Hz for servo reliability
  }

  // -- PCA9685 Driver --
  namespace pca9685 {
    constexpr int OSCILLATOR_FREQ = 27000000;
    constexpr long I2C_CLOCK_SPEED = constants::I2C_CLOCK_SPEED;
  }

  // -- Motors --
  namespace motor {
    constexpr int MIN_PWM = constants::PWM_STOPPED;
    constexpr int MAX_PWM = constants::PWM_MAX;
  }

  // -- VIA "banh mi" v2023 Board Channel Limits --
  namespace channels {
    constexpr uint8_t SERVO_MIN_CHANNEL = 2;   // VIA banh mi v2023: Servos only on channels 2-7
    constexpr uint8_t SERVO_MAX_CHANNEL = 7;   // (6 servo ports available)
    constexpr uint8_t MOTOR_MIN_CHANNEL = 8;   // VIA banh mi v2023: Motors only on channels 8-15
    constexpr uint8_t MOTOR_MAX_CHANNEL = 15;  // VIA banh mi v2023: Motors only on channels 8-15
  }

  // -- Physical Layout --
  // Motor Driver Channels
  constexpr uint8_t CHAN_DRIVE_L_FWD   = 8;
  constexpr uint8_t CHAN_DRIVE_L_REV   = 9;
  constexpr uint8_t CHAN_DRIVE_R_FWD   = 10;
  constexpr uint8_t CHAN_DRIVE_R_REV   = 11;
  constexpr uint8_t CHAN_OUTTAKE_L_FWD = 12;
  constexpr uint8_t CHAN_OUTTAKE_L_REV = 13;
  constexpr uint8_t CHAN_OUTTAKE_R_FWD = 14;
  constexpr uint8_t CHAN_OUTTAKE_R_REV = 15;

  // Servo Channels
  constexpr uint8_t OUTTAKE_SERVO_CHANNEL    = 2;
  constexpr uint8_t INTAKE_SERVO_CHANNEL    = 3;

  // Sensor Pins
  constexpr uint8_t LIMIT_SWITCH_PIN = 32;


  //================================================================================
  // PERFORMANCE TUNING PARAMETERS
  // Tweak these values to optimize robot performance and behavior
  //================================================================================
  namespace tuning {
    // -- Drive Control Performance --
    constexpr int PWM_MAX_PERCENT      = 55; // Maximum speed for normal driving (0-100%)
    constexpr int PWM_PREC_PERCENT     = 20; // Maximum speed for precision mode when R1 pressed (0-100%)
    constexpr int TURN_SPEED_PERCENT   = 60; // Turn sensitivity - higher = more responsive turning (0-100%)

    // -- Drive Control Style --
    constexpr bool USE_CHEESY_DRIVE = true;  // Use Cheesy Drive (maintains speed while turning) vs Differential Drive
    constexpr int CHEESY_DRIVE_NONLINEARITY = 80; // Cheesy drive turning feel (0-100%) - higher = more aggressive turning

    // -- FTC-Style Braking Control (Ramping Removed for Better Performance) --
    // Turn detection sensitivity (used for brake optimization)
    constexpr int TURN_DETECTION_THRESHOLD = 15;     // Minimum speed difference % to consider as turning (0-100%)

    // -- FTC-Style Braking Configuration --
    constexpr bool ENABLE_ACTIVE_BRAKING = true;     // Enable FTC-style active braking
    constexpr bool USE_ELECTROMAGNETIC_BRAKING = true; // Use terminal shorting (true FTC-style) vs reverse current
    constexpr int BRAKE_POWER_PERCENT = 25;          // Electromagnetic braking power (0-100%) - lower for terminal shorting
    constexpr int BRAKE_TIME_MS = 200;               // How long to apply active brake (ms) - longer for electromagnetic
    constexpr int BRAKE_THRESHOLD_PERCENT = 15;      // Minimum speed % to trigger braking (lower threshold for electromagnetic)
    constexpr int BRAKE_HOLD_POWER_PERCENT = 10;     // Continuous holding power after initial brake (0-100%)

    // -- Motor Acceleration/Deceleration Limiting --
    constexpr bool ENABLE_MOTOR_RAMPING = true;      // Enable time-based acceleration/deceleration limiting
    constexpr int ACCELERATION_TIME_MS = 150;        // Time to accelerate from 0 to full throttle (ms)
    constexpr int DECELERATION_TIME_MS = 75;         // Time to decelerate from full throttle to 0 (ms)



    //================================================================================
    // SLEW RATE TUNING GUIDE
    //================================================================================
    /*
     * TUNING PARAMETERS EXPLANATION:
     *
     * ACCEL_TIME_MS (150ms default):
     *   - Time to accelerate from 0 to max speed
     *   - Lower = more responsive, higher = smoother
     *   - Range: 50-500ms
     *   - Effect: How quickly robot responds to forward/reverse input
     *
     * DECEL_TIME_MS (250ms default):
     *   - Time to decelerate from max to 0 speed
     *   - Lower = more abrupt stops, higher = smoother stops
     *   - Range: 100-800ms
     *   - Effect: How smoothly robot comes to a stop
     *
     * TURN_ACCEL_TIME_MS (120ms default):
     *   - Acceleration time during turning movements
     *   - Lower = more agile turns, higher = smoother turns
     *   - Range: 50-300ms
     *   - Effect: How quickly robot responds to turning input
     *
     * TURN_DECEL_TIME_MS (180ms default):
     *   - Deceleration time during turning movements
     *   - Lower = sharper turn stops, higher = smoother turn transitions
     *   - Range: 80-400ms
     *   - Effect: How smoothly robot transitions out of turns
     *
     * LOW_SPEED_RAMP_MULTIPLIER (150% default):
     *   - Ramping speed multiplier at low speeds
     *   - Higher = faster ramping at low speeds for better control
     *   - Range: 100-300%
     *   - Effect: Responsiveness when starting from stop
     *
     * HIGH_SPEED_RAMP_MULTIPLIER (75% default):
     *   - Ramping speed multiplier at high speeds
     *   - Lower = slower ramping at high speeds for stability
     *   - Range: 50-150%
     *   - Effect: Stability and smoothness at high speeds
     *
     * TURN_DETECTION_THRESHOLD (15% default):
     *   - Minimum speed difference between motors to detect turning
     *   - Lower = more sensitive turn detection, higher = less sensitive
     *   - Range: 5-50%
     *   - Effect: When adaptive turn ramping activates
     *
     * TUNING RECOMMENDATIONS:
     * 1. Start with default values
     * 2. Adjust ACCEL_TIME_MS for responsiveness vs smoothness
     * 3. Adjust TURN_ACCEL_TIME_MS for turning agility
     * 4. Fine-tune speed multipliers for different speed zones
     * 5. Test with different loads and surface conditions
     */

    // -- Outtake --
    constexpr int HOLD_POWER_PERCENT = 0;  // Holding power for outtake motors (0-100%)
    constexpr int OUTTAKE_TIMEOUT_MS = 3000; // Timeout for the automatic outtake sequence (ms)
    constexpr int OUTTAKE_FORWARD_TIMEOUT_MS = 3250; // Timeout for the timed forward outtake (ms)

    // -- Servo Angles --
    constexpr int INTAKE_ARM_CLOSE_ANGLE  = 135;
    constexpr int INTAKE_ARM_OPEN_ANGLE   = 45;
    constexpr int OUTTAKE_ARM_OPEN_ANGLE  = 145;
    constexpr int OUTTAKE_ARM_CLOSE_ANGLE = 45;

    // -- Drive Inversion --
    inline bool invertDriveLeft    = false;
    inline bool invertDriveRight   = true;
    inline bool invertOuttakeRight = true;
    inline bool limitSwitchDisabled = false;
    constexpr unsigned long DEBOUNCE_DELAY_MS = 50; // Debounce delay for limit switch
    constexpr int CONNECTION_TIMEOUT_MS = 1000;  // Time before robot goes IDLE: 1 second
    constexpr int RECONNECTION_ATTEMPT_INTERVAL_MS = 10;  // Reconnection attempts every 10ms

    // Enhanced PS2 controller recovery
    constexpr int PS2_MAX_RETRY_DELAY_MS = 5000;         // Maximum retry delay (5 seconds)
    constexpr int PS2_RETRY_BACKOFF_MULTIPLIER = 2;      // Exponential backoff multiplier
    constexpr int PS2_MAX_CONSECUTIVE_FAILURES = 10;     // Max failures before giving up temporarily

  }
}

#endif // CONFIG_H
