#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>
#include <cstddef>  // For size_t

//================================================================================
// ROBOT CONFIGURATION - VIA "banh mi" v2023 Board (ESP32 + PCA9685)
//================================================================================
namespace config {

  // === CORE CONSTANTS ===
  namespace constants {
    // PWM Range (PCA9685 12-bit resolution)
    constexpr int PWM_MAX = 4095;                    // Maximum PWM value (100% duty cycle)
    constexpr int PWM_DEFAULT_FREQ = 50;             // Unified frequency (servo-limited)

    // Mathematical constants
    constexpr int PERCENT_TO_DECIMAL_DIVISOR = 100;
    constexpr int QUARTER_DIVISOR = 4;
    constexpr int KILOHERTZ_DIVISOR = 1000;
    constexpr long MICROSECONDS_PER_SECOND = 1000000;
    constexpr int MAX_CONSECUTIVE_ERRORS = 5;

    // Servo constants
    constexpr int SERVO_MIN_ANGLE = 0;
    constexpr int SERVO_MAX_ANGLE = 180;

    // Communication
    constexpr int SERIAL_BAUD_RATE = 115200;
    constexpr unsigned long PS2_RETRY_DELAY_MS = 25;
    constexpr unsigned long SERIAL_INIT_DELAY_MS = 1000;

    // Motor control
    constexpr bool MOTOR_NOT_INVERTED = false;
    constexpr bool MOTOR_INVERTED = true;

    // Sensor constants
    constexpr int LIMIT_SWITCH_TRIGGERED = 0;        // LOW when triggered (INPUT_PULLUP)
    constexpr int LIMIT_SWITCH_NOT_TRIGGERED = 1;    // HIGH when not triggered (INPUT_PULLUP)

    // Safety constants
    constexpr unsigned long TIMEOUT_SAFETY_MARGIN_MS = 1000;
    constexpr bool ENABLE_TEMPERATURE_MONITORING = true;
    constexpr float MAX_ESP32_TEMPERATURE = 80.0;
    constexpr bool ENABLE_WATCHDOG = true;
    constexpr unsigned long WATCHDOG_FEED_INTERVAL_MS = 1000;
    constexpr unsigned long WATCHDOG_TIMEOUT_MS = 5000;
    constexpr unsigned long SAFETY_CHECK_INTERVAL_MS = 5000;
  }

// ===== COMPETITION MODE TOGGLE =====
// Set COMPETITION_MODE to 1 to disable all debug output for maximum performance
#define COMPETITION_MODE 0

// Performance-optimized debug macros
#if COMPETITION_MODE
  #define ENABLE_DEBUG_OUTPUT 0
  #define ENABLE_ERROR_OUTPUT 0
#else
  #define ENABLE_DEBUG_OUTPUT 1
  #define ENABLE_ERROR_OUTPUT 1
#endif

// Debug and error output macros - optimized for zero overhead when disabled
#if ENABLE_DEBUG_OUTPUT
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x) ((void)0)
  #define DEBUG_PRINTLN(x) ((void)0)
#endif

#if ENABLE_ERROR_OUTPUT
  #define ERROR_PRINT(x) Serial.print(x)
  #define ERROR_PRINTLN(x) Serial.println(x)
#else
  #define ERROR_PRINT(x) ((void)0)
  #define ERROR_PRINTLN(x) ((void)0)
#endif

  // === ROBOT CONFIGURATION ===
  // -- Robot States --
  enum RobotState {
    IDLE,
    MANUAL_CONTROL,
    AUTOMATIC_OUTTAKE_REVERSE,
    TIMED_OUTTAKE_FORWARD
  };

  // -- PS2 Controller --
  // Using ESP32 HSPI pins for PS2 controller with PsxNewLib hardware SPI
  // HSPI pins: MISO=12, MOSI=13, SCK=14, CS=15
  // PsxNewLib automatically uses MISO/MOSI/SCK, we only specify ATT (CS) pin
  namespace ps2 {
    constexpr uint8_t DATA_PIN = 12;  // MISO (Master In Slave Out) - PS2 Data
    constexpr uint8_t CMD_PIN  = 13;  // MOSI (Master Out Slave In) - PS2 Command
    constexpr uint8_t CLK_PIN  = 14;  // SCK (Serial Clock) - PS2 Clock
    constexpr uint8_t ATT_PIN  = 15;  // CS (Chip Select) - PS2 Attention/Select
    // Legacy pin names for compatibility (not used by PsxNewLib)
    constexpr uint8_t SEL_PIN  = ATT_PIN;  // Alias for ATT_PIN
    constexpr int JOYSTICK_DEADZONE = 1;   // Dead zone for joystick inputs
    constexpr int JOYSTICK_MAX = 128;      // Maximum joystick value (PS2 controller range)
  }



  // -- Servos --
  namespace servo {
    constexpr int MIN_PULSE = 500;
    constexpr int MAX_PULSE = 2500;

  }

  // Note: PWM frequency constants moved to constants namespace to eliminate duplication

  // -- PCA9685 Driver --
  namespace pca9685 {
    constexpr int OSCILLATOR_FREQ = 27000000;               // PCA9685 internal oscillator frequency (27MHz)
    constexpr long I2C_CLOCK_SPEED = 1000000;
    constexpr uint8_t I2C_ADDRESS = 0x40;                   // PCA9685 default I2C address (7-bit)
  }

  // -- MPU6050 IMU --
  namespace imu {
    constexpr uint8_t I2C_ADDRESS = 0x68;                   // MPU6050 default I2C address
    constexpr long I2C_CLOCK_SPEED = 400000;                // MPU6050 I2C speed (400kHz - most reliable)
    constexpr bool ENABLE_IMU = false;                       // Enable IMU functionality
    constexpr unsigned long UPDATE_INTERVAL_MS = 10;       // Update IMU every 10ms (100Hz)

    // Movement detection thresholds
    constexpr float MOVEMENT_THRESHOLD_G = 0.2;             // Movement detection threshold (G-force)
    constexpr float TURNING_THRESHOLD_DPS = 30.0;           // Turning detection threshold (degrees/second)
  }

  // Note: Motor PWM constants moved to constants namespace to eliminate duplication

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
  constexpr uint8_t OUTTAKE_SERVO_CHANNEL    = 2;  // PCA9685 channel for outtake servo
  constexpr uint8_t INTAKE_SERVO_CHANNEL     = 3;  // PCA9685 channel for intake servo

  // Sensor Pins
  constexpr uint8_t LIMIT_SWITCH_PIN = 32;

  // WS2812B LED Strip Control
  constexpr uint8_t WS2812B_DATA_PIN = 25;         // GPIO 25 for WS2812B data signal
  constexpr uint8_t WS2812B_POWER_CHANNEL = 7;     // PCA9685 channel 7 for power switching
  constexpr int WS2812B_NUM_LEDS = 120;            // Number of LEDs in the strip




  //================================================================================
  // ESP32 PERFORMANCE OPTIMIZATION
  // Maximum performance settings for competition robotics
  //================================================================================
  namespace performance {
    // -- Loop Timing --
    constexpr unsigned long TARGET_LOOP_TIME_US = 1000; // Target 1ms loop time (1000Hz)
    constexpr int ESP32_MAX_CPU_FREQ_MHZ = 240;          // Maximum ESP32 CPU frequency

    // -- Performance Constants --
    constexpr float PWM_TO_PERCENT_MULTIPLIER = 100.0f / constants::PWM_MAX;

    // -- Fast Math Macros --
    #define FAST_ABS(x) ((x) < 0 ? -(x) : (x))
    #define LIKELY(x) __builtin_expect(!!(x), 1)
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)

    // -- I2C Health Monitoring --
    constexpr unsigned long I2C_HEALTH_CHECK_INTERVAL_MS = 10000;
    constexpr unsigned long I2C_RESPONSE_TIMEOUT_US = 1000;
    constexpr float I2C_MAX_ERROR_RATE = 0.1;
    constexpr unsigned int I2C_TIMEOUT_MS = 100;
    constexpr bool ENABLE_STANDARD_I2C_RECOVERY = true;
  }

  //================================================================================
  // PERFORMANCE TUNING PARAMETERS
  // Tweak these values to optimize robot performance and behavior
  //================================================================================
  namespace tuning {
    // -- Drive Control Performance --
    constexpr int DRIVE_MAX_SPEED_PERCENT    = 100; // Maximum speed for normal driving (0-100%)
    constexpr int DRIVE_PRECISION_SPEED_PERCENT = 17; // Maximum speed for precision mode when R1 pressed (0-100%)
    constexpr int TURN_SENSITIVITY_PERCENT   = 35; // Turn sensitivity - higher = more responsive turning (0-100%)

    // Pre-calculated PWM values for performance optimization
    constexpr int DRIVE_MAX_SPEED_PWM = (constants::PWM_MAX * DRIVE_MAX_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;        // 2252 PWM units
    constexpr int DRIVE_PRECISION_PWM = (constants::PWM_MAX * DRIVE_PRECISION_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;  // 819 PWM units

    // -- Outtake Motor Settings (Independent from Drive) --
    constexpr int OUTTAKE_MAX_SPEED_PERCENT = 100;  // Outtake motors always run at full power (0-100%)
    constexpr int OUTTAKE_PWM = (constants::PWM_MAX * OUTTAKE_MAX_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;              // 4095 PWM units (full power)

    // -- Drive Control Style --
    constexpr int CHEESY_DRIVE_NONLINEARITY = 25; // Cheesy drive turning feel (0-100%)

    // -- Motor Braking Configuration --
    constexpr bool ENABLE_ACTIVE_BRAKING = true;
    constexpr int BRAKE_POWER_PERCENT = 100;
    constexpr int BRAKE_THRESHOLD_PERCENT = 5;
    constexpr int TURN_DETECTION_THRESHOLD = 5;
    constexpr int BRAKE_BASE_TIME_MS = 50;
    constexpr int BRAKE_MAX_TIME_MS = 300;
    constexpr int BRAKE_HOLD_POWER_PERCENT = 100;
    constexpr int FAST_DECAY_PERCENT = 30;
    constexpr int SLOW_DECAY_PERCENT = 70;

    // -- Motor Ramping --
    constexpr bool ENABLE_MOTOR_RAMPING = true;
    constexpr int ACCELERATION_TIME_MS = 300;
    constexpr int DECELERATION_TIME_MS = 200;




    // -- Timeouts --
    constexpr int OUTTAKE_TIMEOUT_MS = 1500;
    constexpr int OUTTAKE_FORWARD_TIMEOUT_MS = 1500;

    // -- Servo Angles --
    constexpr int INTAKE_ARM_CLOSE_ANGLE = 135;
    constexpr int INTAKE_ARM_OPEN_ANGLE = 45;
    constexpr int OUTTAKE_ARM_OPEN_ANGLE = 145;
    constexpr int OUTTAKE_ARM_CLOSE_ANGLE = 45;

    // -- Runtime Settings --
    inline bool INVERT_DRIVE_LEFT = false;
    inline bool INVERT_DRIVE_RIGHT = true;
    inline bool INVERT_OUTTAKE_RIGHT = true;
    inline bool LIMIT_SWITCH_DISABLED = false;
    constexpr unsigned long DEBOUNCE_DELAY_MS = 50;

    // -- PS2 Controller --
    constexpr int CONNECTION_TIMEOUT_MS = 1000;
    constexpr int RECONNECTION_ATTEMPT_INTERVAL_MS = 1000;

    // -- Repetitive Input Safety --
    constexpr unsigned long REPETITIVE_INPUT_TIMEOUT_MS = 5000;  // 5 seconds of same input triggers shutdown
    constexpr int REPETITIVE_INPUT_THRESHOLD = 5;               // Minimum change required to reset timer (PWM units)

    // -- WS2812B LED Strip Settings --
    constexpr uint8_t WS2812B_BRIGHTNESS = 128;                 // Global brightness (0-255, 50% for safety)
    constexpr unsigned long WS2812B_UPDATE_INTERVAL_MS = 50;    // Update interval (20 FPS)
    constexpr uint8_t WS2812B_DEFAULT_HUE = 120;                // Default hue (green)
    constexpr uint8_t WS2812B_DEFAULT_SAT = 255;                // Default saturation (full)
    constexpr uint8_t WS2812B_DEFAULT_VAL = 128;                // Default value/brightness
    constexpr int WS2812B_POWER_ON_PWM = constants::PWM_MAX;    // Full PWM to enable power via PCA9685
    constexpr int WS2812B_POWER_OFF_PWM = 0;                    // Zero PWM to disable power via PCA9685




  }
}

#endif // CONFIG_H
