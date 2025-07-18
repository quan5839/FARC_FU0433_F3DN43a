#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>
#include <cstddef>  // For size_t

//================================================================================
// ROBOT CONFIGURATION - VIA "banh mi" v2023 Board (ESP32 + PCA9685)
// Competition-optimized configuration with performance macros
//================================================================================
namespace config {

  // === UNIVERSAL CONSTANTS ===
  namespace constants {
    // PWM Range Constants (PCA9685 12-bit resolution)
    constexpr int PWM_MIN = 0;                       // Minimum PWM value (0% duty cycle)
    constexpr int PWM_MAX = 4095;                    // Maximum PWM value (100% duty cycle, 12-bit)
    constexpr int PWM_STOPPED = 0;                   // Motor stopped state (alias for PWM_MIN)

    // PWM Frequency Constants (consolidated from pwm namespace)
    constexpr int PCA9685_MAX_FREQ = 1526;           // PCA9685 hardware maximum frequency limit
    constexpr int SERVO_REQUIRED_FREQ = 50;         // Required frequency for servo control (20ms period)
    constexpr int PWM_DEFAULT_FREQ = 20; // Current unified frequency (servo-limited)

    // Note: Removed unused PERCENT_MIN/MAX constants (validation classes were removed)

    // Note: PS2 joystick constants removed - using literal values (128) directly in code

    // Communication constants
    constexpr int SERIAL_BAUD_RATE = 115200;
    constexpr int I2C_CLOCK_SPEED = 400000; // 400kHz Fast Mode (safe without external pull-ups)

    // Motor control constants
    constexpr bool MOTOR_NOT_INVERTED = false;
    constexpr bool MOTOR_INVERTED = true;

    // Mathematical constants
    constexpr int PERCENT_TO_DECIMAL_DIVISOR = 100;
    constexpr int HALF_DIVISOR = 2;
    constexpr int QUARTER_DIVISOR = 4;
    constexpr int MAX_CONSECUTIVE_ERRORS = 5;

    // Memory optimization constants
    constexpr size_t CONTROLLER_STATE_POOL_SIZE = 2;     // Pre-allocate controller state objects
    constexpr size_t PWM_CALCULATION_POOL_SIZE = 4;      // Pre-allocate PWM calculation buffers
    

    // Sensor constants
    constexpr int LIMIT_SWITCH_TRIGGERED = 0; // LOW when triggered (INPUT_PULLUP)

    // Safety constants
    constexpr unsigned long TIMEOUT_SAFETY_MARGIN_MS = 1000; // 1 second safety margin
    constexpr unsigned long WATCHDOG_TIMEOUT_MS = 5000; // 5 second watchdog


    // Simplified safety monitoring (optimized for VIA board capabilities)
    constexpr bool ENABLE_WATCHDOG = true;               // ESP32 hardware watchdog
    constexpr bool ENABLE_TEMPERATURE_MONITORING = true; // ESP32 internal temperature sensor
    // Note: Voltage monitoring removed (VIA board has no voltage sensors)
    constexpr float MAX_ESP32_TEMPERATURE = 80.0;        // Maximum ESP32 temperature (°C)

    // Communication timeouts
    constexpr unsigned long PS2_RETRY_DELAY_MS = 25;  // Optimized: Faster PS2 recovery for competition

    // Safety monitoring intervals
    constexpr unsigned long WATCHDOG_FEED_INTERVAL_MS = 1000;  // Feed watchdog every 1 second
    constexpr unsigned long SAFETY_CHECK_INTERVAL_MS = 5000;   // Safety checks every 5 seconds
  }

// ===== COMPETITION MODE TOGGLE =====
// Set COMPETITION_MODE to 1 to disable all debug output for maximum performance
#define COMPETITION_MODE 0  // Enable debug for servo pin testing

// Performance-optimized debug macros
#if COMPETITION_MODE
  #define ENABLE_DEBUG_OUTPUT 0
  #define ENABLE_ERROR_OUTPUT 0
#else
  #define ENABLE_DEBUG_OUTPUT 1
  #define ENABLE_ERROR_OUTPUT 1
#endif

// Debug and error output macros - optimized for zero overhead when disabled
#if ENABLE_DEBUG_OUTPUT && !defined(COMPETITION_MODE)
  #define DEBUG_PRINT(x) do { if (config::performance::MINIMIZE_SERIAL_OUTPUT) break; Serial.print(x); } while(0)
  #define DEBUG_PRINTLN(x) do { if (config::performance::MINIMIZE_SERIAL_OUTPUT) break; Serial.println(x); } while(0)
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
  namespace ps2 {
    constexpr uint8_t DATA_PIN = 12;
    constexpr uint8_t CMD_PIN  = 13;
    constexpr uint8_t CLK_PIN  = 14;
    constexpr uint8_t SEL_PIN  = 15;
    constexpr int JOYSTICK_DEADZONE = 1; // Dead zone for joystick inputs (0-128 range)
    constexpr int JOYSTICK_MAX = 128; // Maximum joystick value after processing
  }

  // -- Servos --
  namespace servo {
    constexpr int MIN_PULSE = 500;
    constexpr int MAX_PULSE = 2500;

    // ESP32 LEDC servo control
    constexpr int LEDC_FREQUENCY = 50;        // 50Hz for servo compatibility
    constexpr int LEDC_RESOLUTION = 16;       // 16-bit resolution (0-65535)
    constexpr int LEDC_CHANNEL = 0;           // LEDC channel for outtake servo
    constexpr int LEDC_MIN_DUTY = 1638;       // ~500us pulse (1638/65535 * 20ms)
    constexpr int LEDC_MAX_DUTY = 8192;       // ~2500us pulse (8192/65535 * 20ms)
  }

  // Note: PWM frequency constants moved to constants namespace to eliminate duplication

  // -- PCA9685 Driver --
  namespace pca9685 {
    constexpr int OSCILLATOR_FREQ = 27000000;               // PCA9685 internal oscillator frequency (27MHz)
    constexpr long I2C_CLOCK_SPEED = constants::I2C_CLOCK_SPEED;
    constexpr uint8_t I2C_ADDRESS = 0x40;                   // PCA9685 default I2C address (7-bit)

    // Hardware-specific constants
    constexpr int OSCILLATOR_FREQ_MHZ = 27;                 // Oscillator frequency in MHz for readability
    constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x40;           // Default factory I2C address
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
  constexpr uint8_t OUTTAKE_SERVO_CHANNEL    = 2;  // PCA9685 channel (power only)
  constexpr uint8_t INTAKE_SERVO_CHANNEL    = 3;   // PCA9685 channel

  // ESP32 Direct Servo Control (front accessible pins)
  constexpr uint8_t OUTTAKE_SERVO_GPIO_PIN   = 25; // IO25 front pin (signal only)

  // Sensor Pins
  constexpr uint8_t LIMIT_SWITCH_PIN = 32;

  // Status LED Pin
  constexpr uint8_t RGB_LED_PIN = 27;  // VIA "banh mi" v2023 RGB LED


  //================================================================================
  // ESP32 PERFORMANCE OPTIMIZATION
  // Maximum performance settings for competition robotics
  //================================================================================
  namespace performance {
    // -- CPU Performance --
    constexpr bool ENABLE_MAXIMUM_PERFORMANCE = true;   // Enable all performance optimizations
    constexpr bool USE_DUAL_CORE = false;               // Keep single core for Arduino compatibility
    constexpr bool ENABLE_FAST_MATH = true;             // Enable fast math operations
    constexpr bool MINIMIZE_SERIAL_OUTPUT = true;       // Reduce serial overhead in competition mode
    constexpr bool ENABLE_SERIAL_OPTIMIZATION = true;   // Use faster serial output methods

    // -- Loop Timing Optimization --
    constexpr unsigned long TARGET_LOOP_TIME_US = 1000; // Target 1ms loop time (1000Hz)
    constexpr bool ENABLE_LOOP_TIMING_MONITOR = false;  // Disable in competition for max speed

    // -- Memory Optimization --
    constexpr bool ENABLE_MEMORY_OPTIMIZATION = true;   // Optimize memory usage for speed
    constexpr bool PRECOMPUTE_CONSTANTS = true;         // Pre-calculate values at compile time

    // -- Pre-computed Performance Constants --
    constexpr int PWM_HALF = constants::PWM_MAX / 2;    // Pre-computed PWM half value
    constexpr int PWM_QUARTER = constants::PWM_MAX / 4; // Pre-computed PWM quarter value
    constexpr int JOYSTICK_HALF = 64;                   // Pre-computed joystick half range (128/2)
    constexpr float PWM_TO_PERCENT_MULTIPLIER = 100.0f / constants::PWM_MAX; // Pre-computed conversion factor

    // -- Fast Math Macros for Performance-Critical Code --
    #define FAST_ABS(x) ((x) < 0 ? -(x) : (x))
    #define FAST_MIN(a, b) ((a) < (b) ? (a) : (b))
    #define FAST_MAX(a, b) ((a) > (b) ? (a) : (b))
    #define FAST_CLAMP(x, min, max) FAST_MIN(FAST_MAX(x, min), max)
    #define LIKELY(x) __builtin_expect(!!(x), 1)
    #define UNLIKELY(x) __builtin_expect(!!(x), 0)

    // -- I2C Performance --
    constexpr long I2C_FAST_MODE_PLUS = 1000000;        // 1MHz I2C (requires 1kΩ pull-ups)
    constexpr bool ENABLE_I2C_OPTIMIZATION = false;     // Disabled until pull-ups added

    // -- PS2 Controller Performance --
    constexpr bool ENABLE_PS2_OPTIMIZATION = true;      // Enable PS2 reading optimizations
    constexpr unsigned long PS2_READ_INTERVAL_US = 5000; // 5ms PS2 read interval (200Hz)
    constexpr bool SKIP_PS2_VALIDATION = true;          // Skip redundant PS2 data validation

    // -- I2C Performance (Proven Methods Only) --
    constexpr bool ENABLE_STANDARD_I2C_RECOVERY = true; // Use documented Arduino recovery methods
    constexpr unsigned int I2C_TIMEOUT_MS = 100;        // Proven: Faster timeout (Arduino docs recommend 10-1000ms)
  }

  //================================================================================
  // PERFORMANCE TUNING PARAMETERS
  // Tweak these values to optimize robot performance and behavior
  //================================================================================
  namespace tuning {
    // -- Drive Control Performance --
    constexpr int DRIVE_MAX_SPEED_PERCENT    = 70; // Maximum speed for normal driving (0-100%)
    constexpr int DRIVE_PRECISION_SPEED_PERCENT = 35; // Maximum speed for precision mode when R1 pressed (0-100%)
    constexpr int TURN_SENSITIVITY_PERCENT   = 60; // Turn sensitivity - higher = more responsive turning (0-100%)

    // Pre-calculated PWM values for performance optimization
    constexpr int DRIVE_MAX_SPEED_PWM = (constants::PWM_MAX * DRIVE_MAX_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;        // 2252 PWM units
    constexpr int DRIVE_PRECISION_PWM = (constants::PWM_MAX * DRIVE_PRECISION_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;  // 819 PWM units
    constexpr int OUTTAKE_PWM = DRIVE_MAX_SPEED_PWM;                                                                                    // Same as max drive speed
    constexpr int TURN_OFFSET_MULTIPLIER = (TURN_SENSITIVITY_PERCENT * constants::PWM_MAX) / (constants::PERCENT_TO_DECIMAL_DIVISOR * config::ps2::JOYSTICK_MAX); // Pre-calculated turn scaling

    // -- Drive Control Style --
    constexpr bool USE_CHEESY_DRIVE = true;  // Use Cheesy Drive (maintains speed while turning) vs Differential Drive
    constexpr int CHEESY_DRIVE_NONLINEARITY = 50; // Cheesy drive turning feel (0-100%) - higher = more aggressive turning

    // -- Two-Stage Deceleration Control --
    // Stage 1: Ramp down smoothly using DECELERATION_TIME_MS
    // Stage 2: Apply electromagnetic brake for final stop and direction changes
    constexpr int TURN_DETECTION_THRESHOLD = 15;     // Minimum speed difference % to consider as turning (0-100%)

    // -- Mixed-Decay Braking Configuration --
    constexpr bool ENABLE_ACTIVE_BRAKING = true;     // Enable mixed-decay braking system
    constexpr bool USE_ELECTROMAGNETIC_BRAKING = true; // Use terminal shorting for slow decay

    // Mixed-Decay Parameters
    constexpr int BRAKE_BASE_TIME_MS = 25;          // Base brake time for low speeds (ms)
    constexpr int BRAKE_MAX_TIME_MS = 100;           // Maximum brake time for high speeds (ms)
    constexpr int FAST_DECAY_PERCENT = 30;           // Fast decay percentage (0-100%) - coasting phase
    constexpr int SLOW_DECAY_PERCENT = 100 - FAST_DECAY_PERCENT;           // Slow decay percentage (100 - fast_decay) - electromagnetic braking
    constexpr int BRAKE_POWER_PERCENT = 100;         // Electromagnetic braking power during slow decay phase
    constexpr int BRAKE_HOLD_POWER_PERCENT = 100;    // Final holding power after braking complete
    constexpr int BRAKE_THRESHOLD_PERCENT = 5;       // Minimum speed % to trigger mixed-decay braking

    // -- Motor Acceleration/Deceleration Limiting --
    constexpr bool ENABLE_MOTOR_RAMPING = true;      // Enable time-based acceleration/deceleration limiting
    constexpr int ACCELERATION_TIME_MS = 150;        // Time to accelerate from 0 to full throttle (ms)
    constexpr int DECELERATION_TIME_MS = 75;         // Time to decelerate from full throttle to 0 (ms)



    // Motor ramping parameters (see ACCELERATION_TIME_MS and DECELERATION_TIME_MS above)

    // -- Outtake Timeouts --
    constexpr int OUTTAKE_TIMEOUT_MS = 3000; // Timeout for the automatic outtake sequence (ms)
    constexpr int OUTTAKE_FORWARD_TIMEOUT_MS = 3250; // Timeout for the timed forward outtake (ms)

    // Note: Removed HOLD_POWER_PERCENT (was always 0, effectively disabled)

    // -- Servo Angles --
    constexpr int INTAKE_ARM_CLOSE_ANGLE  = 135;
    constexpr int INTAKE_ARM_OPEN_ANGLE   = 45;
    constexpr int OUTTAKE_ARM_OPEN_ANGLE  = 145;
    constexpr int OUTTAKE_ARM_CLOSE_ANGLE = 45;

    // -- Drive Inversion Settings (Runtime Configurable) --
    inline bool INVERT_DRIVE_LEFT = false;
    inline bool INVERT_DRIVE_RIGHT = true;
    inline bool INVERT_OUTTAKE_RIGHT = true;
    inline bool LIMIT_SWITCH_DISABLED = false;
    constexpr unsigned long DEBOUNCE_DELAY_MS = 50; // Debounce delay for limit switch
    constexpr int CONNECTION_TIMEOUT_MS = 1000;  // Time before robot goes IDLE: 1 second
    constexpr int RECONNECTION_ATTEMPT_INTERVAL_MS = 5;   // Optimized: Faster reconnection attempts

  }
}

#endif // CONFIG_H
