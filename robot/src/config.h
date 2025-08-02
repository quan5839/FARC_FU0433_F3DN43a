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

    // Time constants
    constexpr long MICROSECONDS_PER_SECOND = 1000000L; // 1 second = 1,000,000 microseconds

    // Unit conversion constants
    constexpr int KILOHERTZ_DIVISOR = 1000;          // Convert Hz to kHz

    // Servo angle limits
    constexpr int SERVO_MIN_ANGLE = 0;               // Minimum servo angle (degrees)
    constexpr int SERVO_MAX_ANGLE = 180;             // Maximum servo angle (degrees)

    // Mathematical constants
    constexpr int PERCENT_TO_DECIMAL_DIVISOR = 100;
    constexpr int QUARTER_DIVISOR = 4;
    constexpr int MAX_CONSECUTIVE_ERRORS = 5;

    // Communication
    constexpr int SERIAL_BAUD_RATE = 115200;
    constexpr unsigned long PS2_RETRY_DELAY_MS = 25;

    // Motor control
    constexpr bool MOTOR_NOT_INVERTED = false;
    constexpr bool MOTOR_INVERTED = true;

    // Sensor constants
    constexpr int LIMIT_SWITCH_TRIGGERED = 0;        // LOW when triggered (INPUT_PULLUP)

    // Safety constants
    constexpr unsigned long TIMEOUT_SAFETY_MARGIN_MS = 1000;
    constexpr bool ENABLE_TEMPERATURE_MONITORING = true;
    constexpr float MAX_ESP32_TEMPERATURE = 80.0;
    constexpr bool ENABLE_WATCHDOG = true;
    constexpr unsigned long WATCHDOG_FEED_INTERVAL_MS = 1000;
    constexpr unsigned long WATCHDOG_TIMEOUT_MS = 5000;
    constexpr unsigned long SAFETY_CHECK_INTERVAL_MS = 5000;

    // Controller input safety constants
    constexpr bool ENABLE_CONTROLLER_INPUT_SAFETY = true;
    constexpr unsigned long CONTROLLER_INPUT_SAFETY_TIMEOUT_MS = 10000;  
    constexpr int CONTROLLER_INPUT_CHANGE_THRESHOLD = 1;  // Minimum change to detect input variation

    // Memory optimization constants
    constexpr size_t PWM_CALCULATION_POOL_SIZE = 4;
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
    TIMED_OUTTAKE_FORWARD,
    // Homing sequence states (3D printer style)
    HOMING_INITIAL_CLEARANCE,
    HOMING_FAST_APPROACH,
    HOMING_RETRACTION,
    HOMING_SLOW_APPROACH,
    HOMING_FINAL_POSITION,
    HOMING_COMPLETE
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
    constexpr int JOYSTICK_DEADZONE = 1; // Dead zone for joystick inputs (0-128 range)
    constexpr int JOYSTICK_MAX = 128; // Maximum joystick value after processing
  }



  // -- Servos --
  namespace servo {
    constexpr int MIN_PULSE = 500;
    constexpr int MAX_PULSE = 2500;

    // ESP32 LEDC servo control (for future use)
    constexpr int LEDC_FREQUENCY = 50;        // 50Hz for servo compatibility
    constexpr int LEDC_RESOLUTION = 16;       // 16-bit resolution (0-65535)
    constexpr int LEDC_MIN_DUTY = 1638;       // ~500us pulse (1638/65535 * 20ms)
    constexpr int LEDC_MAX_DUTY = 8192;       // ~2500us pulse (8192/65535 * 20ms)
  }

  // Note: PWM frequency constants moved to constants namespace to eliminate duplication

  // -- PCA9685 Driver --
  namespace pca9685 {
    constexpr int OSCILLATOR_FREQ = 27000000;               // PCA9685 internal oscillator frequency (27MHz)
    constexpr long I2C_CLOCK_SPEED = 1000000;               // 1MHZ
    constexpr uint8_t I2C_ADDRESS = 0x40;                   // PCA9685 default I2C address (7-bit)

    // Hardware-specific constants
    constexpr int OSCILLATOR_FREQ_MHZ = 27;                 // Oscillator frequency in MHz for readability
    constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x40;           // Default factory I2C address
  }

  // -- MPU6050 IMU --
  namespace imu {
    constexpr uint8_t I2C_ADDRESS = 0x68;                   // MPU6050 default I2C address
    constexpr uint8_t I2C_ADDRESS_ALT = 0x69;               // MPU6050 alternative I2C address
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
  constexpr uint8_t BALL_SERVO_CHANNEL    = 2;  // PCA9685 channel for ball servo (was outtake)
  constexpr uint8_t FRUIT_SERVO_CHANNEL   = 3;  // PCA9685 channel for fruit servo (was intake)
  constexpr uint8_t FRUIT_INTAKE_LEFT_SERVO_CHANNEL  = 4;  // PCA9685 channel for fruit intake left continuous servo
  constexpr uint8_t FRUIT_INTAKE_RIGHT_SERVO_CHANNEL = 5;  // PCA9685 channel for fruit intake right continuous servo (reverse)

  // Sensor Pins
  constexpr uint8_t LIMIT_SWITCH_PIN = 32;

  // WS2812B LED Strip Control
  constexpr uint8_t WS2812B_DATA_PIN = 25;         // GPIO 25 for WS2812B data signal
  constexpr uint8_t WS2812B_POWER_CHANNEL = 7;     // PCA9685 channel 7 for power switching
  constexpr int WS2812B_NUM_LEDS = 60;            // Number of LEDs in the strip

  // -- LED Status Codes --
  namespace led_status {
    constexpr uint8_t OFF = 0;                      // LED strip off
    constexpr uint8_t IDLE = 1;                     // Idle state - slow blue breathing
    constexpr uint8_t MANUAL_CONTROL = 2;           // Manual control - solid green
    constexpr uint8_t AUTOMATIC_MODE = 3;           // Automatic mode - rainbow animation
    constexpr uint8_t SYSTEM_ERROR = 4;             // System error - fast red blinking
    constexpr uint8_t CONTROLLER_SAFETY = 5;        // Controller safety shutdown - orange warning
    constexpr uint8_t CONTROLLER_DISCONNECTED = 6;  // Controller disconnected - slow red breathing
    constexpr uint8_t TEMPERATURE_WARNING = 7;      // High temperature - yellow pulse
    constexpr uint8_t STARTUP_SEQUENCE = 8;         // System startup - rainbow sweep
    constexpr uint8_t I2C_ERROR = 9;                // I2C communication error - purple blink
    constexpr uint8_t LIMIT_SWITCH_ACTIVE = 10;     // Limit switch triggered - cyan solid
    // Homing sequence status codes
    constexpr uint8_t HOMING_INITIAL = 11;          // Initial clearance phase - orange pulse
    constexpr uint8_t HOMING_FAST = 12;             // Fast approach phase - fast blue pulse
    constexpr uint8_t HOMING_RETRACT = 13;          // Retraction phase - slow yellow pulse
    constexpr uint8_t HOMING_SLOW = 14;             // Slow approach phase - slow blue pulse
    constexpr uint8_t HOMING_FINAL = 15;            // Final positioning - green pulse
  }




  //================================================================================
  // ESP32 PERFORMANCE OPTIMIZATION
  // Maximum performance settings for competition robotics
  //================================================================================
  namespace performance {
    // -- CPU Performance --
    constexpr int ESP32_MAX_CPU_FREQ_MHZ = 240; // Maximum ESP32 CPU frequency for optimal performance

    // -- Loop Timing --
    constexpr unsigned long TARGET_LOOP_TIME_US = 1000; // Target 1ms loop time (1000Hz)

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
    constexpr int DRIVE_MAX_SPEED_PERCENT    = 90; // Maximum speed for normal driving (0-100%)
    constexpr int DRIVE_PRECISION_SPEED_PERCENT = 17; // Maximum speed for precision mode when R1 pressed (0-100%)
    constexpr int TURN_SENSITIVITY_PERCENT   = 37; // Base turn sensitivity - higher = more responsive turning (0-100%)

    // -- Dynamic Turning Sensitivity --
    constexpr float MIN_TURN_MULTIPLIER = 1.0f;  // Turn sensitivity when not moving (100% of base)
    constexpr float MAX_TURN_MULTIPLIER = 2.0f;  // Turn sensitivity at full speed (200% of base)

    // Pre-calculated PWM values for performance optimization
    constexpr int DRIVE_MAX_SPEED_PWM = (constants::PWM_MAX * DRIVE_MAX_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;        // 3685 PWM units (90%)
    constexpr int DRIVE_PRECISION_PWM = (constants::PWM_MAX * DRIVE_PRECISION_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;  // 696 PWM units (17%)

    // -- Outtake Motor Settings (Independent from Drive) --
    constexpr int OUTTAKE_MAX_SPEED_PERCENT = 100;  // Outtake motors always run at full power (0-100%)
    constexpr int OUTTAKE_PWM = (constants::PWM_MAX * OUTTAKE_MAX_SPEED_PERCENT) / constants::PERCENT_TO_DECIMAL_DIVISOR;              // 4095 PWM units (full power)
    constexpr int OUTTAKE_HOLD_POWER_PERCENT = 10;  // Forward holding power to prevent rolling back (0-100%)
    constexpr int OUTTAKE_REVERSE_HOLD_POWER_PERCENT = 7;  // Reverse holding power when limit switch is active (0-100%)
    constexpr int OUTTAKE_SELECT_SPEED_PERCENT = 75;  // SELECT button outtake reverse speed (0-100%)

    // -- Motor Braking Configuration (Drive Motors) --
    constexpr bool ENABLE_ACTIVE_BRAKING = true;
    constexpr int BRAKE_POWER_PERCENT = 100;
    constexpr int BRAKE_THRESHOLD_PERCENT = 1;
    constexpr int BRAKE_BASE_TIME_MS = 50;
    constexpr int BRAKE_MAX_TIME_MS = 300;    // Normal brake time for drive motors
    constexpr int BRAKE_HOLD_POWER_PERCENT = 100;
    constexpr int FAST_DECAY_PERCENT = 30;    // Normal coasting for drive motors
    constexpr int SLOW_DECAY_PERCENT = 70;    // Normal electromagnetic braking for drive motors

    // -- Outtake Motor Braking Configuration --
    constexpr int OUTTAKE_BRAKE_MAX_TIME_MS = -1;     // Infinite brake time (never timeout)
    constexpr int OUTTAKE_FAST_DECAY_PERCENT = 0;     // No coasting - immediate electromagnetic braking
    constexpr int OUTTAKE_SLOW_DECAY_PERCENT = 100;   // 100% electromagnetic braking

    // -- Motor Ramping --
    constexpr bool ENABLE_MOTOR_RAMPING = true;
    constexpr int ACCELERATION_TIME_MS = 0;
    constexpr int DECELERATION_TIME_MS = 100;




    // -- Timeouts --
    constexpr int OUTTAKE_SELECT_TIMEOUT_MS = 2500;
    constexpr int OUTTAKE_TIMEOUT_MS = 1500;
    constexpr int OUTTAKE_FORWARD_TIMEOUT_MS = 1500;

    // -- Servo Angles --
    constexpr int FRUIT_SERVO_CLOSE_ANGLE = 140;
    constexpr int FRUIT_SERVO_OPEN_ANGLE = 45;
    constexpr int BALL_SERVO_OPEN_ANGLE = 145;
    constexpr int BALL_SERVO_CLOSE_ANGLE = 45;

    // -- Continuous Servo Speeds --
    constexpr int FRUIT_INTAKE_SERVO_SPEED = 100;  // Speed for fruit intake servos (0-100%, positive = down/intake)

    // -- Runtime Settings --
    inline bool INVERT_DRIVE_LEFT = false;
    inline bool INVERT_DRIVE_RIGHT = true;
    inline bool INVERT_OUTTAKE_RIGHT = true;
    inline bool LIMIT_SWITCH_DISABLED = false;
    constexpr unsigned long DEBOUNCE_DELAY_MS = 50;

    // -- PS2 Controller --
    constexpr int CONNECTION_TIMEOUT_MS = 1000;
    constexpr int RECONNECTION_ATTEMPT_INTERVAL_MS = 1000;

    // -- Homing Sequence (3D Printer Style) --
    constexpr bool ENABLE_STARTUP_HOMING = false;           // Enable automatic homing on startup
    constexpr int HOMING_FAST_SPEED_PERCENT = 50;          // Fast approach speed (like 3D printer fast homing)
    constexpr int HOMING_SLOW_SPEED_PERCENT = 30;          // Slow precision speed (like 3D printer precision homing)
    constexpr unsigned long HOMING_TIMEOUT_MS = 10000;     // Total homing timeout (10 seconds)
    constexpr unsigned long HOMING_INITIAL_CLEARANCE_MS = 400; // Initial clearance time if switch already triggered
    constexpr unsigned long HOMING_BUMP_DISTANCE_MS = 750; // Retraction time after first contact (like bump distance)
    constexpr unsigned long HOMING_SAFE_POSITION_MS = 1000; // Time to move to safe operating position



    // -- WS2812B LED Strip Settings --
    constexpr uint8_t WS2812B_BRIGHTNESS = 128;                 // Global brightness (0-255, 50% for safety)
    constexpr unsigned long WS2812B_UPDATE_INTERVAL_MS = 50;    // Update interval (20 FPS)
    constexpr uint8_t WS2812B_DEFAULT_HUE = 120;                // Default hue (green)
    constexpr uint8_t WS2812B_DEFAULT_SAT = 255;                // Default saturation (full)
    constexpr uint8_t WS2812B_DEFAULT_VAL = 128;                // Default value/brightness
    constexpr int WS2812B_POWER_ON_PWM = 4095;                  // Full PWM to enable power via PCA9685
    constexpr int WS2812B_POWER_OFF_PWM = 0;                    // Zero PWM to disable power via PCA9685

    // -- LED Animation Constants --
    constexpr unsigned long STARTUP_SEQUENCE_DURATION_MS = 3000; // 3 second startup sequence
    constexpr int STARTUP_ANIMATION_FPS = 20;                    // 20 FPS for smooth animation
    constexpr int STARTUP_FRAME_DELAY_MS = 50;                   // 50ms delay between frames
    constexpr uint8_t STARTUP_SWEEP_SPEED = 8;                   // Speed multiplier for startup sweep
    constexpr int STARTUP_SWEEP_TAIL_LENGTH = 10;               // Length of rainbow tail in startup
    constexpr int STARTUP_SWEEP_EXTRA_RANGE = 20;               // Extra range for sweep animation
    constexpr uint8_t STARTUP_HUE_STEP = 25;                    // Hue step between LEDs in startup
    constexpr uint8_t STARTUP_HUE_SPEED = 5;                    // Hue animation speed

    // -- LED Color Constants (HSV Hue values 0-255) --
    constexpr uint8_t LED_HUE_RED = 0;                          // Red hue
    constexpr uint8_t LED_HUE_ORANGE = 30;                      // Orange hue
    constexpr uint8_t LED_HUE_YELLOW = 60;                      // Yellow hue
    constexpr uint8_t LED_HUE_GREEN = 120;                      // Green hue
    constexpr uint8_t LED_HUE_BLUE = 160;                       // Blue hue
    constexpr uint8_t LED_HUE_CYAN = 180;                       // Cyan hue
    constexpr uint8_t LED_HUE_PURPLE = 200;                     // Purple hue

    // -- LED Animation Speed Constants --
    constexpr uint8_t LED_SPEED_SLOW = 1;                       // Slow animation speed
    constexpr uint8_t LED_SPEED_MEDIUM = 2;                     // Medium animation speed
    constexpr uint8_t LED_SPEED_NORMAL = 3;                     // Normal animation speed
    constexpr uint8_t LED_SPEED_FAST = 5;                       // Fast animation speed

    // -- LED Blink Pattern Constants --
    constexpr uint8_t LED_BLINK_FAST_DIVISOR = 5;               // Fast blink rate (animation_counter / 5)
    constexpr uint8_t LED_BLINK_SLOW_DIVISOR = 8;               // Slow blink rate (animation_counter / 8)
    constexpr uint8_t LED_BREATHING_MULTIPLIER = 2;             // Breathing speed multiplier
    constexpr uint8_t LED_PULSE_MULTIPLIER = 3;                 // Pulse speed multiplier

    // -- LED Hardware Test Constants --
    constexpr uint8_t LED_TEST_BRIGHTNESS = 255;                // Full brightness for hardware test
    constexpr int LED_TEST_COUNT = 10;                          // Number of LEDs to test
    constexpr int LED_TEST_DELAY_MS = 500;                      // Delay between test steps
    constexpr int LED_TEST_COLOR_CYCLE = 3;                     // RGB color cycle for test

    // -- LED Transition Constants --
    constexpr unsigned long LED_TRANSITION_DURATION_MS = 500;   // Smooth transition duration
    constexpr uint8_t LED_TRANSITION_STEPS = 50;                // Number of transition steps
    constexpr unsigned long LED_FADE_STEP_MS = 10;              // Time between fade steps




  }
}

#endif // CONFIG_H
