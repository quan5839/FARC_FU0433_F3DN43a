#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "../config.h"
#include "../controller/controller_state.h"

/**
 * @class SafetyMonitor
 * @brief Comprehensive safety monitoring system for robot health and reliability
 *
 * This class provides:
 * - Hardware watchdog timer management
 * - Battery voltage monitoring
 * - ESP32 temperature monitoring
 * - Controller input safety monitoring
 * - System health checks
 * - Emergency shutdown capabilities
 */
class SafetyMonitor {
public:
    /**
     * @brief Initialize safety monitoring systems
     */
    static void init();

    /**
     * @brief Update safety monitoring (call in main loop)
     */
    static void update();

    /**
     * @brief Feed the watchdog timer to prevent reset
     */
    static void feedWatchdog();

    /**
     * @brief Monitor controller input for safety violations
     * @param controllerState Current controller state to monitor
     * @return true if controller input is safe, false if safety violation detected
     */
    static bool checkControllerInputSafety(const ControllerState& controllerState);

    /**
     * @brief Check if controller input safety shutdown is active
     * @return true if controller safety shutdown is active
     */
    static bool isControllerSafetyShutdownActive();

    /**
     * @brief Reset controller input safety monitoring
     */
    static void resetControllerInputSafety();

    /**
     * @brief Unified safety check - consolidates all safety monitoring
     * @return true if all safety checks pass
     */
    static bool isSystemSafe();

    /**
     * @brief Fast safety check for critical operations (inlined for performance)
     * @return true if system is safe for immediate operation
     */
    static inline bool isSafeForOperation() {
        return !_emergency_shutdown_active && !_controller_safety_shutdown_active;
    }

    // Note: Other functions moved to private section since they're only used internally

private:
    static unsigned long _last_safety_check;
    static unsigned long _last_watchdog_feed;
    static bool _safety_initialized;
    static bool _emergency_shutdown_active;
    static float _last_battery_voltage;
    static float _last_temperature;

    // Controller input safety monitoring
    static bool _controller_safety_shutdown_active;
    static unsigned long _controller_input_start_time;
    static bool _controller_input_detected;
    static int _last_left_joystick_y;
    static int _last_right_joystick_x;
    static bool _last_button_states;

    // Internal helper functions
    static float getBatteryVoltage();
    static float getESP32Temperature();
    static bool isBatteryOK();
    static bool isTemperatureOK();
    static void checkBatteryVoltage();
    static void checkTemperature();
    static void handleUnsafeCondition(const char* reason);
};

#endif // SAFETY_MONITOR_H
