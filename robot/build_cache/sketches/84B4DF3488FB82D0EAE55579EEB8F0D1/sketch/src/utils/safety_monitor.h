#line 1 "/Users/admin/Documents/Arduino/robot/src/utils/safety_monitor.h"
#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "../config.h"

/**
 * @class SafetyMonitor
 * @brief Comprehensive safety monitoring system for robot health and reliability
 * 
 * This class provides:
 * - Hardware watchdog timer management
 * - Battery voltage monitoring
 * - ESP32 temperature monitoring
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
     * @brief Check if system is in safe operating condition
     * @return true if all safety checks pass
     */
    static bool isSystemSafe();

    /**
     * @brief Get current battery voltage
     * @return Battery voltage in volts
     */
    static float getBatteryVoltage();

    /**
     * @brief Get current ESP32 temperature
     * @return Temperature in Celsius
     */
    static float getESP32Temperature();

    /**
     * @brief Check if battery voltage is adequate
     * @return true if battery voltage is above minimum
     */
    static bool isBatteryOK();

    /**
     * @brief Check if ESP32 temperature is within safe limits
     * @return true if temperature is safe
     */
    static bool isTemperatureOK();

    /**
     * @brief Trigger emergency shutdown
     */
    static void emergencyShutdown();

    /**
     * @brief Get human-readable safety status
     * @return Status string for debugging
     */
    static const char* getSafetyStatus();

private:
    static unsigned long _last_safety_check;
    static unsigned long _last_watchdog_feed;
    static bool _safety_initialized;
    static bool _emergency_shutdown_active;
    static float _last_battery_voltage;
    static float _last_temperature;
    static char _status_buffer[128];

    static void checkBatteryVoltage();
    static void checkTemperature();
    static void handleUnsafeCondition(const char* reason);
};

#endif // SAFETY_MONITOR_H
