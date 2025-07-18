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
     * @brief Unified safety check - consolidates all safety monitoring
     * @return true if all safety checks pass
     */
    static bool isSystemSafe();

    /**
     * @brief Fast safety check for critical operations (inlined for performance)
     * @return true if system is safe for immediate operation
     */
    static inline bool isSafeForOperation() {
        return !_emergency_shutdown_active;
    }

    // Note: Other functions moved to private section since they're only used internally

private:
    static unsigned long _last_safety_check;
    static unsigned long _last_watchdog_feed;
    static bool _safety_initialized;
    static bool _emergency_shutdown_active;
    static float _last_battery_voltage;
    static float _last_temperature;

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
