#include "safety_monitor.h"
#include <Arduino.h>
#include <esp_task_wdt.h>
#include <esp_system.h>

// Static member initialization
unsigned long SafetyMonitor::_last_safety_check = 0;
unsigned long SafetyMonitor::_last_watchdog_feed = 0;
bool SafetyMonitor::_safety_initialized = false;
bool SafetyMonitor::_emergency_shutdown_active = false;
float SafetyMonitor::_last_battery_voltage = 0.0;
float SafetyMonitor::_last_temperature = 0.0;

void SafetyMonitor::init() {
    if (config::constants::ENABLE_WATCHDOG) {
        // Initialize ESP32 hardware watchdog (newer API)
        esp_task_wdt_config_t wdt_config = {
            .timeout_ms = config::constants::WATCHDOG_TIMEOUT_MS,
            .idle_core_mask = 0,
            .trigger_panic = true
        };
        esp_task_wdt_init(&wdt_config);
        esp_task_wdt_add(NULL); // Add current task to watchdog
        DEBUG_PRINTLN("Hardware watchdog initialized");
    }

    _safety_initialized = true;
    _last_safety_check = millis();
    _last_watchdog_feed = millis();

    DEBUG_PRINTLN("Safety monitoring system initialized");
}

void SafetyMonitor::update() {
    if (UNLIKELY(!_safety_initialized)) return;

    const unsigned long current_time = millis();

    // Performance-optimized watchdog feeding with branch prediction
    if (LIKELY(config::constants::ENABLE_WATCHDOG) &&
        UNLIKELY((current_time - _last_watchdog_feed) > config::constants::WATCHDOG_FEED_INTERVAL_MS)) {
        esp_task_wdt_reset();  // Inline the feedWatchdog call
        _last_watchdog_feed = current_time;
    }

    // Optimized safety checks with minimal overhead
    if (UNLIKELY((current_time - _last_safety_check) > config::constants::SAFETY_CHECK_INTERVAL_MS)) {
        if (UNLIKELY(config::constants::ENABLE_TEMPERATURE_MONITORING)) {
            checkTemperature();
        }
        _last_safety_check = current_time;
    }
}

void SafetyMonitor::feedWatchdog() {
    // Optimized to avoid redundant checks when called from Robot class
    esp_task_wdt_reset();
    _last_watchdog_feed = millis();
}

bool SafetyMonitor::isSystemSafe() {
    if (_emergency_shutdown_active) return false;

    // Simplified: Only temperature monitoring on VIA board
    return !config::constants::ENABLE_TEMPERATURE_MONITORING || isTemperatureOK();
}

float SafetyMonitor::getBatteryVoltage() {
    // NOTE: VIA "banh mi" v2023 board does not provide battery voltage monitoring
    // Battery connects directly to power board via XT60 connector
    // ESP32 has no direct access to battery voltage without additional circuitry

    // Return a placeholder voltage to prevent safety system from triggering
    // If you add voltage monitoring hardware, implement the actual reading here
    _last_battery_voltage = 12.0; // Placeholder - assume healthy battery
    return _last_battery_voltage;
}

float SafetyMonitor::getESP32Temperature() {
    // Get ESP32 internal temperature
    // Note: This is approximate and varies between chips
    float temp = temperatureRead();
    _last_temperature = temp;
    return temp;
}

bool SafetyMonitor::isBatteryOK() {
    // Always return true since voltage monitoring is disabled on VIA board
    // getBatteryVoltage() returns placeholder value anyway
    return true;
}

bool SafetyMonitor::isTemperatureOK() {
    float temp = getESP32Temperature();
    
    if (temp > config::constants::MAX_ESP32_TEMPERATURE) {
        ERROR_PRINT("WARNING: High ESP32 temperature: ");
        ERROR_PRINT(temp);
        ERROR_PRINTLN("°C");
        return false;
    }
    
    return true;
}

// Note: emergencyShutdown() and getSafetyStatus() removed as they were unused

void SafetyMonitor::checkBatteryVoltage() {
    if (!isBatteryOK()) {
        handleUnsafeCondition("Low battery voltage");
    }
}

void SafetyMonitor::checkTemperature() {
    if (!isTemperatureOK()) {
        handleUnsafeCondition("High temperature");
    }
}

void SafetyMonitor::handleUnsafeCondition(const char* reason) {
    ERROR_PRINT("UNSAFE CONDITION DETECTED: ");
    ERROR_PRINTLN(reason);
    
    // For now, just log the condition
    // In a more advanced system, you might reduce power or trigger emergency stop
    // emergencyShutdown(); // Uncomment for more aggressive safety
}
