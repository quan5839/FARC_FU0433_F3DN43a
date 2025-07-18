#line 1 "/Users/admin/Documents/Arduino/robot/src/utils/safety_monitor.cpp"
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
char SafetyMonitor::_status_buffer[128];

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
        Serial.println("Hardware watchdog initialized");
    }

    _safety_initialized = true;
    _last_safety_check = millis();
    _last_watchdog_feed = millis();
    
    Serial.println("Safety monitoring system initialized");
}

void SafetyMonitor::update() {
    if (!_safety_initialized) return;

    unsigned long current_time = millis();
    
    // Feed watchdog regularly
    if (config::constants::ENABLE_WATCHDOG && 
        (current_time - _last_watchdog_feed) > 1000) { // Feed every second
        feedWatchdog();
    }

    // Perform safety checks every 5 seconds
    if ((current_time - _last_safety_check) > 5000) {
        if (config::constants::ENABLE_VOLTAGE_MONITORING) {
            checkBatteryVoltage();
        }
        
        if (config::constants::ENABLE_TEMPERATURE_MONITORING) {
            checkTemperature();
        }
        
        _last_safety_check = current_time;
    }
}

void SafetyMonitor::feedWatchdog() {
    if (config::constants::ENABLE_WATCHDOG) {
        esp_task_wdt_reset();
        _last_watchdog_feed = millis();
    }
}

bool SafetyMonitor::isSystemSafe() {
    if (_emergency_shutdown_active) return false;
    
    bool safe = true;
    
    if (config::constants::ENABLE_VOLTAGE_MONITORING && !isBatteryOK()) {
        safe = false;
    }
    
    if (config::constants::ENABLE_TEMPERATURE_MONITORING && !isTemperatureOK()) {
        safe = false;
    }
    
    return safe;
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
    float voltage = getBatteryVoltage();
    
    if (voltage < config::constants::MIN_BATTERY_VOLTAGE) {
        return false;
    }
    
    if (voltage < config::constants::LOW_BATTERY_WARNING) {
        Serial.print("WARNING: Low battery voltage: ");
        Serial.print(voltage);
        Serial.println("V");
    }
    
    return true;
}

bool SafetyMonitor::isTemperatureOK() {
    float temp = getESP32Temperature();
    
    if (temp > config::constants::MAX_ESP32_TEMPERATURE) {
        Serial.print("WARNING: High ESP32 temperature: ");
        Serial.print(temp);
        Serial.println("°C");
        return false;
    }
    
    return true;
}

void SafetyMonitor::emergencyShutdown() {
    _emergency_shutdown_active = true;
    Serial.println("EMERGENCY SHUTDOWN ACTIVATED");
    
    // Additional emergency actions can be added here
    // The robot class should check isSystemSafe() and stop all motors
}

const char* SafetyMonitor::getSafetyStatus() {
    snprintf(_status_buffer, sizeof(_status_buffer), 
             "Safety: %s | Battery: %.1fV | Temp: %.1f°C | Emergency: %s",
             isSystemSafe() ? "OK" : "UNSAFE",
             _last_battery_voltage,
             _last_temperature,
             _emergency_shutdown_active ? "ACTIVE" : "INACTIVE");
    
    return _status_buffer;
}

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
    Serial.print("UNSAFE CONDITION DETECTED: ");
    Serial.println(reason);
    
    // For now, just log the condition
    // In a more advanced system, you might reduce power or trigger emergency stop
    // emergencyShutdown(); // Uncomment for more aggressive safety
}
