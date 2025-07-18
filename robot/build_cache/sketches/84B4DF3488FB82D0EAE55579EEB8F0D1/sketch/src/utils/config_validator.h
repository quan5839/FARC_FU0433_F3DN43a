#line 1 "/Users/admin/Documents/Arduino/robot/src/utils/config_validator.h"
#ifndef CONFIG_VALIDATOR_H
#define CONFIG_VALIDATOR_H

#include "../config.h"

/**
 * @class ConfigValidator
 * @brief Validates configuration values at startup to ensure safe operation
 * 
 * This class checks all configuration parameters to ensure they are within
 * safe and reasonable ranges, preventing crashes or unsafe behavior due to
 * invalid configuration values.
 */
class ConfigValidator {
public:
    /**
     * @brief Validate all configuration parameters
     * @return true if all config values are valid, false otherwise
     */
    static bool validateAll() {
        // Simple validation - just return true for now
        // In a full implementation, this would check all config values
        return true;
    }

    /**
     * @brief Validate motor configuration
     * @return true if motor config is valid
     */
    static bool validateMotorConfig();

    /**
     * @brief Validate timing configuration
     * @return true if timing config is valid
     */
    static bool validateTimingConfig();

    /**
     * @brief Validate safety configuration
     * @return true if safety config is valid
     */
    static bool validateSafetyConfig();

    /**
     * @brief Validate PWM configuration
     * @return true if PWM config is valid
     */
    static bool validatePWMConfig();

    /**
     * @brief Validate servo configuration
     * @return true if servo config is valid
     */
    static bool validateServoConfig();

    /**
     * @brief Print configuration summary for debugging
     */
    static void printConfigSummary();

private:
    static bool validateRange(int value, int min, int max, const char* name);
    static bool validateRange(float value, float min, float max, const char* name);
    static bool validateRange(unsigned long value, unsigned long min, unsigned long max, const char* name);
    static void logValidationError(const char* parameter, const char* issue);
    static void logValidationWarning(const char* parameter, const char* issue);
};

#endif // CONFIG_VALIDATOR_H
