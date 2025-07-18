#line 1 "/Users/admin/Documents/Arduino/robot/src/utils/validation.cpp"
#include "validation.h"
#include "../hal/servos.h"
#include <Arduino.h>

bool Validation::setServoAngleSafe(uint8_t channel, int angle) {
    // Validate channel
    if (!isValidServoChannel(channel)) {
        printValidationError("servo channel", channel, "0-15");
        return false;
    }
    
    // Validate angle
    if (!isValidServoAngle(angle)) {
        printValidationError("servo angle", angle, "0-180 degrees");
        // Clamp to valid range and continue
        angle = clampServoAngle(angle);
        Serial.print("Clamped to: ");
        Serial.println(angle);
    }
    
    // Set the servo angle
    setServoAngle(channel, angle);
    return true;
}

int Validation::setMotorSpeedSafe(int speed) {
    // Validate and clamp speed
    if (!isValidPWM(speed)) {
        printValidationError("motor speed", speed, "-4095 to +4095");
        speed = clampPWM(speed);
        Serial.print("Clamped to: ");
        Serial.println(speed);
    }
    
    return speed;
}

void Validation::printValidationError(const char* paramName, int value, const char* validRange) {
    Serial.print("VALIDATION ERROR: ");
    Serial.print(paramName);
    Serial.print(" = ");
    Serial.print(value);
    Serial.print(" (valid range: ");
    Serial.print(validRange);
    Serial.println(")");
}
