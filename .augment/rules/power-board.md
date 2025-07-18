---
type: "agent_requested"
description: "Technical reference for power board with 1x PCA9685 PWM controller and 4x TA6586 H-bridge motor drivers. Covers motor control, electromagnetic braking, and IO port connections."
---
# Power Board Technical Reference

## Overview
The power board combines 1x PCA9685 PWM controller with 4x TA6586 H-bridge motor driver ICs, providing 16-channel PWM control for motors and servos, connecting to the VIA "banh mi" v2023 board via I2C. This reference covers the complete power board architecture and capabilities.

## Hardware Architecture

### PCA9685 PWM Controller (1x IC)
- **Function**: Central PWM signal generator
- **Channels**: 16 independent PWM outputs (0-15)
- **Resolution**: 12-bit PWM (0-4095 values)
- **Frequency**: Fixed at 50Hz (servo compatibility priority)
- **Communication**: I2C interface (address 0x40 default)
- **Current Capacity**: 25mA per channel (control signals only)

### TA6586 H-Bridge Motor Drivers (4x ICs)
- **Quantity**: 4x TA6586 ICs (one per motor pair)
- **Current Rating**: 5A continuous per IC
- **Voltage Range**: 3V-14V motor supply voltage
- **Input Signal**: 2.2V-6.0V (compatible with PCA9685 3.3V output)
- **Control**: Dual H-bridge per IC for bidirectional motor control
- **Braking**: Supports slow decay electromagnetic braking

### IO Port Connections
- **Connection Type**: Direct GPIO pins to ESP32 (NOT through I2C)
- **Available Pins**: IO32, IO2, IO0, IO25, IO39, IO36
- **Access**: Direct pin-to-pin connection from ESP32 to power board headers
- **Usage**: Sensors, limit switches, additional peripherals

### Power Supply Architecture
- **Battery Connection**: Direct XT60 connector to power board
- **Power Distribution**: Power board distributes to motors and ESP32
- **Voltage Monitoring**: **NOT AVAILABLE** - ESP32 has no access to battery voltage
- **Important**: VIA "banh mi" v2023 board does not provide battery voltage sensing
- **Limitation**: No built-in low battery detection without additional circuitry

## The Critical Frequency Constraint

### The Fundamental Problem
- **Hardware Limitation**: ALL 16 channels must operate at identical frequency
- **Motor Preference**: ~1600Hz for responsive, smooth control
- **Servo Requirement**: 50Hz mandatory for reliable operation (20ms period)
- **Impossible Optimization**: Cannot satisfy both requirements simultaneously

### Frequency Selection Strategy
**Priority Order:**
1. **Servo Reliability**: Servos REQUIRE 50Hz - higher frequencies cause damage
2. **Motor Tolerance**: Motors can operate at 50Hz with acceptable performance loss
3. **Safety First**: Servo damage from wrong frequency is permanent and expensive

### Recommended Configuration
```cpp
// In src/config.h
namespace pwm {
    constexpr int DEFAULT_FREQ = 50; // Prioritize servo reliability over motor response
}
```

## Channel Allocation

### Motor Channels (8-15) - TA6586 Control
- **Channels 8-9**: Drive Left Motor (TA6586 #1)
  - Channel 8: Drive Left Forward (CHAN_DRIVE_L_FWD)
  - Channel 9: Drive Left Reverse (CHAN_DRIVE_L_REV)
- **Channels 10-11**: Drive Right Motor (TA6586 #2)
  - Channel 10: Drive Right Forward (CHAN_DRIVE_R_FWD)
  - Channel 11: Drive Right Reverse (CHAN_DRIVE_R_REV)
- **Channels 12-13**: Outtake Left Motor (TA6586 #3)
  - Channel 12: Outtake Left Forward (CHAN_OUTTAKE_L_FWD)
  - Channel 13: Outtake Left Reverse (CHAN_OUTTAKE_L_REV)
- **Channels 14-15**: Outtake Right Motor (TA6586 #4)
  - Channel 14: Outtake Right Forward (CHAN_OUTTAKE_R_FWD)
  - Channel 15: Outtake Right Reverse (CHAN_OUTTAKE_R_REV)

### Servo Channels (2-7) - Direct PCA9685
- **Channel 2**: Outtake Servo (OUTTAKE_SERVO_CHANNEL)
- **Channel 3**: Intake Servo (INTAKE_SERVO_CHANNEL)
- **Channels 4-7**: Reserved for future servo expansion

### Reserved Channels
- **Channels 0-1**: Not accessible on VIA power board

## Communication Architecture

### System Connections
```
ESP32 GPIO Pins:
├── Direct GPIO (IO32, IO2, IO0, IO25, IO39, IO36) → Power Board IO Headers
├── I2C (GPIO 21, 22) → PCA9685 → PWM Channels 0-15 → TA6586 ICs → Motors
└── SPI (GPIO 12, 13, 14, 15) → PS2 Controller (separate system)
```

### Pin Conflict Resolution
**IMPORTANT: No conflicts between systems**
- **PS2 Controller**: Uses ESP32 GPIO 12,13,14,15 for SPI communication
- **Motor Channels**: Uses PCA9685 PWM channels 12,13,14,15 (internal to PCA9685)
- **IO Ports**: Uses direct ESP32 GPIO pins (IO32, IO2, IO0, IO25, IO39, IO36)
- **Different Systems**: SPI vs I2C vs Direct GPIO - no interference

## Control Characteristics at 50Hz

### Motor Control Impact
- **Response Time**: Slightly reduced compared to higher frequencies (~1600Hz)
- **Smoothness**: Adequate for most robotic applications
- **Mitigation**: Software ramping and acceleration curves compensate for frequency limitation
- **Benefit**: Unified frequency eliminates servo compatibility issues

### Servo Control Requirements
- **Frequency**: 50Hz (20ms period) - ABSOLUTELY MANDATORY
- **Pulse Width Range**:
  - Standard: 1000-2000μs (1-2ms)
  - Extended: 500-2500μs (0.5-2.5ms) for increased range
- **Neutral Position**: ~1500μs (1.5ms) pulse width
- **Rotation Mapping**: 1ms = full CCW, 2ms = full CW (standard servos)

## Power Supply Design

### Voltage Requirements
- **ESP32 Logic**: 3.3V (provided by VIA board)
- **PCA9685 Logic**: 5V (required for proper operation)
- **Servo Power**: 5V (current varies: 0.5-3A per servo depending on size/load)
- **Motor Power**: 6-12V typical (high current: 5-20A total depending on motors)

### Current Considerations
- **PCA9685 Output**: 25mA max per channel (LED driving only)
- **External Drivers**: Required for all motor and servo loads
- **Power Supply Sizing**: Calculate total current for all connected devices
- **Isolation**: Consider separate supplies for logic and power to reduce noise

## Implementation Best Practices

### Frequency Configuration
1. **Always use 50Hz** for mixed motor/servo applications - no exceptions
2. **Document frequency changes** and test all connected devices after modifications
3. **Monitor servo temperature** - overheating indicates frequency problems
4. **Never exceed 50Hz** when servos are connected - damage can occur

### Software Optimization
1. **Pre-calculate PWM values** to reduce real-time computation overhead
2. **Implement software ramping** for smooth motor acceleration/deceleration at 50Hz
3. **Use proper debouncing** for limit switches and sensor inputs
4. **Add performance monitoring** for debugging and optimization
5. **Batch I2C operations** to minimize communication overhead

### Hardware Setup
1. **Power supply sizing**: Calculate total current draw for all motors and servos
2. **Wire gauge selection**: Use appropriate wire gauge for motor current loads
3. **Bypass capacitors**: Add near PCA9685 for noise reduction and stability
4. **Heat management**: Consider heat sinking for high-current driver applications
5. **Ground loops**: Maintain proper grounding between power supplies

## Troubleshooting Guide

### Servo Problems
- **Jittering/Twitching**: Frequency too high (check 50Hz setting)
- **No movement**: Verify 5V power supply and pulse width range (1-2ms)
- **Overheating**: Wrong frequency or mechanical binding
- **Erratic behavior**: Check I2C connections and power supply stability

### Motor Problems
- **Sluggish response**: Expected at 50Hz - implement software ramping
- **Jerky movement**: Add acceleration/deceleration curves in software
- **Overheating**: Check current limits, duty cycle, and heat dissipation
- **Inconsistent speed**: Verify power supply capacity and voltage regulation

## Code Implementation Examples

### PCA9685 Initialization
```cpp
void initPCA9685() {
    Wire.begin(21, 22); // SDA=21, SCL=22 (VIA board defaults)
    pwm.begin();
    pwm.setOscillatorFrequency(27000000); // Internal oscillator frequency
    pwm.setPWMFreq(50); // CRITICAL: 50Hz for servo compatibility
    Wire.setClock(400000L); // Fast I2C (400kHz)
    Serial.println("PCA9685 initialized at 50Hz");
}
```

### Servo Control Implementation
```cpp
void setServoAngle(uint8_t channel, int angle) {
    // Constrain angle to safe range
    angle = constrain(angle, 0, 180);

    // Map angle to pulse width (extended range for better control)
    int pulse_us = map(angle, 0, 180, 500, 2500); // 0.5-2.5ms range

    // Convert to PWM value (12-bit resolution, 20ms period at 50Hz)
    int pwm_value = (pulse_us * 4095L) / 20000L;

    // Set PWM output
    pwm.setPWM(channel, 0, pwm_value);
}
```

### Motor Control with Software Ramping
```cpp
class Motor {
private:
    int currentSpeed = 0;
    int targetSpeed = 0;
    unsigned long lastUpdate = 0;

public:
    void setSpeed(int speed) {
        targetSpeed = constrain(speed, -4095, 4095);
    }

    void update() {
        unsigned long now = millis();
        if (now - lastUpdate >= SLEW_RATE_TIME_MS / 100) { // Update every 2ms
            currentSpeed = rampTowards(currentSpeed, targetSpeed);
            updatePWM();
            lastUpdate = now;
        }
    }

private:
    int rampTowards(int current, int target) {
        int step = SLEW_INCREMENT; // Pre-calculated step size
        int diff = target - current;
        if (abs(diff) <= step) return target;
        return current + (diff > 0 ? step : -step);
    }
};
```

## Pin Conflict Clarification

**IMPORTANT: No conflicts between PS2 controller and PCA9685 motor channels**

Many users notice that PS2 controller pins and PCA9685 motor channels use the same numbers (12, 13, 14, 15) and worry about conflicts. **There are NO conflicts** because these are completely separate systems:

### PS2 Controller (SPI Communication)
- **Uses ESP32 GPIO pins**: 12, 13, 14, 15
- **Protocol**: SPI (direct pin-to-pin communication)
- **Connection**: PS2 controller → ESP32 GPIO pins

### PCA9685 Motors (I2C Communication)
- **Uses ESP32 pins**: Only SDA (21) and SCL (22) for I2C
- **Protocol**: I2C (2-wire bus controls all 16 channels)
- **Connection**: ESP32 I2C → PCA9685 chip → PWM channels 0-15

### Why No Conflict
- **PS2 pins 12,13,14,15** = ESP32 GPIO pins for SPI
- **Motor channels 12,13,14,15** = PCA9685 PWM output channels
- **Different systems**: SPI vs I2C, different physical pins
- **PCA9685 channels** are internal to the PCA9685 chip, not ESP32 pins

**Communication Flow:**
```
PS2 Controller → [SPI: GPIO 12,13,14,15] → ESP32 → [I2C: GPIO 21,22] → PCA9685 → [PWM channels 0-15] → Motors/Servos
```

## TA6586 Motor Control Patterns

### Standard Motor Control
```cpp
// Forward motion (positive PWM)
hal_pca9685_set_pwm(fwd_channel, pwm_value);
hal_pca9685_set_pwm(rev_channel, 0);

// Reverse motion (negative PWM)
hal_pca9685_set_pwm(fwd_channel, 0);
hal_pca9685_set_pwm(rev_channel, abs(pwm_value));

// Stop/Coast
hal_pca9685_set_pwm(fwd_channel, 0);
hal_pca9685_set_pwm(rev_channel, 0);

// Electromagnetic Braking (TA6586 Slow Decay)
hal_pca9685_set_pwm(fwd_channel, brake_power);
hal_pca9685_set_pwm(rev_channel, brake_power);
// Both inputs HIGH → Both low-side FETs ON → Slow decay braking
```

### TA6586 Electromagnetic Braking Mechanism
When both H-bridge inputs are HIGH (both channels receive same PWM value):
- **Both low-side FETs turn ON**
- **Both motor terminals connect to ground (separately)**
- **Motor back-EMF drives current through FET resistance**
- **Creates electromagnetic braking force opposing motion**
- **Energy dissipated as heat in FETs (not motor)**

### Braking Type: Slow Decay (NOT True Terminal Shorting)
- **Slow Decay**: Both terminals to ground through separate FET paths
- **NOT Terminal Shorting**: Terminals are not directly connected to each other
- **Current Path**: Motor → FET → Ground → FET → Motor (through ground)
- **Resistance**: Higher than true terminal shorting (includes FET resistance)

### Braking Characteristics
- **Type**: Slow decay electromagnetic braking (legitimate but not terminal shorting)
- **Speed-dependent**: More effective at higher motor speeds
- **Moderate force**: Less aggressive than true FTC terminal shorting, more than coasting
- **Safe operation**: No risk of direction reversal
- **Hardware-compatible**: Works with standard H-bridge configuration
- **Effectiveness**: ~60-70% of true terminal shorting performance

## Hardware Limitations

### Battery Voltage Monitoring
- **NOT AVAILABLE**: VIA "banh mi" v2023 board provides no battery voltage sensing
- **Power Path**: Battery → XT60 → Power Board → Motors/ESP32
- **ESP32 Access**: No direct connection to battery voltage
- **Workaround**: External voltage divider circuit required for monitoring
- **Safety Impact**: No automatic low battery detection without additional hardware

### Recommended Voltage Monitoring Solution
If battery monitoring is needed:
```cpp
// Add external voltage divider to GPIO pin
// Battery+ → 10kΩ → GPIO_PIN → 3.3kΩ → GND
// Provides ~4:1 voltage division for 12V battery monitoring
const int BATTERY_MONITOR_PIN = 36; // Use available GPIO
float voltage = (analogRead(BATTERY_MONITOR_PIN) / 4095.0) * 3.3 * 4.0;
```

## Technical References
- **PCA9685 Datasheet**: NXP 16-channel, 12-bit PWM Fm+ I2C-bus LED controller
- **TA6586 Datasheet**: 5A dual H-bridge motor driver IC
- **Servo Standards**: 50Hz frequency, 1-2ms pulse width (20ms period)
- **ESP32 I2C**: Default pins SDA=21, SCL=22 on VIA "banh mi" v2023
- **Motor Control**: H-bridge drivers required for bidirectional control
- **VIA Board**: No built-in battery voltage monitoring capability
- **Source**: https://cms.vsteam.edu.vn/mod/page/view.php?id=125 (TA6586 hardware confirmation)
