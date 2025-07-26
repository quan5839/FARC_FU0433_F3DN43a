#include "imu.h"
#include "../config.h"
#include <Arduino.h>

// Performance optimization pragmas for IMU processing
#pragma GCC optimize("O3")
#pragma GCC optimize("fast-math")

namespace IMU {
    // Static MPU6050 object
    static Adafruit_MPU6050 mpu6050;
    
    // Sensor event structures
    static sensors_event_t accel_event, gyro_event, temp_event;
    
    // Cached sensor readings
    static float accel_x = 0, accel_y = 0, accel_z = 0;
    static float gyro_x = 0, gyro_y = 0, gyro_z = 0;
    static float temperature = 0;
    
    // Heading calculation
    static float current_heading = 0;
    static float heading_offset = 0;
    static unsigned long last_update_time = 0;
    
    // Movement detection thresholds
    static const float MOVEMENT_THRESHOLD = 2.0; // m/s² above gravity
    static const float TURNING_THRESHOLD = 0.5;  // rad/s
    
    bool init() {
        DEBUG_PRINTLN("Initializing MPU6050 IMU...");

        // Set I2C speed for MPU6050 (400kHz for better reliability)
        Wire.setClock(config::imu::I2C_CLOCK_SPEED);
        DEBUG_PRINT("MPU6050 I2C speed set to ");
        DEBUG_PRINT(config::imu::I2C_CLOCK_SPEED / 1000);
        DEBUG_PRINTLN("kHz");

        // Initialize MPU6050 at 0x68 only
        #if !COMPETITION_MODE
        Serial.print("Initializing MPU6050 at address 0x");
        Serial.println(config::imu::I2C_ADDRESS, HEX);
        #endif

        if (!mpu6050.begin(config::imu::I2C_ADDRESS)) {
            #if !COMPETITION_MODE
            Serial.print("Failed to find MPU6050 at address 0x");
            Serial.println(config::imu::I2C_ADDRESS, HEX);
            Serial.println("Check wiring and run I2C scanner (L1 + SELECT)");
            #endif

            // Restore PCA9685 I2C speed after failed MPU6050 init
            Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);
            DEBUG_PRINT("I2C speed restored to ");
            DEBUG_PRINT(config::pca9685::I2C_CLOCK_SPEED / 1000);
            DEBUG_PRINTLN("kHz for PCA9685");

            return false;
        }

        #if !COMPETITION_MODE
        Serial.print("MPU6050 initialized successfully at 0x");
        Serial.println(config::imu::I2C_ADDRESS, HEX);
        #endif
        
        // Configure accelerometer range (8G for robot movements)
        mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
        DEBUG_PRINTLN("Accelerometer range set to 8G");
        
        // Configure gyroscope range (500°/s for robot turning)
        mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);
        DEBUG_PRINTLN("Gyroscope range set to 500°/s");
        
        // Configure low-pass filter (21Hz to reduce noise)
        mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);
        DEBUG_PRINTLN("Filter bandwidth set to 21Hz");
        
        // Initialize timing
        last_update_time = millis();
        
        DEBUG_PRINTLN("MPU6050 IMU initialized successfully");

        // Restore PCA9685 I2C speed after successful MPU6050 init
        Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);
        DEBUG_PRINT("I2C speed restored to ");
        DEBUG_PRINT(config::pca9685::I2C_CLOCK_SPEED / 1000);
        DEBUG_PRINTLN("kHz for PCA9685");

        return true;
    }
    
    bool update() {
        // Temporarily switch to MPU6050 I2C speed for reliable communication
        Wire.setClock(config::imu::I2C_CLOCK_SPEED);

        bool success = mpu6050.getEvent(&accel_event, &gyro_event, &temp_event);

        // Restore PCA9685 I2C speed immediately after MPU6050 operation
        Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);

        if (!success) {
            ERROR_PRINTLN("Failed to read MPU6050 data");
            return false;
        }
        
        // Cache accelerometer readings
        accel_x = accel_event.acceleration.x;
        accel_y = accel_event.acceleration.y;
        accel_z = accel_event.acceleration.z;
        
        // Cache gyroscope readings
        gyro_x = gyro_event.gyro.x;
        gyro_y = gyro_event.gyro.y;
        gyro_z = gyro_event.gyro.z;
        
        // Cache temperature
        temperature = temp_event.temperature;
        
        // Update heading calculation
        unsigned long current_time = millis();
        float dt = (current_time - last_update_time) / 1000.0; // Convert to seconds
        
        if (dt > 0) {
            // Integrate Z-axis gyroscope for heading (yaw)
            current_heading += gyro_z * dt * 57.2958; // Convert rad/s to deg/s
            
            // Keep heading in 0-360 range
            while (current_heading >= 360.0) current_heading -= 360.0;
            while (current_heading < 0.0) current_heading += 360.0;
        }
        
        last_update_time = current_time;
        return true;
    }
    
    void getAcceleration(float& x, float& y, float& z) {
        x = accel_x;
        y = accel_y;
        z = accel_z;
    }
    
    void getRotation(float& x, float& y, float& z) {
        x = gyro_x;
        y = gyro_y;
        z = gyro_z;
    }
    
    float getTemperature() {
        return temperature;
    }
    
    bool isConnected() {
        // Temporarily switch to MPU6050 I2C speed for connection test
        Wire.setClock(config::imu::I2C_CLOCK_SPEED);

        // Try to read a register to check if device is responding
        sensors_event_t test_accel, test_gyro, test_temp;
        bool connected = mpu6050.getEvent(&test_accel, &test_gyro, &test_temp);

        // Restore PCA9685 I2C speed
        Wire.setClock(config::pca9685::I2C_CLOCK_SPEED);

        return connected;
    }
    
    float getHeading() {
        return current_heading - heading_offset;
    }
    
    void resetHeading() {
        heading_offset = current_heading;
        DEBUG_PRINTLN("IMU heading reset to zero");
    }
    
    bool isMoving() {
        // Calculate total acceleration magnitude
        float total_accel = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
        
        // Check if acceleration is significantly different from gravity (9.8 m/s²)
        return abs(total_accel - 9.8) > MOVEMENT_THRESHOLD;
    }
    
    bool isTurning() {
        // Check if any rotational rate exceeds threshold
        return (abs(gyro_x) > TURNING_THRESHOLD || 
                abs(gyro_y) > TURNING_THRESHOLD || 
                abs(gyro_z) > TURNING_THRESHOLD);
    }
}
