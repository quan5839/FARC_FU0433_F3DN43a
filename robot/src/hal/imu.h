#ifndef IMU_H
#define IMU_H

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "../config.h"

/**
 * @file imu.h
 * @brief MPU6050 IMU (Inertial Measurement Unit) Interface
 * 
 * Provides accelerometer, gyroscope, and temperature readings from MPU6050.
 * Optimized for competition robotics with performance-tuned settings.
 * 
 * Features:
 * - 8G accelerometer range (good for robot movements)
 * - 500°/s gyroscope range (suitable for robot turning)
 * - 21Hz low-pass filter (reduces noise)
 * - Alternative I2C address support (0x69)
 */

namespace IMU {
    /**
     * @brief Initialize MPU6050 IMU sensor
     * @return true if initialization successful, false otherwise
     */
    bool init();
    
    /**
     * @brief Update IMU readings (call regularly in main loop)
     * @return true if reading successful, false otherwise
     */
    bool update();
    
    /**
     * @brief Get current accelerometer readings
     * @param x X-axis acceleration (m/s²)
     * @param y Y-axis acceleration (m/s²) 
     * @param z Z-axis acceleration (m/s²)
     */
    void getAcceleration(float& x, float& y, float& z);
    
    /**
     * @brief Get current gyroscope readings
     * @param x X-axis rotation rate (rad/s)
     * @param y Y-axis rotation rate (rad/s)
     * @param z Z-axis rotation rate (rad/s)
     */
    void getRotation(float& x, float& y, float& z);
    
    /**
     * @brief Get current temperature
     * @return Temperature in Celsius
     */
    float getTemperature();
    
    /**
     * @brief Check if IMU is connected and responding
     * @return true if IMU is healthy, false otherwise
     */
    bool isConnected();
    
    /**
     * @brief Get robot heading (yaw angle)
     * @return Heading in degrees (0-360)
     */
    float getHeading();
    
    /**
     * @brief Reset heading to zero (calibrate current direction as forward)
     */
    void resetHeading();
    
    /**
     * @brief Detect if robot is moving (based on acceleration)
     * @return true if significant movement detected
     */
    bool isMoving();
    
    /**
     * @brief Detect if robot is turning (based on gyroscope)
     * @return true if significant rotation detected
     */
    bool isTurning();
}

#endif // IMU_H
