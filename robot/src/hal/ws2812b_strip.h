#ifndef WS2812B_STRIP_H
#define WS2812B_STRIP_H

#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include "../config.h"

/**
 * @brief WS2812B LED strip control module
 * 
 * This module provides control for a WS2812B LED strip with:
 * - Data signal on GPIO 19 (ESP32 for precise timing)
 * - Power control via PCA9685 channel 7 (power switching)
 * Features include solid colors, rainbow effects, breathing patterns, and robot status indication.
 */
namespace WS2812BStrip {
    
    /**
     * @brief Initialize the WS2812B LED strip
     * @return true if initialization successful
     */
    bool init();
    
    /**
     * @brief Update LED strip (call in main loop)
     */
    void update();
    
    /**
     * @brief Enable LED strip power via PCA9685 channel 7
     */
    void powerOn();

    /**
     * @brief Disable LED strip power via PCA9685 channel 7
     */
    void powerOff();

    /**
     * @brief Check if LED strip power is enabled
     * @return true if power is on
     */
    bool isPowerOn();
    
    /**
     * @brief Set all LEDs to a solid color
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    void setSolidColor(uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * @brief Set all LEDs to a solid color using HSV
     * @param hue Hue (0-255)
     * @param sat Saturation (0-255)
     * @param val Value/brightness (0-255)
     */
    void setSolidColorHSV(uint8_t hue, uint8_t sat, uint8_t val);
    
    /**
     * @brief Turn off all LEDs
     */
    void clear();
    
    /**
     * @brief Set rainbow effect
     * @param speed Animation speed (higher = faster)
     */
    void setRainbow(uint8_t speed = 5);
    
    /**
     * @brief Set breathing effect
     * @param hue Color hue (0-255)
     * @param speed Breathing speed (higher = faster)
     */
    void setBreathing(uint8_t hue, uint8_t speed = 3);
    
    /**
     * @brief Set robot status indication
     * @param status Robot status using config::led_status constants
     */
    void setRobotStatus(uint8_t status);

    /**
     * @brief Start startup sequence animation
     */
    void startupSequence();
    
    /**
     * @brief Set LED strip brightness
     * @param brightness Global brightness (0-255)
     */
    void setBrightness(uint8_t brightness);
    
    /**
     * @brief Get current brightness
     * @return Current brightness (0-255)
     */
    uint8_t getBrightness();
    
    /**
     * @brief Set individual LED color
     * @param index LED index (0 to NUM_LEDS-1)
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     */
    void setLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
    
    /**
     * @brief Set individual LED color using HSV
     * @param index LED index (0 to NUM_LEDS-1)
     * @param hue Hue (0-255)
     * @param sat Saturation (0-255)
     * @param val Value/brightness (0-255)
     */
    void setLEDHSV(uint16_t index, uint8_t hue, uint8_t sat, uint8_t val);
    
    /**
     * @brief Force immediate update of LED strip
     */
    void show();
    
    /**
     * @brief Check if strip is ready for next update
     * @return true if ready for update
     */
    bool isReady();
    
    // Animation modes
    enum AnimationMode {
        SOLID_COLOR,
        RAINBOW,
        BREATHING,
        STATUS_INDICATION,
        CUSTOM
    };
    
    /**
     * @brief Set animation mode
     * @param mode Animation mode to set
     */
    void setAnimationMode(AnimationMode mode);
    
    /**
     * @brief Get current animation mode
     * @return Current animation mode
     */
    AnimationMode getAnimationMode();

    /**
     * @brief Direct hardware test (bypasses power control)
     * Tests if LED strip responds to data signal
     */
    void directHardwareTest();
}

#endif // WS2812B_STRIP_H
