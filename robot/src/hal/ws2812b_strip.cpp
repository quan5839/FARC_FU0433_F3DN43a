#include "ws2812b_strip.h"
#include "pca9685_driver.h"

// Performance optimization pragmas
#pragma GCC optimize("O3")
#pragma GCC optimize("fast-math")

namespace WS2812BStrip {
    
    // LED array and control variables
    static CRGB leds[config::WS2812B_NUM_LEDS];
    static AnimationMode current_mode = SOLID_COLOR;
    static unsigned long last_update = 0;
    static uint8_t animation_counter = 0;
    static bool initialized = false;
    static bool power_enabled = false;
    
    // Animation state variables
    static uint8_t solid_r = 0, solid_g = 0, solid_b = 0;
    static uint8_t rainbow_speed = 5;
    static uint8_t breathing_hue = config::tuning::WS2812B_DEFAULT_HUE;
    static uint8_t breathing_speed = 3;
    static uint8_t robot_status = 0;
    
    // Forward declarations for private functions
    void updateRainbow();
    void updateBreathing();
    void updateStatusIndication();
    
    bool init() {
        if (initialized) {
            return true;
        }
        
        // Initialize FastLED with WS2812B strip on GPIO 19
        FastLED.addLeds<WS2812B, config::WS2812B_DATA_PIN, GRB>(leds, config::WS2812B_NUM_LEDS);
        
        // Set global brightness and power limit for safety
        FastLED.setBrightness(config::tuning::WS2812B_BRIGHTNESS);
        FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000); // 5V, 2A max
        
        // Clear all LEDs initially
        clear();

        // Power off initially
        powerOff();
        
        initialized = true;
        DEBUG_PRINTLN("WS2812B LED strip initialized on GPIO 19");
        return true;
    }
    
    void update() {
        if (!initialized || !power_enabled) {
            return;
        }
        
        // Check if it's time to update
        unsigned long current_time = millis();
        if (current_time - last_update < config::tuning::WS2812B_UPDATE_INTERVAL_MS) {
            return;
        }
        
        last_update = current_time;
        animation_counter++;
        
        // Update based on current animation mode
        switch (current_mode) {
            case SOLID_COLOR:
                // No animation needed for solid color
                break;
                
            case RAINBOW:
                updateRainbow();
                break;
                
            case BREATHING:
                updateBreathing();
                break;
                
            case STATUS_INDICATION:
                updateStatusIndication();
                break;
                
            case CUSTOM:
                // Custom mode - no automatic updates
                break;
        }
        
        // Update the LED strip
        FastLED.show();
    }
    
    void powerOn() {
        hal_pca9685_set_pwm(config::WS2812B_POWER_CHANNEL, config::tuning::WS2812B_POWER_ON_PWM);
        power_enabled = true;
        DEBUG_PRINT("WS2812B power enabled via PCA9685 channel ");
        DEBUG_PRINT(config::WS2812B_POWER_CHANNEL);
        DEBUG_PRINT(" with PWM value ");
        DEBUG_PRINTLN(config::tuning::WS2812B_POWER_ON_PWM);
    }

    void powerOff() {
        hal_pca9685_set_pwm(config::WS2812B_POWER_CHANNEL, config::tuning::WS2812B_POWER_OFF_PWM);
        power_enabled = false;
        // Clear LEDs when power is off
        clear();
        if (initialized) {
            FastLED.show();
        }
        DEBUG_PRINTLN("WS2812B power disabled via PCA9685 channel 7");
    }

    bool isPowerOn() {
        return power_enabled;
    }
    
    void setSolidColor(uint8_t r, uint8_t g, uint8_t b) {
        if (!initialized) {
            DEBUG_PRINTLN("setSolidColor: Not initialized!");
            return;
        }

        DEBUG_PRINT("Setting solid color RGB(");
        DEBUG_PRINT(r);
        DEBUG_PRINT(",");
        DEBUG_PRINT(g);
        DEBUG_PRINT(",");
        DEBUG_PRINT(b);
        DEBUG_PRINT(") for ");
        DEBUG_PRINT(config::WS2812B_NUM_LEDS);
        DEBUG_PRINTLN(" LEDs");

        solid_r = r;
        solid_g = g;
        solid_b = b;
        current_mode = SOLID_COLOR;

        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CRGB(r, g, b);
        }
    }
    
    void setSolidColorHSV(uint8_t hue, uint8_t sat, uint8_t val) {
        if (!initialized) return;
        
        current_mode = SOLID_COLOR;
        
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CHSV(hue, sat, val);
        }
        
        // Store RGB values for reference
        CRGB rgb = CHSV(hue, sat, val);
        solid_r = rgb.r;
        solid_g = rgb.g;
        solid_b = rgb.b;
    }
    
    void clear() {
        if (!initialized) return;
        
        current_mode = SOLID_COLOR;
        solid_r = solid_g = solid_b = 0;
        
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
        }
    }
    
    void setRainbow(uint8_t speed) {
        rainbow_speed = speed;
        current_mode = RAINBOW;
    }
    
    void setBreathing(uint8_t hue, uint8_t speed) {
        breathing_hue = hue;
        breathing_speed = speed;
        current_mode = BREATHING;
    }
    
    void setRobotStatus(uint8_t status) {
        robot_status = status;
        current_mode = STATUS_INDICATION;
    }
    
    void setBrightness(uint8_t brightness) {
        if (initialized) {
            FastLED.setBrightness(brightness);
        }
    }
    
    uint8_t getBrightness() {
        return initialized ? FastLED.getBrightness() : 0;
    }
    
    void setLED(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
        if (!initialized || index >= config::WS2812B_NUM_LEDS) {
            return;
        }
        
        leds[index] = CRGB(r, g, b);
        current_mode = CUSTOM;
    }
    
    void setLEDHSV(uint16_t index, uint8_t hue, uint8_t sat, uint8_t val) {
        if (!initialized || index >= config::WS2812B_NUM_LEDS) {
            return;
        }
        
        leds[index] = CHSV(hue, sat, val);
        current_mode = CUSTOM;
    }
    
    void show() {
        DEBUG_PRINT("show() called - initialized: ");
        DEBUG_PRINT(initialized ? "YES" : "NO");
        DEBUG_PRINT(", power_enabled: ");
        DEBUG_PRINTLN(power_enabled ? "YES" : "NO");

        if (initialized && power_enabled) {
            DEBUG_PRINTLN("Calling FastLED.show()");
            FastLED.show();
        } else {
            DEBUG_PRINTLN("Skipping FastLED.show() - not ready");
        }
    }
    
    bool isReady() {
        return (millis() - last_update) >= config::tuning::WS2812B_UPDATE_INTERVAL_MS;
    }
    
    void setAnimationMode(AnimationMode mode) {
        current_mode = mode;
    }
    
    AnimationMode getAnimationMode() {
        return current_mode;
    }
    
    // Private animation update functions
    void updateRainbow() {
        uint8_t hue_offset = animation_counter * rainbow_speed;
        
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            uint8_t hue = hue_offset + (i * 255 / config::WS2812B_NUM_LEDS);
            leds[i] = CHSV(hue, 255, config::tuning::WS2812B_DEFAULT_VAL);
        }
    }
    
    void updateBreathing() {
        // Create breathing effect using sine wave
        uint8_t breath = beatsin8(breathing_speed * 2, 0, config::tuning::WS2812B_DEFAULT_VAL);
        
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CHSV(breathing_hue, 255, breath);
        }
    }
    
    void updateStatusIndication() {
        switch (robot_status) {
            case 0: // Off
                clear();
                break;
                
            case 1: // Idle - slow blue breathing
                breathing_hue = 160; // Blue
                breathing_speed = 1; // Slow
                updateBreathing();
                break;
                
            case 2: // Manual control - solid green
                setSolidColorHSV(120, 255, config::tuning::WS2812B_DEFAULT_VAL); // Green
                break;
                
            case 3: // Automatic mode - rainbow
                rainbow_speed = 3;
                updateRainbow();
                break;
                
            case 4: // Error - fast red blinking
                if ((animation_counter / 5) % 2) {
                    setSolidColorHSV(0, 255, config::tuning::WS2812B_DEFAULT_VAL); // Red
                } else {
                    clear();
                }
                break;
                
            default:
                clear();
                break;
        }
    }

    void directHardwareTest() {
        DEBUG_PRINTLN("=== DIRECT LED STRIP HARDWARE TEST ===");
        DEBUG_PRINTLN("This test bypasses power control to test LED strip directly");

        if (!initialized) {
            DEBUG_PRINTLN("Initializing FastLED...");
            FastLED.addLeds<WS2812B, config::WS2812B_DATA_PIN, GRB>(leds, config::WS2812B_NUM_LEDS);
            FastLED.setBrightness(255); // Full brightness for test
            initialized = true;
        }

        DEBUG_PRINTLN("Testing first 10 LEDs with different colors...");

        // Clear all LEDs first
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
        }
        FastLED.show();
        delay(500);

        // Test first 10 LEDs with different colors
        for (int i = 0; i < 10 && i < config::WS2812B_NUM_LEDS; i++) {
            // Clear previous
            if (i > 0) leds[i-1] = CRGB::Black;

            // Set current LED color based on position
            switch (i % 3) {
                case 0: leds[i] = CRGB::Red; break;
                case 1: leds[i] = CRGB::Green; break;
                case 2: leds[i] = CRGB::Blue; break;
            }

            DEBUG_PRINT("LED ");
            DEBUG_PRINT(i);
            DEBUG_PRINT(" = ");
            switch (i % 3) {
                case 0: DEBUG_PRINTLN("RED"); break;
                case 1: DEBUG_PRINTLN("GREEN"); break;
                case 2: DEBUG_PRINTLN("BLUE"); break;
            }

            FastLED.show();
            delay(500);
        }

        // Clear all LEDs
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
        }
        FastLED.show();

        DEBUG_PRINTLN("Direct hardware test complete");
        DEBUG_PRINTLN("If no LEDs lit up, check:");
        DEBUG_PRINTLN("1. Power supply (5V, sufficient current)");
        DEBUG_PRINTLN("2. Data connection (GPIO 19 to LED strip DIN)");
        DEBUG_PRINTLN("3. Ground connection");
        DEBUG_PRINTLN("4. LED strip type (should be WS2812B)");
    }
}
