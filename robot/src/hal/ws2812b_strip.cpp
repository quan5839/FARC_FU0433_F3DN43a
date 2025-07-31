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
    static uint8_t rainbow_speed = config::tuning::LED_SPEED_FAST;
    static uint8_t breathing_hue = config::tuning::WS2812B_DEFAULT_HUE;
    static uint8_t breathing_speed = config::tuning::LED_SPEED_NORMAL;
    static uint8_t robot_status = 0;
    static uint8_t previous_status = 0;

    // Transition state variables
    static bool in_transition = false;
    static unsigned long transition_start_time = 0;
    static uint8_t transition_progress = 0;
    
    // Forward declarations for private functions
    void updateRainbow();
    void updateBreathing();
    void updateStatusIndication();
    void updateStartupSequence();
    void updatePulse();
    void updateTransition();
    void startTransition(uint8_t new_status);
    uint8_t blendColors(uint8_t color1, uint8_t color2, uint8_t blend_amount);
    
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
        DEBUG_PRINT("WS2812B LED strip initialized on GPIO ");
        DEBUG_PRINTLN(config::WS2812B_DATA_PIN);
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
        
        // Handle transitions first
        if (in_transition) {
            updateTransition();
        } else {
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
        if (status != robot_status) {
            // Start transition if status changed
            startTransition(status);
        }
        robot_status = status;
        current_mode = STATUS_INDICATION;
    }

    void startupSequence() {
        if (!initialized) return;

        DEBUG_PRINTLN("Starting LED startup sequence");
        setRobotStatus(config::led_status::STARTUP_SEQUENCE);

        // Run startup sequence for a few seconds
        unsigned long start_time = millis();
        while ((millis() - start_time) < config::tuning::STARTUP_SEQUENCE_DURATION_MS) {
            update();
            delay(config::tuning::STARTUP_FRAME_DELAY_MS);
        }

        DEBUG_PRINTLN("LED startup sequence complete");
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
            uint8_t hue = hue_offset + (i * config::tuning::WS2812B_DEFAULT_SAT / config::WS2812B_NUM_LEDS);
            leds[i] = CHSV(hue, config::tuning::WS2812B_DEFAULT_SAT, config::tuning::WS2812B_DEFAULT_VAL);
        }
    }
    
    void updateBreathing() {
        // Enhanced breathing effect with smoother sine wave and subtle variations
        uint8_t base_breath = beatsin8(breathing_speed * config::tuning::LED_BREATHING_MULTIPLIER, 0, config::tuning::WS2812B_DEFAULT_VAL);

        // Add subtle wave effect across the strip for more visual interest
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            // Create a slight phase offset for each LED to create a wave effect
            uint8_t phase_offset = (i * 4) % 255;
            uint8_t wave_breath = beatsin8(breathing_speed * config::tuning::LED_BREATHING_MULTIPLIER, 0, config::tuning::WS2812B_DEFAULT_VAL, 0, phase_offset);

            // Blend the base breathing with the wave effect (90% base, 10% wave)
            uint8_t final_breath = ((base_breath * 9) + wave_breath) / 10;

            leds[i] = CHSV(breathing_hue, config::tuning::WS2812B_DEFAULT_SAT, final_breath);
        }
    }
    
    void updateStatusIndication() {
        switch (robot_status) {
            case config::led_status::OFF:
                clear();
                break;

            case config::led_status::IDLE:
                breathing_hue = config::tuning::LED_HUE_BLUE;
                breathing_speed = config::tuning::LED_SPEED_SLOW;
                updateBreathing();
                break;

            case config::led_status::MANUAL_CONTROL:
                setSolidColorHSV(config::tuning::LED_HUE_GREEN, config::tuning::WS2812B_DEFAULT_SAT, config::tuning::WS2812B_DEFAULT_VAL);
                break;

            case config::led_status::AUTOMATIC_MODE:
                rainbow_speed = config::tuning::LED_SPEED_NORMAL;
                updateRainbow();
                break;

            case config::led_status::SYSTEM_ERROR:
                // Fast red blinking
                if ((animation_counter / config::tuning::LED_BLINK_FAST_DIVISOR) % 2) {
                    setSolidColorHSV(config::tuning::LED_HUE_RED, config::tuning::WS2812B_DEFAULT_SAT, config::tuning::WS2812B_DEFAULT_VAL);
                } else {
                    clear();
                }
                break;

            case config::led_status::CONTROLLER_SAFETY:
                // Orange/yellow warning - slow pulse
                breathing_hue = config::tuning::LED_HUE_ORANGE;
                breathing_speed = config::tuning::LED_SPEED_MEDIUM;
                updateBreathing();
                break;

            case config::led_status::CONTROLLER_DISCONNECTED:
                // Slow red breathing
                breathing_hue = config::tuning::LED_HUE_RED;
                breathing_speed = config::tuning::LED_SPEED_SLOW;
                updateBreathing();
                break;

            case config::led_status::TEMPERATURE_WARNING:
                // Yellow pulse
                breathing_hue = config::tuning::LED_HUE_YELLOW;
                breathing_speed = config::tuning::LED_SPEED_NORMAL;
                updateBreathing();
                break;

            case config::led_status::STARTUP_SEQUENCE:
                // Rainbow sweep
                updateStartupSequence();
                break;

            case config::led_status::I2C_ERROR:
                // Purple blink
                if ((animation_counter / config::tuning::LED_BLINK_SLOW_DIVISOR) % 2) {
                    setSolidColorHSV(config::tuning::LED_HUE_PURPLE, config::tuning::WS2812B_DEFAULT_SAT, config::tuning::WS2812B_DEFAULT_VAL);
                } else {
                    clear();
                }
                break;

            case config::led_status::LIMIT_SWITCH_ACTIVE:
                // Cyan solid
                setSolidColorHSV(config::tuning::LED_HUE_CYAN, config::tuning::WS2812B_DEFAULT_SAT, config::tuning::WS2812B_DEFAULT_VAL);
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
            FastLED.setBrightness(config::tuning::LED_TEST_BRIGHTNESS);
            initialized = true;
        }

        DEBUG_PRINTLN("Testing first LEDs with different colors...");

        // Clear all LEDs first
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
        }
        FastLED.show();
        delay(config::tuning::LED_TEST_DELAY_MS);

        // Test first LEDs with different colors
        for (int i = 0; i < config::tuning::LED_TEST_COUNT && i < config::WS2812B_NUM_LEDS; i++) {
            // Clear previous
            if (i > 0) leds[i-1] = CRGB::Black;

            // Set current LED color based on position
            switch (i % config::tuning::LED_TEST_COLOR_CYCLE) {
                case 0: leds[i] = CRGB::Red; break;
                case 1: leds[i] = CRGB::Green; break;
                case 2: leds[i] = CRGB::Blue; break;
            }

            DEBUG_PRINT("LED ");
            DEBUG_PRINT(i);
            DEBUG_PRINT(" = ");
            switch (i % config::tuning::LED_TEST_COLOR_CYCLE) {
                case 0: DEBUG_PRINTLN("RED"); break;
                case 1: DEBUG_PRINTLN("GREEN"); break;
                case 2: DEBUG_PRINTLN("BLUE"); break;
            }

            FastLED.show();
            delay(config::tuning::LED_TEST_DELAY_MS);
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

    void updateStartupSequence() {
        // Rainbow sweep effect for startup
        uint8_t sweep_position = (animation_counter * config::tuning::STARTUP_SWEEP_SPEED) % (config::WS2812B_NUM_LEDS + config::tuning::STARTUP_SWEEP_EXTRA_RANGE);

        // Clear all LEDs first
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CRGB::Black;
        }

        // Create a moving rainbow segment
        for (int i = 0; i < config::tuning::STARTUP_SWEEP_TAIL_LENGTH && (sweep_position + i) < config::WS2812B_NUM_LEDS; i++) {
            if ((sweep_position + i) >= 0) {
                uint8_t hue = (animation_counter * config::tuning::STARTUP_HUE_SPEED + i * config::tuning::STARTUP_HUE_STEP) % config::tuning::WS2812B_DEFAULT_SAT;
                leds[sweep_position + i] = CHSV(hue, config::tuning::WS2812B_DEFAULT_SAT, config::tuning::WS2812B_DEFAULT_VAL);
            }
        }
    }

    void updatePulse() {
        // Enhanced pulse effect with smoother transitions
        uint8_t pulse_value = beatsin8(breathing_speed * config::tuning::LED_PULSE_MULTIPLIER, 0, config::tuning::WS2812B_DEFAULT_VAL);

        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            leds[i] = CHSV(breathing_hue, config::tuning::WS2812B_DEFAULT_SAT, pulse_value);
        }
    }

    void startTransition(uint8_t new_status) {
        // Only start transition if not already in one and status actually changed
        if (!in_transition && new_status != previous_status) {
            previous_status = robot_status;
            in_transition = true;
            transition_start_time = millis();
            transition_progress = 0;
            DEBUG_PRINT("LED transition started from status ");
            DEBUG_PRINT(previous_status);
            DEBUG_PRINT(" to ");
            DEBUG_PRINTLN(new_status);
        }
    }

    void updateTransition() {
        if (!in_transition) return;

        unsigned long elapsed = millis() - transition_start_time;

        if (elapsed >= config::tuning::LED_TRANSITION_DURATION_MS) {
            // Transition complete
            in_transition = false;
            transition_progress = 255;
            DEBUG_PRINTLN("LED transition complete");
        } else {
            // Calculate transition progress (0-255)
            transition_progress = (elapsed * 255) / config::tuning::LED_TRANSITION_DURATION_MS;
        }

        // Apply transition effect (fade between states)
        // For now, just use a simple fade - could be enhanced with more complex effects
        uint8_t fade_amount = transition_progress;

        // Blend between previous and current status colors
        // This is a simplified version - full implementation would blend actual colors
        for (int i = 0; i < config::WS2812B_NUM_LEDS; i++) {
            // Simple fade effect during transition
            uint8_t brightness = blendColors(0, config::tuning::WS2812B_DEFAULT_VAL, fade_amount);
            leds[i] = CHSV(breathing_hue, config::tuning::WS2812B_DEFAULT_SAT, brightness);
        }
    }

    uint8_t blendColors(uint8_t color1, uint8_t color2, uint8_t blend_amount) {
        // Linear interpolation between two color values
        return ((color1 * (255 - blend_amount)) + (color2 * blend_amount)) / 255;
    }
}
