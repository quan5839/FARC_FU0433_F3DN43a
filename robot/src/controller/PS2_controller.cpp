#include "PS2_controller.h"
#include "../config.h"
// Note: PS2X_lib.h is included via PS2_controller.h

// Forward declarations
void checkRepetitiveInput();

// ───── GLOBALS ─────────────────────────────────────────
PS2X ps2;
static ControllerState controllerState __attribute__((aligned(4))); // Cache-aligned for performance
static unsigned long last_read_time = 0;

// Button debouncers for commands that need single-trigger behavior
static ButtonDebouncer r2Debouncer;
static ButtonDebouncer circleDebouncer;
static ButtonDebouncer selectDebouncer;
static ButtonDebouncer startDebouncer;
static ButtonDebouncer padRightDebouncer;
static ButtonDebouncer padDownDebouncer;
static ButtonDebouncer padUpDebouncer;

// Connection monitoring
static unsigned long last_connection_attempt = 0;
static bool was_connected = false;

// Repetitive input monitoring for safety
static int last_left_y = 0;
static int last_right_x = 0;
static bool last_r1 = false;
static bool last_l1 = false;
static bool last_l2 = false;
static unsigned long repetitive_input_start_time = 0;
static bool repetitive_input_detected = false;

// ───── INIT PS2 PAD ────────────────────────────────────
void setupPS2() {
  Serial.begin(config::constants::SERIAL_BAUD_RATE);
  while (ps2.config_gamepad(config::ps2::CLK_PIN, config::ps2::CMD_PIN,
                            config::ps2::SEL_PIN, config::ps2::DATA_PIN,
                            true, true)) {
    // DEBUG_PRINTLN(F("PS2 retry…"));
    delay(config::constants::PS2_RETRY_DELAY_MS);
  }
  DEBUG_PRINTLN(F("PS2 ready"));
  last_read_time = millis();
}

// ───── CONTROL LOOP ────────────────────────────────────
void readController() {
    // Performance-optimized PS2 reading with branch prediction
    if (LIKELY(ps2.read_gamepad(false, 0))) {
        // Most common path: successful read
        last_read_time = millis();

        // Track connection state changes (uncommon)
        if (UNLIKELY(!was_connected)) {
            DEBUG_PRINTLN("PS2 controller reconnected!");
            was_connected = true;
        }

        // Read joystick values (convert from PS2's 0-255 range to -128 to +128 range)
        controllerState.left_joystick_y = 128 - ps2.Analog(PSS_LY);
        controllerState.right_joystick_x = ps2.Analog(PSS_RX) - 128;

        // Read immediate buttons (no debouncing - for continuous actions)
        controllerState.r1_pressed = ps2.Button(PSB_R1);
        controllerState.l1_pressed = ps2.Button(PSB_L1);
        controllerState.l2_pressed = ps2.Button(PSB_L2);

        // Read and debounce toggle/command buttons
        controllerState.r2_pressed = r2Debouncer.update(ps2.Button(PSB_R2));
        controllerState.circle_pressed = circleDebouncer.update(ps2.Button(PSB_CIRCLE));
        controllerState.select_pressed = selectDebouncer.update(ps2.Button(PSB_SELECT));
        controllerState.start_pressed = startDebouncer.update(ps2.Button(PSB_START));
        controllerState.pad_right_pressed = padRightDebouncer.update(ps2.Button(PSB_PAD_RIGHT));
        controllerState.pad_down_pressed = padDownDebouncer.update(ps2.Button(PSB_PAD_DOWN));
        controllerState.pad_up_pressed = padUpDebouncer.update(ps2.Button(PSB_PAD_UP));

        // Check for repetitive input (safety feature)
        checkRepetitiveInput();
    } else {
        // Failed read - track when connection was first lost
        if (was_connected) {
            was_connected = false;
            DEBUG_PRINTLN("PS2 controller connection lost!");
        }

        // Reset controller state on connection loss
        controllerState = {};

        // Attempt reconnection if enough time has passed
        unsigned long now = millis();
        if (now - last_connection_attempt > config::tuning::RECONNECTION_ATTEMPT_INTERVAL_MS) {
            DEBUG_PRINTLN("Attempting PS2 reconnection...");
            setupPS2();
            last_connection_attempt = now;
        }
    }
}

const ControllerState& getControllerState() {
    return controllerState;
}

bool isConnected() {
    return (millis() - last_read_time) < config::tuning::CONNECTION_TIMEOUT_MS;
}

// ───── REPETITIVE INPUT SAFETY ─────────────────────────────
void checkRepetitiveInput() {
    // Get current input values
    int current_left_y = controllerState.left_joystick_y;
    int current_right_x = controllerState.right_joystick_x;
    bool current_r1 = controllerState.r1_pressed;
    bool current_l1 = controllerState.l1_pressed;
    bool current_l2 = controllerState.l2_pressed;

    // Check if there's any active input (not idle/neutral)
    bool has_active_input = false;

    // Check for significant joystick movement from center (128 is neutral)
    if (abs(current_left_y - 128) > config::tuning::REPETITIVE_INPUT_THRESHOLD ||
        abs(current_right_x - 128) > config::tuning::REPETITIVE_INPUT_THRESHOLD) {
        has_active_input = true;
    }

    // Check for any button presses
    if (current_r1 || current_l1 || current_l2) {
        has_active_input = true;
    }

    // Only monitor repetitive input when there's active input
    if (!has_active_input) {
        // Controller is idle - reset timer and don't monitor
        repetitive_input_start_time = millis();
        repetitive_input_detected = false;

        // Update last known values to current (idle) state
        last_left_y = current_left_y;
        last_right_x = current_right_x;
        last_r1 = current_r1;
        last_l1 = current_l1;
        last_l2 = current_l2;
        return;
    }

    // Check if input has changed significantly (only when active)
    bool input_changed = false;

    // Check joystick changes (use threshold to ignore small drift)
    if (abs(current_left_y - last_left_y) > config::tuning::REPETITIVE_INPUT_THRESHOLD ||
        abs(current_right_x - last_right_x) > config::tuning::REPETITIVE_INPUT_THRESHOLD) {
        input_changed = true;
    }

    // Check button state changes
    if (current_r1 != last_r1 || current_l1 != last_l1 || current_l2 != last_l2) {
        input_changed = true;
    }

    // Update tracking variables
    if (input_changed) {
        // Input changed - reset timer
        repetitive_input_start_time = millis();
        repetitive_input_detected = false;

        // Update last known values
        last_left_y = current_left_y;
        last_right_x = current_right_x;
        last_r1 = current_r1;
        last_l1 = current_l1;
        last_l2 = current_l2;
    } else {
        // Active input hasn't changed - check timeout
        unsigned long current_time = millis();
        if (current_time - repetitive_input_start_time > config::tuning::REPETITIVE_INPUT_TIMEOUT_MS) {
            if (!repetitive_input_detected) {
                DEBUG_PRINTLN("SAFETY: Repetitive active input detected - triggering shutdown!");
                repetitive_input_detected = true;
            }
        }
    }
}

bool isRepetitiveInputDetected() {
    return repetitive_input_detected;
}

void resetRepetitiveInputDetection() {
    repetitive_input_detected = false;
    repetitive_input_start_time = millis();
}