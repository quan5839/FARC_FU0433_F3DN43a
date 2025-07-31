#include "PS2_controller.h"
#include "../config.h"
// Note: PS2X_lib.h is included via PS2_controller.h

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
        controllerState.square_pressed = ps2.Button(PSB_SQUARE);  // A button - Fruit intake down
        controllerState.cross_pressed = ps2.Button(PSB_CROSS);    // X button - Fruit intake up

        // Read and debounce toggle/command buttons
        controllerState.r2_pressed = r2Debouncer.update(ps2.Button(PSB_R2));
        controllerState.circle_pressed = circleDebouncer.update(ps2.Button(PSB_CIRCLE));
        controllerState.select_pressed = selectDebouncer.update(ps2.Button(PSB_SELECT));
        controllerState.start_pressed = startDebouncer.update(ps2.Button(PSB_START));
        controllerState.pad_right_pressed = padRightDebouncer.update(ps2.Button(PSB_PAD_RIGHT));
        controllerState.pad_down_pressed = padDownDebouncer.update(ps2.Button(PSB_PAD_DOWN));
        controllerState.pad_up_pressed = padUpDebouncer.update(ps2.Button(PSB_PAD_UP));
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