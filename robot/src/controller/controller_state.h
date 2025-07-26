#ifndef CONTROLLER_STATE_H
#define CONTROLLER_STATE_H

// Removed PS2X_lib.h include - using PsxNewLib instead to avoid conflicts

struct ControllerState {
    // Joysticks
    int left_joystick_y;
    int right_joystick_x;

    // Buttons (immediate states - no debouncing needed for continuous actions)
    bool r1_pressed;      // Precision mode - needs immediate response
    bool l1_pressed;      // Outtake forward - needs immediate response
    bool l2_pressed;      // Outtake reverse - needs immediate response

    // Debounced buttons (only true for one cycle after press)
    bool r2_pressed;      // Servo toggle - needs debouncing
    bool circle_pressed;  // Servo toggle - needs debouncing
    bool select_pressed;  // Special command - needs debouncing
    bool start_pressed;   // Special command - needs debouncing
    bool pad_right_pressed; // Special command - needs debouncing
    bool pad_down_pressed;  // Special command - needs debouncing
    bool pad_up_pressed;    // Special command - needs debouncing
};

/**
 * @brief Button debouncing helper class
 * Ensures buttons only trigger once per press, preventing rapid toggling
 */
class ButtonDebouncer {
public:
    ButtonDebouncer() : lastState(false), triggered(false) {}

    /**
     * @brief Update button state and return true only on rising edge
     * @param newState Current button state from controller
     * @return true only once when button is first pressed
     */
    bool update(bool newState) {
        bool result = false;
        if (newState && !lastState && !triggered) {
            result = true;
            triggered = true;
        } else if (!newState) {
            triggered = false;
        }
        lastState = newState;
        return result;
    }

private:
    bool lastState;
    bool triggered;
};

#endif // CONTROLLER_STATE_H
