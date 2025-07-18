#line 1 "/Users/admin/Documents/Arduino/robot/src/controller/PS2_controller.h"
#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H
/**
 * PS2_controller.h: Translates PS2 controller inputs into robot commands.
 */

#include <PS2X_lib.h>
#include "controller_state.h"

extern PS2X ps2;

// --- PUBLIC FUNCTIONS ---

// Call in setup() to initialize the PS2 controller
void setupPS2();

// Call in loop() to read the controller
void readController();

// Call in loop() to get the controller state
const ControllerState& getControllerState();

// Connection monitoring functions
bool isConnected();

#endif // PS2_CONTROLLER_H