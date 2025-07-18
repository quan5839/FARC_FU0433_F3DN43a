#include <Arduino.h>
#line 1 "/Users/admin/Documents/Arduino/robot/robot.ino"
#include "src/robot/robot.h"            // Include the main robot header
#include "src/hal/pca9685_driver.h" // Include the new central driver
#include "src/hal/motor.h"
#include "src/hal/servos.h"           // Include the new servo library
#include "src/controller/PS2_controller.h"

Robot robot;
unsigned long lastLoopTime = 0;

#line 10 "/Users/admin/Documents/Arduino/robot/robot.ino"
void setup();
#line 20 "/Users/admin/Documents/Arduino/robot/robot.ino"
void loop();
#line 10 "/Users/admin/Documents/Arduino/robot/robot.ino"
void setup() {
  Serial.begin(115200);

  robot.init();
  setupPS2();
  robot.setRobotState(config::MANUAL_CONTROL);

  Serial.println("Done setup!");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastLoopTime < config::ps2::LOOP_MS) {
    return;
  }

  lastLoopTime = currentTime;

  readController();
  const ControllerState& controllerState = getControllerState();

  robot.processControllerInput(controllerState);
  robot.loop();
}
