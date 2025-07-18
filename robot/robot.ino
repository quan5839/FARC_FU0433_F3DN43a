// Main robot includes - optimized for competition performance
#include "src/robot/robot.h"
#include "src/controller/PS2_controller.h"
// Note: Other includes are pulled in through robot.h

Robot robot;
// Note: Removed artificial loop timing - let ESP32 run at maximum natural speed

void setup() {
  Serial.begin(115200);

  // ESP32 Maximum Performance Optimizations
  setCpuFrequencyMhz(240);                    // Boost CPU to 240MHz (50% faster)

  robot.init();
  setupPS2();
  robot.setRobotState(config::MANUAL_CONTROL);

  DEBUG_PRINTLN("Done setup!");
}

void loop() {
  // Performance-optimized main loop
  static unsigned long last_loop_time = 0;

  // Target high-frequency operation (1000Hz = 1ms loop time)
  const unsigned long current_time = micros();
  if (LIKELY(current_time - last_loop_time >= config::performance::TARGET_LOOP_TIME_US)) {
    readController();
    const ControllerState& controllerState = getControllerState();

    robot.processControllerInput(controllerState);
    robot.loop();

    last_loop_time = current_time;
  }

  // Yield to prevent watchdog timeout while maintaining high frequency
  yield();
}