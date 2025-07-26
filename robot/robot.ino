// Main robot includes - optimized for competition performance
#include "src/robot/robot.h"
#include "src/controller/PS2_controller.h"
// Note: Other includes are pulled in through robot.h
Robot robot;

void setup() {
  Serial.begin(config::constants::SERIAL_BAUD_RATE);
  delay(config::constants::SERIAL_INIT_DELAY_MS);  // Give serial time to initialize
  DEBUG_PRINTLN("=== Robot Starting ===");
  DEBUG_PRINTLN("Serial communication test");
  DEBUG_PRINT("Baud rate: ");
  DEBUG_PRINTLN(config::constants::SERIAL_BAUD_RATE);



  // ESP32 Maximum Performance Optimizations
  setCpuFrequencyMhz(config::performance::ESP32_MAX_CPU_FREQ_MHZ);

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