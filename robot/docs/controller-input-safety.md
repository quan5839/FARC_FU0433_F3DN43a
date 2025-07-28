# Controller Input Safety Feature

## Overview

The Controller Input Safety feature is designed to automatically stop the robot when the controller has constant, unchanging input for 5 consecutive seconds. This prevents situations where a stuck controller button or joystick could cause the robot to continue moving uncontrollably.

## How It Works

### Detection Logic
The safety system monitors:
- **Joystick positions**: Left joystick Y-axis and right joystick X-axis
- **Button states**: All controller buttons (R1, L1, L2, R2, Circle, Select, Start, D-pad buttons)

### Triggering Conditions
1. **Input Detection**: Any significant controller input is detected (joystick movement > 2 units or any button pressed)
2. **Constant Input**: The input values remain unchanged for 5 consecutive seconds
3. **Safety Activation**: Robot automatically stops all motors and enters IDLE state

### Input Change Detection
The system considers input "changed" when:
- Joystick position changes by more than 2 units from the last reading
- Any button state changes (pressed/released)

## Configuration

### Enable/Disable Feature
```cpp
// In src/config.h
constexpr bool ENABLE_CONTROLLER_INPUT_SAFETY = true;  // Set to false to disable
```

### Timeout Duration
```cpp
// In src/config.h
constexpr unsigned long CONTROLLER_INPUT_SAFETY_TIMEOUT_MS = 5000;  // 5 seconds
```

### Input Change Threshold
```cpp
// In src/config.h
constexpr int CONTROLLER_INPUT_CHANGE_THRESHOLD = 2;  // Minimum change to detect variation
```

## User Interface

### Safety Activation
When the safety feature triggers:
1. **Console Message**: "CONTROLLER SAFETY: Constant input detected for 5 seconds - activating safety shutdown"
2. **Robot Behavior**: All motors stop immediately and robot enters IDLE state
3. **Input Blocking**: All controller input is ignored until safety is reset
4. **LED Indicator**: LED strip shows orange/yellow warning pattern (status 5)
5. **Clean Shutdown**: Does NOT trigger the error handling system or consecutive error counting

### Safety Reset
To reset the controller input safety and resume normal operation:
- **Button Combination**: Press **R2 + SELECT** simultaneously
- **Console Message**: "Controller input safety reset triggered via controller (R2 + SELECT)"
- **Result**: Safety monitoring is reset and normal operation resumes

## Technical Implementation

### Files Modified
- `src/utils/safety_monitor.h` - Added controller safety monitoring functions
- `src/utils/safety_monitor.cpp` - Implemented safety logic
- `src/robot/robot.cpp` - Integrated safety checks into main control loop
- `src/config.h` - Added configuration constants

### Key Functions
- `SafetyMonitor::checkControllerInputSafety()` - Main safety monitoring function
- `SafetyMonitor::isControllerSafetyShutdownActive()` - Check if safety shutdown is active
- `SafetyMonitor::resetControllerInputSafety()` - Reset safety monitoring

### Integration Points
1. **Main Control Loop**: Safety check runs before processing any controller input
2. **Input Processing**: All input is blocked when safety shutdown is active
3. **State Management**: Robot automatically transitions to IDLE state on safety trigger

## Testing

### Manual Testing
1. **Normal Operation**: Verify robot responds normally to controller input
2. **Safety Trigger**: Hold a joystick in one position for 5+ seconds and verify robot stops
3. **Safety Reset**: Use R2 + SELECT to reset and verify normal operation resumes
4. **Input Variation**: Move joystick slightly during constant input to verify timer resets

### Debug Output
Enable debug output to monitor safety system:
```cpp
#define DEBUG_MODE 1  // In config.h
```

Debug messages include:
- "Controller input safety monitoring started"
- "Controller input changed - safety timer reset"
- "Controller input stopped - safety monitoring reset"
- "Controller input safety reset triggered via controller (R2 + SELECT)"

## Safety Considerations

### When Safety Triggers
- **Stuck buttons**: Physical button stuck in pressed position
- **Joystick drift**: Joystick not returning to neutral position
- **Controller malfunction**: Electronic failure causing constant signals
- **Intentional constant input**: User holding input for extended period

### Limitations
- **Minimum variation**: Small joystick movements (< 2 units) won't reset the timer
- **Button combinations**: Different button combinations may not reset timer if core buttons remain constant
- **Disable option**: Feature can be disabled in configuration if needed

### Competition Use
- **Reliability**: Feature is designed to be fail-safe and not interfere with normal operation
- **Performance**: Minimal overhead on main control loop
- **Override**: Can be quickly reset during competition if needed

## Troubleshooting

### False Positives
If safety triggers during normal operation:
1. Check joystick calibration and deadzone settings
2. Verify controller is not physically damaged
3. Adjust `CONTROLLER_INPUT_CHANGE_THRESHOLD` if needed

### Safety Not Triggering
If safety doesn't activate when expected:
1. Verify `ENABLE_CONTROLLER_INPUT_SAFETY` is set to `true`
2. Check debug output for monitoring messages
3. Ensure input exceeds the change threshold

### Reset Not Working
If R2 + SELECT doesn't reset safety:
1. Verify both buttons are pressed simultaneously
2. Check controller connection and button functionality
3. Look for debug message confirming reset command

### Error System Triggering (Fixed in Latest Version)
**Previous Issue**: In earlier versions, controller safety shutdown would trigger the robot's error handling system, causing messages like:
- "System error detected. Consecutive errors: 3"
- "CRITICAL: Maximum consecutive errors reached - entering emergency stop"
- "PCA9685 EMERGENCY STOP - All outputs disabled"

**Solution**: The controller safety system now operates independently from the error handling system. Controller safety shutdown is a clean, intentional stop that does not increment error counters or trigger emergency stop procedures.
