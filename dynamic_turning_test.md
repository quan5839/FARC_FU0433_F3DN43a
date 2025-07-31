# Dynamic Turning Sensitivity Test

## What Changed

The turning sensitivity now increases **quadratically** based on forward/backward drive power:

### **Formula:**
```
Dynamic Turn Sensitivity = Base Sensitivity × (Min Multiplier + (Max Multiplier - Min Multiplier) × Throttle²)
```

### **Configuration:**
- **Base Turn Sensitivity**: 37% (unchanged)
- **Min Turn Multiplier**: 0.3 (30% sensitivity when not moving)
- **Max Turn Multiplier**: 2.0 (200% sensitivity at full speed)

### **Behavior:**
- **Not moving (0% throttle)**: Turn sensitivity = 37% × 0.3 = **11%** (very precise)
- **Half speed (50% throttle)**: Turn sensitivity = 37% × (0.3 + 1.7 × 0.25) = **53%** (moderate)
- **Full speed (100% throttle)**: Turn sensitivity = 37% × 2.0 = **74%** (very responsive)

## Testing Instructions

1. **Upload the updated code**
2. **Test at different speeds:**
   - **Slow forward**: Barely push left joystick forward, try turning → Should be very precise
   - **Medium forward**: Push left joystick halfway, try turning → Should be moderately responsive  
   - **Full forward**: Push left joystick fully forward, try turning → Should be very responsive

3. **Watch serial monitor** (if COMPETITION_MODE = 0):
   - You'll see debug output showing throttle percentage and current turn sensitivity

## Expected Results

- **Low speed driving**: Much more precise turning, easier to make small adjustments
- **High speed driving**: More aggressive turning, easier to make quick direction changes
- **Smooth transition**: Turning responsiveness should gradually increase as you drive faster

## Tuning Options

If you want to adjust the behavior, modify these values in `config.h`:

```cpp
// More aggressive high-speed turning
constexpr float MAX_TURN_MULTIPLIER = 2.5f;  // 250% sensitivity at full speed

// Less precise low-speed turning  
constexpr float MIN_TURN_MULTIPLIER = 0.5f;  // 50% sensitivity when not moving

// More gradual transition (cubic instead of quadratic)
// Would require code change to use throttleIntensity³ instead of throttleIntensity²
```

## Benefits

1. **Better precision** when driving slowly or positioning carefully
2. **Better responsiveness** when driving fast or making quick maneuvers  
3. **More intuitive feel** - turning effort matches driving intensity
4. **Maintains cheesy drive benefits** - forward speed is preserved during turns
