# Robot Tour Project - Copilot Instructions

## Project Overview
Autonomous robot navigation system for choreographed tile-based movement using PID control. Built on Arduino Mega2560 with dual DC motors, encoder feedback, and 9-DOF IMU (BNO055) for heading correction.

## Architecture Patterns

### Sensor Fusion & Control Loop
- **Gyroscope (BNO055)**: Provides Euler angles for heading correction; initialized with external crystal for accuracy
- **Encoders**: Dual interrupt-driven encoders track motor rotation for distance measurement (1 tick ≈ 2.5mm)
- **PID Controllers**: Two separate instances (`pidStraight`, `pidTurn`) tune independently:
  - Straight: Higher Kp (4.0) for responsive heading correction during forward/backward movement
  - Turn: Higher Kd (0.25) and lower Kp (1.0) for smooth angular convergence

### Movement Model
Movement uses high-level primitives that compose into sequences:
- `fwd()/back()` → `moveStraight()` with PID yaw correction + encoder tick counting
- `left()/right()` → `turn()` with PID heading tracking + normalization (±360°)
- **Braking**: Applied opposite-direction PWM to halt momentum before stopping motors

Key calibration constants:
- Full tile: 770 encoder ticks
- Half tile: 390 ticks  
- Robot length: 180 ticks
- Speeds tuned for specific gear ratio and motor characteristics

### Critical Data Flows
1. **Interrupt chain**: Encoder pulses → `readEncoder<j>()` template → `encoderPos[j]` updates
2. **Control loop**: `updateGyro()` → `computeDeltaT()` → PID evaluate → motor PWM commands
3. **Motor abstraction**: `setMotor(dir, pwm, index)` handles dual motors with unified direction/speed API

## Developer Workflow

### Building & Uploading
```bash
# Build and upload (COM7 configured in platformio.ini)
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

### Testing Movement Sequences
- Movement primitives execute in the main loop after button press debounce
- Edit tile count constants (fullTile, halfTile, robotLength) for different paths
- Serial debug output: "BNO055 Initialized", "Starting Sequences", encoder ticks on movement
- Add `Serial.print(gyroHeadings[0])` to debug heading drift

### Hardware Pinout
- **Motor PWM**: 9, 10
- **Motor direction**: 4, 5 (motor 0) | 6, 7 (motor 1)
- **Encoders**: 2, 3 (A pins) | 13, 12 (B pins)
- **Button start**: Pin 8 (active-low)
- **I2C**: Wire (default SDA/SCL for Mega2560 → BNO055)

## Code Conventions

### Comment Markers
Section headers use `// ?` prefix (e.g., `// ?PID Controller`, `// ?Motor Control`) for easy navigation.

### Angle Handling
`headingDiff()` normalizes heading differences to shortest path (−180 to +180°). Always use for error computation in PID to avoid discontinuities at 0/360°.

### Interrupt-Safe Variables
`encoderPos[]` is volatile; read entire movement completion checks outside interrupt context.

### Template Functions
`readEncoder<int j>()` uses template specialization for compile-time motor selection in ISR—avoids function pointer overhead.

## Known Issues & Constraints
- Gyro initialization failure halts robot indefinitely; add watchdog timer if autonomous operation required
- No timeout on movement loops—infinite loop if encoder fails
- PID integral accumulation (`eintegral`) never resets; can cause steady-state error drift over long sequences
- Motor speed asymmetry not auto-compensated; manual `straightSpeed` tuning required

## Files to Reference
- [src/main.cpp](../src/main.cpp) — Complete movement, sensor, and control implementation
- [platformio.ini](../platformio.ini) — Adafruit BNO055 and sensor library dependencies
