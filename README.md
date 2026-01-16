# VEX V5 Robotics Control System

A comprehensive C++ codebase for VEX V5 competitive robotics, featuring PID-based motion control, three-wheel odometry tracking, autonomous routines, and driver control systems. This codebase is designed for the "Push Back" competition and includes advanced features such as ball color sorting, pneumatic wing control, and a descoring arm mechanism.

---

## Table of Contents

1. [Project Structure](#project-structure)
2. [System Architecture](#system-architecture)
3. [Hardware Configuration](#hardware-configuration)
4. [Core Components](#core-components)
   - [PID Controller](#pid-controller)
   - [Motion Controller](#motion-controller)
   - [Odometry System](#odometry-system)
   - [Driver Control](#driver-control)
   - [Autonomous System](#autonomous-system)
   - [Subsystems](#subsystems)
5. [Utility Modules](#utility-modules)
6. [Control Theory Background](#control-theory-background)
7. [Tuning Guide](#tuning-guide)
8. [API Reference](#api-reference)
9. [Building and Deployment](#building-and-deployment)
10. [Troubleshooting](#troubleshooting)

---

## Project Structure

```
vex-robotics/
├── include/                    # Header files
│   ├── PID.h                   # PID controller class with anti-windup
│   ├── autons.h                # Autonomous routine declarations
│   ├── drive.h                 # Drivetrain control interface
│   ├── manual.h                # Driver control entry point
│   ├── motion.h                # Motion controller class
│   ├── odom.h                  # Odometry system interface
│   ├── pre_auton.h             # Pre-autonomous initialization
│   ├── robot_config.h          # Hardware configuration declarations
│   ├── sensors.h               # Sensor utility functions
│   ├── subsystems.h            # Subsystem controllers (wings, intake, etc.)
│   ├── utils.h                 # Mathematical utility functions
│   └── vex.h                   # VEX API includes and macros
│
├── src/                        # Source files
│   ├── autons.cpp              # Autonomous routine implementations
│   ├── drive.cpp               # Drivetrain implementation
│   ├── main.cpp                # Program entry point and competition callbacks
│   ├── manual.cpp              # Driver control implementation
│   ├── motion.cpp              # Motion controller implementation
│   ├── odom.cpp                # Odometry calculation and tracking
│   ├── pre_auton.cpp           # Sensor calibration and task initialization
│   ├── robot_config.cpp        # Hardware object definitions
│   └── subsystems.cpp          # Subsystem implementations
│
└── README.md                   # This documentation file
```

---

## System Architecture

The codebase follows a modular architecture with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────────┐
│                         main.cpp                                │
│              (Competition callbacks & main loop)                │
└─────────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┼────────────────────┐
         ▼                    ▼                    ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│   pre_auton()   │  │  autonomous()   │  │  usercontrol()  │
│                 │  │                 │  │                 │
│ - Sensor init   │  │ - Run selected  │  │ - Driver input  │
│ - Task startup  │  │   auton routine │  │ - Subsystem     │
│ - Odometry init │  │                 │  │   control       │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                    │                    │
         │                    ▼                    │
         │           ┌─────────────────┐           │
         │           │ MotionController│           │
         │           │                 │           │
         │           │ - drive()       │           │
         │           │ - turnTo()      │           │
         │           │ - turnBy()      │           │
         │           └─────────────────┘           │
         │                    │                    │
         ▼                    ▼                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                        Shared Resources                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
│  │   PID    │  │  Odom    │  │  Drive   │  │   Subsystems     │ │
│  │Controller│  │  Task    │  │ Functions│  │ (Wings, Intake)  │ │
│  └──────────┘  └──────────┘  └──────────┘  └──────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                      Hardware Layer                             │
│         (Motors, Sensors, Pneumatics via robot_config)          │
└─────────────────────────────────────────────────────────────────┘
```

### Task Structure

The system runs multiple concurrent tasks:

| Task | Priority | Rate | Purpose |
|------|----------|------|---------|
| Main | Normal | 10 Hz | Competition state management |
| Odometry | Background | 100 Hz | Position tracking |
| Ball Sorter | Background | 50 Hz | Color-based ball ejection |
| User Control | Foreground | 50 Hz | Driver input processing |

---

## Hardware Configuration

### Motor Assignments

#### Drivetrain (6-Motor Tank Drive)

| Motor | Port | Gear Ratio | Reversed | Position |
|-------|------|------------|----------|----------|
| LeftA | 3 | 6:1 (600 RPM) | No | Front Left |
| LeftB | 19 | 6:1 (600 RPM) | No | Middle Left |
| LeftC | 10 | 6:1 (600 RPM) | No | Rear Left |
| RightA | 11 | 6:1 (600 RPM) | Yes | Front Right |
| RightB | 13 | 6:1 (600 RPM) | Yes | Middle Right |
| RightC | 15 | 6:1 (600 RPM) | Yes | Rear Right |

#### Intake System (3 Motors)

| Motor | Port | Gear Ratio | Reversed | Function |
|-------|------|------------|----------|----------|
| IntakeLeft | 16 | 6:1 (600 RPM) | No | Left intake roller |
| IntakeRight | 5 | 6:1 (600 RPM) | Yes | Right intake roller |
| Midtake | 8 | 6:1 (600 RPM) | No | Middle transfer |

#### Outtake System (3 Motors)

| Motor | Port | Gear Ratio | Reversed | Function |
|-------|------|------------|----------|----------|
| LeftOutake | 2 | 6:1 (600 RPM) | No | Left outtake |
| RightOutake | 7 | 6:1 (600 RPM) | Yes | Right outtake |
| MidOutake | 20 | 6:1 (600 RPM) | Yes | Middle outtake |

#### Auxiliary Motors

| Motor | Port | Gear Ratio | Reversed | Function |
|-------|------|------------|----------|----------|
| DescoreMotor | 18 | 36:1 (100 RPM) | Yes | Descoring arm |

### Sensor Assignments

| Sensor | Port/Wire | Type | Purpose |
|--------|-----------|------|---------|
| inertial_sensor | Port 1 | V5 Inertial | Heading and rotation |
| verticalRot | Port 9 | V5 Rotation | Forward/backward tracking |
| horizontalRot | Port 6 | V5 Rotation | Left/right tracking |
| ballSensor | Port 21 | V5 Optical | Ball color detection |
| Descore | 3-Wire A | Potentiometer | Arm position feedback |

### Pneumatic Assignments

| Device | Port | Function |
|--------|------|----------|
| wingsPiston | 3-Wire B | Wing deployment |

### Physical Constants

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| TRACK_WIDTH_M | 0.320 | meters | Distance between wheel centers |
| TRACKING_WHEEL_CIRCUMFERENCE_M | 0.2623 | meters | π × 0.0835m diameter |
| ARCADE_DEADBAND | 1.0 | percent | Joystick deadzone |
| SIDE_OFFSET_M | 0.0 | meters | Horizontal wheel offset from center |
| VERT_OFFSET_M | 0.0 | meters | Vertical wheel offset from center |

---

## Core Components

### PID Controller

The `PID` class (`PID.h`) implements a full-featured PID controller with industrial-grade features for robust motion control.

#### Features

1. **Three-Term Control**: Proportional, Integral, and Derivative terms
2. **Derivative Filtering**: Low-pass filter to reduce noise amplification
3. **Derivative Mode Selection**: Calculate on error or measurement
4. **Integral Anti-Windup**: Multiple strategies to prevent integral saturation
5. **Output Saturation**: Configurable output limits
6. **Feed-Forward**: Open-loop term for improved response
7. **Error Deadband**: Prevents jitter at setpoint
8. **Bumpless Transfer**: Smooth transitions when changing setpoints

#### Block Diagram

```
                                    ┌─────────────┐
                         ┌─────────▶│  Integral   │─────────┐
                         │          │   Σ(e·dt)   │         │
                         │          └─────────────┘         │
                         │                                  │
  Setpoint    ┌──────┐   │          ┌─────────────┐         │    ┌──────────┐
  ──────────▶│  +   │───┼─────────▶│ Proportional│─────────┼───▶│    +     │───▶ Output
             │  -   │   │          │    Kp·e     │         │    │  Σ terms │
  Measure ──▶│      │   │          └─────────────┘         │    └──────────┘
             └──────┘   │                                  │         ▲
                 │      │          ┌─────────────┐         │         │
                 │      └─────────▶│ Derivative  │─────────┘         │
                 │                 │  Kd·de/dt   │                   │
                 │                 │ (filtered)  │                   │
                 │                 └─────────────┘                   │
                 │                                                   │
                 │                 ┌─────────────┐                   │
                 └────────────────▶│ Feed-Forward│───────────────────┘
                                   │     FF      │
                                   └─────────────┘
```

#### Anti-Windup Mechanisms

The controller implements three anti-windup strategies:

1. **Integral Clamping**: Hard limits on integral accumulation
   ```cpp
   setIntegralLimits(iMin, iMax);
   ```

2. **Integral Zone**: Only integrate when error is small
   ```cpp
   setIntegralZone(zone);  // Only integrate when |error| < zone
   ```

3. **Back-Calculation**: Reduce integral when output saturates
   ```cpp
   setAntiWindupTau(tauSec);  // Time constant for back-calculation
   ```

#### Usage Example

```cpp
PID controller(1.0, 0.1, 0.05);  // kP, kI, kD

controller.setSetpoint(100.0);
controller.setOutputLimits(-100, 100);
controller.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
controller.setDerivativeFilterTf(0.02);  // 20ms filter
controller.setIntegralLimits(-50, 50);
controller.reset();

while (running) {
    double measurement = getSensorValue();
    double output = controller.update(measurement, 0.01);  // 10ms dt
    setMotorPower(output);
    wait(10, msec);
}
```

---

### Motion Controller

The `MotionController` class (`motion.h`, `motion.cpp`) provides high-level motion primitives using PD control for accurate autonomous movement.

#### Architecture

```
┌────────────────────────────────────────────────────────────┐
│                    MotionController                        │
│                                                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐     │
│  │   distPD_    │  │   headPD_    │  │   turnPD_    │     │
│  │              │  │              │  │              │     │
│  │ Distance     │  │ Heading      │  │ Turn         │     │
│  │ Control      │  │ Hold         │  │ Control      │     │
│  │              │  │              │  │              │     │
│  │ kP: 0.1      │  │ kP: 1.5      │  │ kP: 3.5      │     │
│  │ kD: 0.0      │  │ kD: 0.04     │  │ kD: 0.35     │     │
│  └──────────────┘  └──────────────┘  └──────────────┘     │
│                                                            │
│  Methods:                                                  │
│  - drive(distM, timeoutMs, maxSpeedPct)                   │
│  - turnTo(targetDeg, timeoutMs)                           │
│  - turnBy(deltaDeg, timeoutMs)                            │
└────────────────────────────────────────────────────────────┘
```

#### drive() Method

Drives the robot straight for a specified distance while maintaining heading.

**Algorithm:**

1. Record starting encoder position and heading
2. Loop at 100 Hz until timeout or settled:
   - Calculate distance traveled from encoder
   - Compute distance error (target - traveled)
   - Apply dynamic speed cap based on distance remaining
   - Calculate PD output for forward velocity
   - Apply minimum floor power to overcome friction
   - Compute heading error from initial heading
   - Calculate heading correction (scaled by velocity)
   - Command tank drive with velocity ± correction
   - Check settle criteria (error and velocity thresholds)
3. Stop motors with brake

**Dynamic Speed Capping:**

```
cap = min(maxSpeed, minCap + 200 × |distanceError|)
```

This creates a velocity profile that starts fast and slows as the target approaches.

**Settle Criteria:**

- Distance error < 0.015 meters
- Velocity command < 10%
- Sustained for 120ms

#### turnTo() Method

Turns to an absolute field heading using the IMU.

**Algorithm:**

1. Calculate initial heading error
2. Initialize rate filter for oscillation detection
3. Loop at 100 Hz until timeout or settled:
   - Read current heading from IMU
   - Compute heading error with wraparound handling
   - Update turn rate filter (exponential moving average)
   - Calculate PD output
   - Command tank drive (differential: +out, -out)
   - Check settle criteria (error < 1° AND rate < 8°/s)
4. Stop motors with brake

#### turnBy() Method

Turns by a relative angle from current heading.

Uses continuous rotation (not heading) to avoid wraparound issues at 0°/360°.

**Key Difference from turnTo():**

- Uses `inertial_sensor.rotation()` (continuous) instead of `heading()` (0-360)
- Calculates target as `startRotation + deltaDeg`
- Same control algorithm as turnTo()

#### Usage Example

```cpp
MotionController motion;

// Drive forward 1 meter
motion.drive(1.0, 3000, 80);  // 1m, 3s timeout, 80% max speed

// Turn to face 90 degrees (field-absolute)
motion.turnTo(90, 2000);      // 90°, 2s timeout

// Turn 45 degrees clockwise from current heading
motion.turnBy(45, 2000);      // +45°, 2s timeout

// Drive backward 0.5 meters
motion.drive(-0.5, 2000, 60); // -0.5m, 2s timeout, 60% max speed
```

---

### Odometry System

The odometry system (`odom.h`, `odom.cpp`) provides continuous position tracking using three-wheel odometry with IMU fusion.

#### Theory

Three-wheel odometry uses two perpendicular tracking wheels to measure translation and an IMU for rotation:

```
                    ▲ +Y (Forward)
                    │
                    │
        ┌───────────┼───────────┐
        │           │           │
        │     ┌─────┴─────┐     │
        │     │   IMU     │     │
        │     └───────────┘     │
        │                       │
   ◀────┼───────────────────────┼────▶ +X (Right)
   Horiz│                       │
   Wheel│      Robot Body       │
        │                       │
        │           │           │
        │           │           │
        │         ┌─┴─┐         │
        │         │   │ Vert    │
        │         │   │ Wheel   │
        └─────────┴───┴─────────┘
```

#### Pose Structure

```cpp
struct Pose {
    double x;      // X position in meters (positive = right)
    double y;      // Y position in meters (positive = forward)
    double theta;  // Heading in radians (continuous, positive = CCW)
};
```

#### Algorithm

Each iteration (10ms):

1. **Read Sensors:**
   ```cpp
   v_m = degToMeters(verticalWheel.position());
   h_m = degToMeters(horizontalWheel.position());
   rotDeg = imu.rotation();
   ```

2. **Calculate Deltas:**
   ```cpp
   dV = v_m - prevV_m;           // Vertical wheel displacement
   dH = h_m - prevH_m;           // Horizontal wheel displacement
   dTheta = degToRad(rotDeg - prevRotDeg);  // Angular change
   ```

3. **Correct for Rotation:**
   
   When the robot rotates, the tracking wheels trace arcs. We must subtract this arc motion:
   ```cpp
   dHc = dH - SIDE_OFFSET × dTheta;   // Corrected horizontal
   dVc = dV - VERT_OFFSET × dTheta;   // Corrected vertical
   ```

4. **Calculate Local Displacement:**
   
   For small angles, use linear approximation. For larger angles, use chord length:
   ```cpp
   if (|dTheta| < 1e-6) {
       localX = dHc;
       localY = dVc;
   } else {
       chord = 2 × sin(dTheta / 2);
       localX = chord × (dHc / dTheta);
       localY = chord × (dVc / dTheta);
   }
   ```

5. **Transform to Field Frame:**
   ```cpp
   avgTheta = theta + dTheta / 2;
   x += localX × cos(avgTheta) + localY × sin(avgTheta);
   y += localY × cos(avgTheta) - localX × sin(avgTheta);
   theta += dTheta;
   ```

#### Coordinate System

- **Origin**: Robot starting position
- **+X Axis**: Robot's right at start
- **+Y Axis**: Robot's forward at start
- **+Theta**: Counter-clockwise rotation

#### Usage

```cpp
// Access current pose
double x_meters = robotPose.x;
double y_meters = robotPose.y;
double heading_rad = robotPose.theta;

// Convert to degrees
double heading_deg = radToDeg(robotPose.theta);

// Reset odometry to origin
resetOdometry();
```

---

### Driver Control

The driver control system (`manual.h`, `manual.cpp`) handles all operator input during the driver-controlled period.

#### Control Scheme

**Arcade Drive with Exponential Curves:**

The joystick inputs are processed through an exponential curve for fine control at low speeds:

```cpp
output = (curve × input³) + ((1 - curve) × input)
```

Where `curve = 0.1` provides mostly linear response with slight exponential character.

```
Output │
  100% │                    ╱
       │                  ╱
       │                ╱
       │             ╱
       │          ╱
       │       ╱
       │    ╱
       │ ╱
    0% └──────────────────────
       0%                  100%  Input
```

**Slew Rate Limiting:**

Motor commands are rate-limited to prevent wheel slip and mechanical stress:

```cpp
maxAccelStep = ACCEL_PCT_PER_S × dt;  // 300%/s × 0.02s = 6%
maxDecelStep = DECEL_PCT_PER_S × dt;  // 300%/s × 0.02s = 6%
```

The slew differentiates between acceleration (increasing magnitude) and deceleration (decreasing magnitude).

#### Button Mapping

| Button | Action | Details |
|--------|--------|---------|
| **Axis 3** | Forward/Backward | Left stick Y-axis |
| **Axis 1** | Turn Left/Right | Right stick X-axis |
| **R1** | Toggle Speed Mode | Fast (80%) ↔ Slow (40%) |
| **Up** | Toggle Wings | Extend/retract pneumatic wings |
| **Left** | Arm Up | Hold to raise descoring arm |
| **Right** | Arm Down | Hold to lower descoring arm |
| **L1** | Intake Only | Run intake, stop outtake |
| **L2** | Intake + Outtake | Run both systems |
| **R2** | Reverse All | Reverse intake and outtake |
| **X** | Reset Odometry | Zero position and heading |
| **Y** | Toggle Display | Arm info ↔ Odometry |

#### Arm Control System

The descoring arm uses a position-based variable speed profile:

```
Power │
      │
 MAX  │      ╱────╲
      │    ╱        ╲
      │  ╱            ╲
 MIN  │╱                ╲
      └───────────────────
       UP    MIDDLE    DOWN
            Position
```

- **Near limits (UP/DOWN)**: Minimum power (12%) for safety
- **At midpoint**: Maximum power (60%) for speed
- **Transition**: Quadratic curve between them

This prevents slamming into hard stops while maintaining responsive movement.

#### Controller Screen Display

**Normal Mode:**
```
┌──────────────────────┐
│ A:150  T:190         │  ← Current angle, Target angle
│ SPEED: FAST          │  ← Speed mode
│ WINGS: DOWN          │  ← Wing state
└──────────────────────┘
```

**Odometry Mode:**
```
┌──────────────────────┐
│ X:  45.2 cm          │  ← X position
│ Y: 120.5 cm          │  ← Y position
│ T:  90.0 deg         │  ← Heading
└──────────────────────┘
```

---

### Autonomous System

The autonomous system (`autons.h`, `autons.cpp`) provides pre-programmed routines for the autonomous period.

#### Routine Selection

```cpp
enum class AutonRoutine {
    NONE,           // No operation
    TEST,           // Basic movement test
    RED_LEFT,       // Red alliance, left start
    RED_RIGHT,      // Red alliance, right start
    BLUE_LEFT,      // Blue alliance, left start
    BLUE_RIGHT,     // Blue alliance, right start
    SKILLS,         // Skills challenge
    TURN_TEST,      // Turn calibration
    DRIVE_TEST,     // Drive calibration
    SQUARE_TEST     // Accuracy test (drive square)
};

extern AutonRoutine selectedAuton;  // Set before match
```

#### Routine Descriptions

**TEST** - Basic Movement Test
```cpp
static void autonTest() {
    MotionController m;
    m.drive(0.8);      // Drive 80cm forward
    m.turnBy(90);      // Turn 90° right
    m.drive(0.4);      // Drive 40cm forward
}
```

**RED_LEFT** - Red Alliance Left Position
```cpp
static void redLeft() {
    MotionController m;
    runIntake(100);           // Start intake
    m.drive(1.2);             // Drive to game pieces
    m.turnTo(90);             // Face scoring zone
    m.drive(0.5);             // Approach goal
    reverseIntake(100);       // Eject
    reverseOutake(100);
    wait(800, msec);
    stopIntake();
    stopOutake();
}
```

**SQUARE_TEST** - Accuracy Verification
```cpp
// Drives a 0.8m square, should return to start
for (int i = 0; i < 4; i++) {
    m.drive(0.8);
    m.turnBy(90);
}
```

Use SQUARE_TEST to verify odometry and motion control accuracy. The robot should return to within a few centimeters of its starting position.

---

### Subsystems

The subsystems module (`subsystems.h`, `subsystems.cpp`) manages auxiliary mechanisms.

#### Wings Class

Pneumatic wing deployment for pushing game elements:

```cpp
class Wings {
public:
    void toggle();              // Flip current state
    void set(bool extended);    // Set specific state
    bool isExtended() const;    // Query current state
private:
    bool state;
};

extern Wings wings;  // Global instance
```

#### Intake/Outtake Functions

```cpp
// Intake control
void runIntake(double speedPct = 100.0);
void reverseIntake(double speedPct = 100.0);
void stopIntake();

// Outtake control
void runOutake(double speedPct = 100.0);
void reverseOutake(double speedPct = 100.0);
void stopOutake();
```

#### Ball Color Sorting

Automatic sorting using the optical sensor:

**Detection Thresholds:**
- Red: hue < 20° OR hue > 340°
- Blue: 200° < hue < 250°

**Algorithm:**
1. Optical sensor continuously monitors for nearby objects
2. When ball detected, read hue value through median filter
3. Compare color to alliance setting
4. If wrong color: reverse intake/outtake for 300ms to eject
5. Resume normal operation

**Median Filter:**

Reduces noise in hue readings:
```cpp
static double filteredHue() {
    static std::vector<double> buf;
    buf.push_back(ballSensor.hue());
    if (buf.size() > 5) buf.erase(buf.begin());
    
    auto sorted = buf;
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
}
```

---

## Utility Modules

### utils.h

Mathematical helper functions:

| Function | Description |
|----------|-------------|
| `clampD(v, lo, hi)` | Clamp double to range [lo, hi] |
| `clampPct(v)` | Clamp to [-100, 100] |
| `degToRad(deg)` | Convert degrees to radians |
| `radToDeg(rad)` | Convert radians to degrees |
| `wrap180(deg)` | Wrap angle to (-180, 180] |

### sensors.h

Sensor interface functions:

| Function | Description |
|----------|-------------|
| `initSensors()` | Calibrate IMU, reset encoders |
| `headingDeg()` | Get IMU heading (0-360°) |
| `rotationDeg()` | Get IMU rotation (continuous) |
| `verticalDeg()` | Get vertical tracking wheel position |
| `horizontalDeg()` | Get horizontal tracking wheel position |

---

## Control Theory Background

### PID Control

PID control combines three terms to minimize error:

**Proportional (P):** Responds to current error
```
P = Kp × error
```
- Larger Kp = faster response, but can cause overshoot
- Too large = oscillation

**Integral (I):** Responds to accumulated error
```
I = Ki × ∫error dt
```
- Eliminates steady-state error
- Too large = slow oscillation, windup

**Derivative (D):** Responds to rate of change
```
D = Kd × d(error)/dt
```
- Dampens oscillation
- Too large = noise amplification, jitter

### Derivative on Measurement

Instead of differentiating error, differentiate measurement:

```
D = -Kd × d(measurement)/dt
```

**Benefits:**
- No derivative kick on setpoint change
- Smoother response to step inputs
- Less sensitive to setpoint noise

### Derivative Filtering

Raw derivatives amplify high-frequency noise. A low-pass filter smooths the signal:

```
filtered += α × (raw - filtered)
α = dt / (Tf + dt)
```

Where Tf is the filter time constant. Larger Tf = more smoothing but more lag.

---

## Tuning Guide

### Motion Controller Tuning

#### Distance PD (distPD_)

| Parameter | Value | Effect of Increase |
|-----------|-------|-------------------|
| kP | 0.1 | Faster approach, more overshoot |
| kD | 0.0 | More damping (unused currently) |
| Filter Tf | 0.10s | Smoother but slower response |
| Deadband | 0.003m | Larger settling tolerance |

**Symptoms and Fixes:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| Undershoots target | kP too low | Increase kP |
| Overshoots and oscillates | kP too high | Decrease kP, add kD |
| Never settles | Deadband too small | Increase deadband |
| Stops short | Floor power too low | Increase kFloor |

#### Turn PD (turnPD_)

| Parameter | Value | Effect of Increase |
|-----------|-------|-------------------|
| kP | 3.5 | Faster turns, more overshoot |
| kD | 0.35 | More damping |
| Filter Tf | 0.06s | Smoother but slower |
| Deadband | 0.3° | Larger settling tolerance |
| Output limit | ±60% | Faster maximum turn speed |

**Symptoms and Fixes:**

| Symptom | Cause | Fix |
|---------|-------|-----|
| Turn overshoots | kP too high, kD too low | Decrease kP, increase kD |
| Turn undershoots | kP too low | Increase kP |
| Turn oscillates | kD too low | Increase kD |
| Turn buzzes/jitters | kD too high, filter too small | Decrease kD, increase filter |

#### Heading Hold PD (headPD_)

| Parameter | Value | Effect of Increase |
|-----------|-------|-------------------|
| kP | 1.5 | Tighter heading hold |
| kD | 0.04 | Smoother correction |
| Output limit | ±12% | Stronger corrections |

### Systematic Tuning Process

1. **Start with P only** (I=0, D=0)
   - Increase P until oscillation begins
   - Back off P by ~20%

2. **Add D for damping**
   - Increase D until oscillation stops
   - Continue until response feels crisp

3. **Add I if needed** (for eliminating steady-state error)
   - Start very small (1/10 of P)
   - Increase until steady-state error eliminated
   - Add integral limits to prevent windup

4. **Fine-tune filters and deadbands**
   - Adjust derivative filter if noisy
   - Adjust deadband for settling behavior

---

## API Reference

### PID Class

```cpp
class PID {
public:
    enum class DerivativeMode { OnError, OnMeasurement };
    
    // Constructor
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0);
    
    // Configuration
    void setGains(double kp, double ki, double kd);
    void setSetpoint(double sp);
    void setOutputLimits(double outMin, double outMax);
    void setIntegralLimits(double iMin, double iMax);
    void setIntegralZone(double zone);
    void setDerivativeFilterTf(double tfSec);
    void setDerivativeMode(DerivativeMode mode);
    void setFeedForward(double ff);
    void setAntiWindupTau(double tauSec);
    void setErrorDeadband(double deadband);
    
    // State access
    double getSetpoint() const;
    double getError() const;
    double getOutput() const;
    double getIntegral() const;
    
    // Control
    void reset(double measurement = 0.0);
    void resetBumpless(double measurement, double holdOutput = 0.0);
    double update(double measurement, double dtSec);
};
```

### MotionController Class

```cpp
class MotionController {
public:
    MotionController();
    
    // Motion primitives
    void drive(double distM, int timeoutMs = 3000, double maxSpeedPct = 100.0);
    void turnTo(double targetDeg, int timeoutMs = 3000);
    void turnBy(double deltaDeg, int timeoutMs = 3000);
    
    // Utilities
    static double wrap180(double a);
    static double angleDiffDeg(double targetDeg, double currentDeg);
};
```

### Drive Functions

```cpp
void tankDrive(double leftPct, double rightPct);
void arcadeDrive(double fwdPct, double turnPct);
void stopDrive(vex::brakeType mode = vex::brakeType::brake);
```

### Subsystem Functions

```cpp
// Wings
void Wings::toggle();
void Wings::set(bool extended);
bool Wings::isExtended() const;

// Intake
void runIntake(double speedPct = 100.0);
void reverseIntake(double speedPct = 100.0);
void stopIntake();

// Outtake
void runOutake(double speedPct = 100.0);
void reverseOutake(double speedPct = 100.0);
void stopOutake();
```

### Odometry

```cpp
extern Pose robotPose;  // Current position (x, y in meters, theta in radians)

void resetOdometry();   // Reset to origin
int odomTaskFn();       // Background task function
```

---

## Building and Deployment

### Prerequisites

- VEXcode Pro V5 or PROS CLI
- VEX V5 Brain and Controller
- USB or Bluetooth connection

### VEXcode Pro V5

1. Open VEXcode Pro V5
2. Create new project or open existing
3. Add all source files to `src/` folder
4. Add all header files to `include/` folder
5. Click "Build" to compile
6. Connect V5 Brain via USB
7. Click "Download" to deploy

### PROS

```bash
# Create new project
pros conductor new-project vex-robotics v5

# Build
pros make

# Upload
pros upload

# Terminal (for debugging)
pros terminal
```

---

## Troubleshooting

### Common Issues

**Robot drifts while driving straight:**
- Check LEFT_BIAS and RIGHT_BIAS values
- Verify motors are not binding
- Check for wheel slippage
- Increase headPD_ kP gain

**Turns overshoot:**
- Decrease turnPD_ kP
- Increase turnPD_ kD
- Increase rate tolerance (rateTol)

**Robot doesn't reach target distance:**
- Increase distPD_ kP
- Increase kFloor constant
- Decrease stopBand threshold

**Odometry drifts over time:**
- Verify tracking wheel circumference
- Check for wheel slippage
- Verify SIDE_OFFSET_M and VERT_OFFSET_M
- Ensure IMU is properly calibrated

**Arm stalls near limits:**
- Increase ARM_MIN_PWR
- Verify potentiometer readings match physical limits
- Check for mechanical binding

**Ball sorter ejects wrong colors:**
- Adjust hue thresholds for your lighting
- Verify optical sensor is properly positioned
- Check myAlliance setting

### Debug Techniques

**Print to Brain Screen:**
```cpp
Brain.Screen.print("Value: %f", someValue);
```

**Print to Controller:**
```cpp
Controller1.Screen.print("X: %.1f", robotPose.x);
```

**Use Odometry Display:**
Press Y during driver control to see real-time position

**Square Test:**
Run SQUARE_TEST autonomous and measure final position error

---

## Additional Resources

**Purdue SIGBots Wiki:**
Browse through the [Purdue SIGBots wiki](https://wiki.purduesigbots.com/) to learn
the heavy theoretics of odometry and the PID controller.

---

## License

This project is intended for educational and competition use in VEX Robotics competitions.

---

## Acknowledgments

Developed for VEX V5 competition robotics. Special thanks to the VEX and PROS communities for documentation and support.
