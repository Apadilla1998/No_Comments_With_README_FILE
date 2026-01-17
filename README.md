# VEX V5 Competitive Robot Control System
## Complete Technical Documentation

A production-grade C++ control system for VEX V5 competitive robotics featuring PID-based motion control, three-wheel odometry with IMU fusion, autonomous routines with adaptive position correction, and advanced operator interface.

**Version:** 1.0 | **Platform:** VEX V5 Brain | **Language:** C++11

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [System Architecture](#2-system-architecture)
3. [Hardware Configuration](#3-hardware-configuration)
4. [PID Controller](#4-pid-controller)
5. [Odometry System](#5-odometry-system)
6. [Motion Controller](#6-motion-controller)
7. [Drive System](#7-drive-system)
8. [Subsystems](#8-subsystems)
9. [Teleop Control](#9-teleop-control)
10. [Autonomous System](#10-autonomous-system)
11. [Utility Functions](#11-utility-functions)
12. [Initialization](#12-initialization)
13. [Tuning Guide](#13-tuning-guide)
14. [Troubleshooting](#14-troubleshooting)
15. [Quick Reference](#15-quick-reference)

---

## 1. Executive Summary

### Key Features

| Category | Features |
|----------|----------|
| **Motion Control** | PID-regulated driving, point turns, heading hold, auto-correction |
| **Position Tracking** | 3-wheel odometry, IMU fusion, 100Hz update rate, arc-based integration |
| **Game Piece Handling** | 3-stage intake, 3-motor outtake, optical color sorting, alliance-aware rejection |
| **Operator Interface** | Arcade drive, exponential curves, slew rate limiting, dual speed modes |
| **Autonomous** | Multiple routines, auto-correction, modular construction |

### Performance Specifications

| Metric | Value |
|--------|-------|
| Odometry Update Rate | 100 Hz (10ms) |
| Motion Control Rate | 100 Hz (10ms) |
| Teleop Control Rate | 50 Hz (20ms) |
| Position Accuracy | ±2 cm typical |
| Heading Accuracy | ±1° typical |
| Settling Time (drive) | 40ms dwell |
| Settling Time (turn) | 150ms dwell |

---

## 2. System Architecture

### 2.1 Module Hierarchy

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              APPLICATION LAYER                               │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   autons    │  │   manual    │  │  pre_auton  │  │       main          │ │
│  │ (Autonomous │  │  (Teleop    │  │   (Setup)   │  │  (Entry point)      │ │
│  │  routines)  │  │  control)   │  │             │  │                     │ │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────────────────────┘ │
└─────────┼────────────────┼────────────────┼─────────────────────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              CONTROL LAYER                                   │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        MotionController                              │    │
│  │        drive() │ turnTo() │ turnBy() │ autoCorrect()                │    │
│  │        ┌───────────┐  ┌───────────┐  ┌───────────┐                  │    │
│  │        │ distPID_  │  │ headPID_  │  │ turnPID_  │                  │    │
│  │        └───────────┘  └───────────┘  └───────────┘                  │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                       │
│  │  subsystems  │  │    odom      │  │     PID      │                       │
│  │(intake,wings)│  │ (tracking)   │  │ (controller) │                       │
│  └──────────────┘  └──────────────┘  └──────────────┘                       │
└─────────────────────────────────────────────────────────────────────────────┘
          │                │                │
          ▼                ▼                ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           HARDWARE ABSTRACTION                               │
│     drive.cpp │ sensors.h │ robot_config.cpp │ utils.h                      │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Task Model

| Task | Function | Period | Priority | Purpose |
|------|----------|--------|----------|---------|
| Main | Competition control | 100ms | Normal | Mode switching, watchdog |
| Odometry | `odomTaskFn()` | 10ms | High | Position tracking |
| Color Sorter | `intakeTaskFn()` | 20ms | Normal | Ball detection/rejection |
| Autonomous/Teleop | Foreground | 10-20ms | Normal | Active control |

### 2.3 Data Flow Diagram

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  SENSORS    │     │  ODOMETRY   │     │   MOTION    │     │  ACTUATORS  │
├─────────────┤     ├─────────────┤     ├─────────────┤     ├─────────────┤
│ IMU         │────►│             │     │             │     │             │
│ VertRot     │────►│  robotPose  │────►│ PID Control │────►│ Motors      │
│ HorizRot    │────►│  {x,y,θ}    │     │             │     │             │
│ Optical     │     │             │     │             │     │ Pneumatics  │
│ Potentiom.  │     └─────────────┘     └─────────────┘     └─────────────┘
└─────────────┘
```

---

## 3. Hardware Configuration

### 3.1 Motor Port Assignments

| Motor | Port | Ratio | Reversed | Function |
|-------|------|-------|----------|----------|
| LeftA | 3 | 6:1 (600 RPM) | No | Left drive |
| LeftB | 19 | 6:1 | No | Left drive |
| LeftC | 10 | 6:1 | No | Left drive |
| RightA | 11 | 6:1 | Yes | Right drive |
| RightB | 13 | 6:1 | Yes | Right drive |
| RightC | 15 | 6:1 | Yes | Right drive |
| IntakeLeft | 16 | 6:1 | No | Left intake roller |
| IntakeRight | 5 | 6:1 | Yes | Right intake roller |
| Midtake | 8 | 18:1 (200 RPM) | No | Transfer stage |
| LeftOutake | 2 | 6:1 | No | Left scoring |
| RightOutake | 7 | 6:1 | Yes | Right scoring |
| MidOutake | 20 | 6:1 | Yes | Center scoring |
| DescoreMotor | 18 | 36:1 (100 RPM) | Yes | Arm actuation |

### 3.2 Sensor Port Assignments

| Sensor | Port | Type | Function |
|--------|------|------|----------|
| inertial_sensor | 1 | V5 Inertial | Heading/rotation |
| verticalRot | 9 | Rotation | Forward tracking wheel |
| horizontalRot | 6 | Rotation | Lateral tracking wheel |
| ballSensor | 21 | Optical | Ball color detection |
| Descore | A (3-Wire) | Potentiometer | Arm position |
| wingsPiston | B (3-Wire) | Digital Out | Pneumatic wings |

### 3.3 Physical Constants

```cpp
namespace config {
    const double TRACK_WIDTH_M = 0.320;                    // Distance between wheels
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.05*π; // ≈0.157m (50mm wheel)
    const double SIDE_OFFSET_M = 0.0;                     // Horizontal wheel offset
    const double VERT_OFFSET_M = 0.0;                     // Vertical wheel offset
}
```

### 3.4 Wiring Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              V5 BRAIN                                        │
│  SMART PORTS (1-21)                                                         │
│  ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐       │
│  │  1  │  2  │  3  │  4  │  5  │  6  │  7  │  8  │  9  │ 10  │ 11  │       │
│  │ IMU │LOut │ LA  │     │IntR │HRot │ROut │Mid  │VRot │ LC  │ RA  │       │
│  └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘       │
│  ┌─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┬─────┐             │
│  │ 12  │ 13  │ 14  │ 15  │ 16  │ 17  │ 18  │ 19  │ 20  │ 21  │             │
│  │     │ RB  │     │ RC  │IntL │     │Desc │ LB  │MOut │ Opt │             │
│  └─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┘             │
│  3-WIRE PORTS: [A]=Potentiometer  [B]=WingsPiston                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 3.5 Coordinate System

```
        +Y (Forward)
            ▲
            │
            │       θ = 0° points along +Y
   ─────────┼─────────► +X (Right)
            │       θ increases clockwise
            │
```

| Angle | Direction |
|-------|-----------|
| 0° | Forward (+Y) |
| 90° | Right (+X) |
| 180° | Backward (-Y) |
| 270° | Left (-X) |

---

## 4. PID Controller

### 4.1 Theory

The PID controller computes output based on error between setpoint and measurement:

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·(de/dt) + feedforward
```

### 4.2 Features Summary

| Feature | Purpose | Configuration |
|---------|---------|---------------|
| **Derivative Modes** | Eliminate derivative kick | `OnError` or `OnMeasurement` |
| **Derivative Filter** | Reduce noise | `setDerivativeFilterTf(seconds)` |
| **Integral Zone** | Prevent overshoot | `setIntegralZone(range)` |
| **Integral Limits** | Prevent windup | `setIntegralLimits(min, max)` |
| **Anti-Windup** | Back-calculation | `setAntiWindupTau(seconds)` |
| **Error Deadband** | Prevent oscillation | `setErrorDeadband(threshold)` |
| **Feed-Forward** | Base output | `setFeedForward(value)` |
| **Bumpless Transfer** | Smooth transitions | `resetBumpless(meas, output)` |

### 4.3 Complete API

```cpp
// Constructor
PID(double kp = 0.0, double ki = 0.0, double kd = 0.0);

// Configuration
void setGains(double kp, double ki, double kd);
void setSetpoint(double sp);
void setOutputLimits(double outMin, double outMax);      // Default: -100, 100
void setIntegralLimits(double iMin, double iMax);        // Default: -1e9, 1e9
void setIntegralZone(double zone);                       // Default: 0 (disabled)
void setDerivativeFilterTf(double tfSec);                // Default: 0 (no filter)
void setDerivativeMode(DerivativeMode mode);             // Default: OnMeasurement
void setFeedForward(double ff);                          // Default: 0
void setAntiWindupTau(double tauSec);                    // Default: 0.2
void setErrorDeadband(double deadband);                  // Default: 0

// State Access
double getSetpoint() const;
double getError() const;
double getOutput() const;
double getIntegral() const;

// Reset
void reset(double measurement = 0.0);
void resetBumpless(double measurement, double holdOutput = 0.0);

// Update (call every loop iteration)
double update(double measurement, double dtSec);
```

### 4.4 Derivative Modes Explained

**OnError (Traditional):**
```cpp
D = Kd × (error[n] - error[n-1]) / dt
```
- Responds to setpoint changes → causes "derivative kick"

**OnMeasurement (Recommended):**
```cpp
D = -Kd × (measurement[n] - measurement[n-1]) / dt
```
- Ignores setpoint changes → smooth response, no kick

### 4.5 Anti-Windup Back-Calculation

When output saturates, integral is unwound:
```cpp
delta = saturated_output - unsaturated_output
integral += (delta / Ki) × (dt / awTau)
```

### 4.6 Usage Example

```cpp
PID distancePID(0.25, 0.0, 0.002);
distancePID.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
distancePID.setDerivativeFilterTf(0.10);
distancePID.setOutputLimits(-100, 100);
distancePID.setSetpoint(1.0);  // Target: 1 meter
distancePID.reset(0.0);

// Control loop
while (!done) {
    double position = getEncoderMeters();
    double power = distancePID.update(position, 0.010);  // 10ms dt
    setMotorPower(power);
    wait(10, msec);
}
```

---

## 5. Odometry System

### 5.1 Overview

Three-wheel odometry with IMU fusion:
- **Vertical wheel**: Measures forward/backward displacement
- **Horizontal wheel**: Measures lateral displacement
- **IMU**: Provides heading (resistant to wheel slip)

### 5.2 Mathematical Model

**Sensor compensation for rotation:**
```cpp
dVc = dV - VERT_OFFSET × dθ    // Compensated vertical
dHc = dH - SIDE_OFFSET × dθ    // Compensated horizontal
```

**Arc-based integration:**
```cpp
if (|dθ| < 1e-6) {
    lX = dHc;  lY = dVc;       // Linear approximation
} else {
    chord = 2 × sin(dθ/2);
    lX = chord × (dHc / dθ);   // Arc geometry
    lY = chord × (dVc / dθ);
}
```

**Global transformation:**
```cpp
avgθ = θ + dθ/2;
x += lX × cos(avgθ) + lY × sin(avgθ);
y += lY × cos(avgθ) - lX × sin(avgθ);
θ += dθ;
```

### 5.3 API Reference

```cpp
// Global pose variable
extern Pose robotPose;  // {x, y, theta} in meters and radians

struct Pose {
    double x;      // Lateral position (+ = right)
    double y;      // Forward position (+ = forward)
    double theta;  // Heading in radians (+ = clockwise)
};

// Functions
int odomTaskFn();           // Background task (100Hz)
void resetOdometry();       // Reset pose to (0,0,0), zero sensors
```

### 5.4 Error Sources

| Source | Cause | Mitigation |
|--------|-------|------------|
| Wheel slip | Fast acceleration | Reduce accel, clean wheels |
| IMU drift | Sensor physics | Periodic reset, short routines |
| Wheel diameter error | Manufacturing | Measure actual circumference |
| Offset calibration | Measurement error | Careful physical measurement |

---

## 6. Motion Controller

### 6.1 Architecture

```cpp
class MotionController {
public:
    // Utility
    double wrap180(double a);
    double angleDiffDeg(double targetDeg, double currentDeg);

    // Basic motions
    void drive(double distM, int timeoutMs, double maxSpeedPct = 80.0);
    void turnTo(double targetDeg, int timeoutMs);
    void turnBy(double deltaDeg, int timeoutMs);

    // Auto-correction
    void autoCorrect(double targetX, double targetY, double targetHeadingDeg,
                     int timeoutMs, double maxSpeedPct = 45.0);

    // AC wrappers
    void driveAC(double distM, int timeoutMs, double maxSpeedPct = 80.0,
                 int correctTimeoutMs = 1200, double correctSpeedPct = 45.0);
    void turnToAC(double targetDeg, int timeoutMs,
                  int correctTimeoutMs = 900, double correctSpeedPct = 40.0);
    void turnByAC(double deltaDeg, int timeoutMs,
                  int correctTimeoutMs = 900, double correctSpeedPct = 40.0);

    // Configuration
    void setAutoCorrectEnabled(bool en);
    bool getAutoCorrectEnabled() const;

private:
    PID distPID_, headPID_, turnPID_;
    bool autoCorrectEnabled_ = true;
};
```

### 6.2 PID Configurations

| Controller | Kp | Ki | Kd | Special Settings |
|------------|-----|-----|-----|------------------|
| distPID_ | 0.25 | 0.0 | 0.002 | Deadband: 0.01m, Filter: 100ms |
| headPID_ | 0.25 | 0.0 | 0.002 | Filter: 50ms, I-limits: ±2 |
| turnPID_ | 0.25 | 0.0 | 0.002 | I-zone: 15°, Deadband: 0.3° |

### 6.3 drive() Algorithm

1. Record start encoder position and heading
2. Loop at 10ms:
   - Calculate distance error
   - Dynamic speed cap: `cap = min(10 + 200×|error|, maxSpeed)`
   - Distance PID → forward velocity
   - Heading PID → turn correction
   - Apply slew rate limiting
   - `tankDrive(v + w, v - w)`
3. Settle when |error| < 2cm AND |velocity| < 6% for 40ms
4. Exit on settle or timeout

### 6.4 turnTo() / turnBy() Algorithm

1. Use IMU heading (turnTo) or rotation (turnBy)
2. Loop at 10ms:
   - Calculate angular error
   - Turn PID → turn output
   - Filter angular rate for settling detection
   - `tankDrive(out, -out)`
3. Settle when |error| < 1° AND |rate| < 8°/s for 150ms
4. Exit on settle or timeout

### 6.5 Auto-Correction System

**Purpose:** Compensate for drift/errors after movements

**Thresholds:**
| Parameter | Enter | Exit |
|-----------|-------|------|
| Distance | 0.12m | 0.07m |
| Angle | 6° | 3° |
| Position | 0.06m | 0.03m |

**Algorithm:**
1. Check if within exit thresholds → exit if yes
2. Correct heading if error > 6°
3. Decompose position error into robot frame (fwd/lat)
4. Correct lateral error: turn 90°, drive, turn back
5. Correct forward error: drive
6. Repeat up to 2 iterations

---

## 7. Drive System

### 7.1 Motor Groups

```cpp
motor_group LeftMotorGroup(LeftA, LeftB, LeftC);
motor_group RightMotorGroup(RightA, RightB, RightC);
```

### 7.2 API

```cpp
void tankDrive(double leftPct, double rightPct);  // Direct control
void arcadeDrive(double fwdPct, double turnPct);  // Arcade mixing
void stopDrive(brakeType mode = brake);           // Stop motors
```

### 7.3 Brake Modes

| Mode | Behavior | Use Case |
|------|----------|----------|
| `coast` | Freewheel | Smooth stop |
| `brake` | Active resistance | Quick stop |
| `hold` | PID position hold | Precise stop |

---

## 8. Subsystems

### 8.1 Intake System

```cpp
void runIntake(double speedPct = 100.0);      // Forward
void reverseIntake(double speedPct = 100.0);  // Reverse
void stopIntake();                             // Stop (coast)
```

**Motors:** IntakeLeft, IntakeRight, Midtake (3-stage)

### 8.2 Outtake System

```cpp
void runOutake(double speedPct = 100.0);      // Forward (score)
void reverseOutake(double speedPct = 100.0);  // Reverse
void stopOutake();                             // Stop (coast)
```

**Motors:** LeftOutake, RightOutake, MidOutake

### 8.3 Wings (Pneumatics)

```cpp
class Wings {
    void toggle();                 // Switch state
    void set(bool s);              // Set directly
    bool isExtended() const;       // Query state
};
extern Wings wings;
```

### 8.4 Descore Arm

**Control:** PD control with velocity damping

```cpp
output = (ARM_KP × error) - (ARM_KD × velocity)
```

**Constants:**
| Parameter | Value |
|-----------|-------|
| ARM_UP_DEG | 77° |
| ARM_DOWN_DEG | 191° |
| ARM_KP | 0.77 |
| ARM_KD | 0.1 |
| ARM_DEADBAND | 2° |
| ARM_MIN_PWR | 5% |
| ARM_MAX_PWR | 80% |
| HARD_MARGIN | 2° |
| SOFT_ZONE | 20° |

**Safety features:**
- Hard limits with margin
- Soft zone power limiting (quadratic curve)
- Deadband at target
- Minimum power floor

### 8.5 Color Sorting

**Hue Detection:**
| Color | Hue Range |
|-------|-----------|
| Red | < 20° OR > 340° |
| Blue | 200° - 250° |

**Algorithm:**
1. Optical sensor detects nearby object
2. Median filter (5 samples) smooths hue
3. If opposing alliance → reverse 300ms → resume

```cpp
extern Alliance myAlliance;  // RED or BLUE
```

---

## 9. Teleop Control

### 9.1 Controller Mapping

| Control | Function |
|---------|----------|
| Axis 3 | Forward/backward |
| Axis 1 | Turn left/right |
| R1 (toggle) | Speed mode |
| Up (toggle) | Wings |
| X (momentary) | Reset odometry |
| Y (toggle) | Display mode |
| Left (hold) | Arm up |
| Right (hold) | Arm down |
| L1 (hold) | Run intake |
| L2 (hold) | Intake + outtake forward |
| R2 (hold) | Reverse all |

### 9.2 Joystick Processing

**Pipeline:** Raw → Deadband → Curve → Scale → Slew → Motors

**Deadband:**
```cpp
if (|input| < 5.0) input = 0;
```

**Exponential Curve:**
```cpp
output = (curve × input³ + (1 - curve) × input) × 100
// FWD_CURVE = 0.1, TURN_CURVE = 0.3
```

**Speed Scaling:**
| Mode | Scale |
|------|-------|
| Fast | 0.8 (80%) |
| Slow | 0.4 (40%) |

**Motor Biasing:**
```cpp
LEFT_BIAS = 0.98, RIGHT_BIAS = 1.0  // Compensate for drift
```

### 9.3 Slew Rate Limiting

```cpp
ACCEL_PCT_PER_S = 300  // Max 6% change per 20ms loop
DECEL_PCT_PER_S = 300
```

### 9.4 Display Modes

**Normal (every 2s):**
```
A:191  T:191     (Actual, Target)
SPEED: FAST
WINGS: DOWN
```

**Odom (every 200ms):**
```
X:  50.0 cm
Y: 100.0 cm
T:  90.0 deg
```

---

## 10. Autonomous System

### 10.1 Routine Selection

```cpp
enum class AutonRoutine {
    NONE, TEST, RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT,
    SKILLS, TURN_TEST, DRIVE_TEST, SQUARE_TEST,
    AUTO_CORRECT_BLUE_RIGHT, AUTO_CORRECT_RED_RIGHT
};

extern AutonRoutine selectedAuton;
```

### 10.2 Example Routine

```cpp
static void autoCorrectBlueRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    m.driveAC(0.82, 4000, 80);    // Drive 82cm
    m.turnTo(90, 4000);            // Turn to 90°
    m.driveAC(0.30, 4000, 80);    // Drive 30cm

    runIntake(100);
    wait(5, sec);
    stopIntake();

    m.driveAC(-0.40, 4000, 80);   // Back up 40cm
    m.turnBy(180, 4000);           // Turn around

    wings.toggle();
    m.driveAC(0.40, 4000, 80);    // Push forward

    runIntake(100); runOutake(100);
    wait(2000, msec);
    stopIntake(); stopOutake();
}
```

---

## 11. Utility Functions

### Mathematical (utils.h)

```cpp
double clampD(double v, double lo, double hi);  // Clamp to range
double clampPct(double v);                       // Clamp to [-100, 100]
double degToRad(double deg);                     // Degrees → radians
double radToDeg(double rad);                     // Radians → degrees
double wrap180(double deg);                      // Normalize to (-180, 180]
```

### Sensor (sensors.h)

```cpp
void initSensors();           // Calibrate IMU, reset encoders
double headingDeg();          // IMU heading [0, 360)
double rotationDeg();         // IMU rotation (unbounded)
double verticalDeg();         // Vertical encoder
double horizontalDeg();       // Horizontal encoder
```

---

## 12. Initialization

### Boot Sequence

1. **main()**: Register competition callbacks
2. **pre_auton()**: 
   - `initSensors()` - Calibrate IMU (~2-3s), reset encoders
   - `resetOdometry()` - Zero pose
   - Start odomTask and sorterTask
   - Display "Sensors ready"
3. **Main loop**: Wait for competition switch

```cpp
void pre_auton() {
    initSensors();
    resetOdometry();
    if (!odomTask)   odomTask   = new task(odomTaskFn);
    if (!sorterTask) sorterTask = new task(intakeTaskFn);
    Brain.Screen.print("Sensors ready");
}
```

---

## 13. Tuning Guide

### 13.1 PID Tuning Steps

1. Set Ki = 0, Kd = 0
2. Increase Kp until oscillation
3. Reduce Kp by 20-30%
4. Add Kd to dampen (start at Kp/10)
5. Add Ki only if steady-state error (start at Kp/100)

### 13.2 Typical Values for VEX

| Application | Kp | Ki | Kd |
|-------------|-----|-----|-----|
| Distance (m) | 0.2-0.5 | 0-0.05 | 0.001-0.01 |
| Angle (°) | 0.2-0.5 | 0-0.02 | 0.001-0.01 |

### 13.3 Odometry Calibration

**Wheel circumference:**
1. Mark wheel, push robot 1m
2. `circumference = 1.0 / rotations`

**Track width:**
1. Turn 10 complete rotations
2. `track_width = encoder_travel / (10 × 2π)`

### 13.4 Arm Tuning

| Symptom | Solution |
|---------|----------|
| Slow | Increase ARM_KP |
| Overshoots | Increase ARM_KD |
| Oscillates | Decrease ARM_KP, increase ARM_KD |

---

## 14. Troubleshooting

### 14.1 Common Issues

| Symptom | Causes | Solutions |
|---------|--------|-----------|
| Robot drifts | Motor imbalance, friction | Adjust LEFT_BIAS/RIGHT_BIAS |
| Jerky movement | High slew, PID oscillation | Reduce accel, tune PID |
| Position drifts | Wheel slip, bad calibration | Recalibrate, reduce accel |
| Heading wrong | IMU not calibrated | Ensure stationary during cal |
| Overshoots | Kp too high, Kd too low | Reduce Kp, increase Kd |
| Doesn't reach | Kp too low, friction | Increase Kp, min power |
| Arm won't move | Outside limits, motor hot | Check limits, let cool |
| Color sort fails | Light off, sensor far | Check setLightPower, position |

### 14.2 Diagnostic Tests

**Motor test:**
```cpp
MotorA.spin(fwd, 50, pct);  // Test each individually
```

**Sensor test:**
```cpp
Brain.Screen.print("Heading: %.1f", headingDeg());
```

**Square test:**
```cpp
drive(1.0); turnBy(90);  // Repeat 4x, should return to start
```

---

## 15. Quick Reference

### Port Summary
```
DRIVE:  L[3,19,10]  R[11,13,15]
INTAKE: L[16] R[5] M[8]
OUTAKE: L[2] R[7] M[20]
ARM:    [18]
IMU:[1] VROT:[9] HROT:[6] OPT:[21] POT:[A] PISTON:[B]
```

### Key Constants
```
Track Width:     0.320 m
Wheel Circum:    0.157 m
Arm Up/Down:     77° / 191°
PID (all):       Kp=0.25, Ki=0, Kd=0.002
```

### Controls Quick Reference
```
Axis3/1: Drive/Turn    R1: Speed    Up: Wings
X: Reset odom          Y: Display   L/R: Arm
L1: Intake             L2: Intake+Out    R2: Reverse
```

### File Structure
```
include/: autons.h, drive.h, manual.h, motion.h, odom.h, 
          PID.h, pre_auton.h, robot_config.h, sensors.h,
          subsystems.h, utils.h, vex.h
src/:     autons.cpp, drive.cpp, main.cpp, manual.cpp,
          motion.cpp, odom.cpp, pre_auton.cpp,
          robot_config.cpp, subsystems.cpp
```

---

## Dependencies

- VEX V5 SDK (v5.h, v5_vcs.h)
- Standard C++: `<cmath>`, `<algorithm>`, `<vector>`

## Build & Deploy

1. Open in VEXcode Pro V5 or PROS
2. Build project
3. Download to V5 Brain
4. Set autonomous routine
5. Connect controller

---

*Documentation for VEX V5 Competition Robot | Last updated: January 2026*
