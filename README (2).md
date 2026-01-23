# VEX V5 Competition Robot Codebase

A sophisticated autonomous robotics control system for VEX V5 competition robots, featuring advanced PID control, three-wheel odometry tracking, motion planning with auto-correction, and comprehensive subsystem management.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [File Structure](#file-structure)
4. [Core Components](#core-components)
   - [PID Controller](#pid-controller)
   - [Odometry System](#odometry-system)
   - [Motion Controller](#motion-controller)
   - [Drive System](#drive-system)
   - [Subsystems](#subsystems)
5. [Robot Configuration](#robot-configuration)
6. [Autonomous Routines](#autonomous-routines)
7. [Manual Control](#manual-control)
8. [Tuning Guide](#tuning-guide)
9. [Usage](#usage)
10. [Troubleshooting](#troubleshooting)

---

## Overview

This codebase implements a complete control system for a VEX V5 competition robot. The system is designed for high-precision autonomous navigation and responsive manual control during driver-operated periods.

### Key Features

- **Advanced PID Control**: Full-featured PID implementation with derivative filtering, anti-windup, integral zones, and bumpless transfer
- **Three-Wheel Odometry**: Real-time position tracking using perpendicular tracking wheels and an IMU
- **Motion Planning**: Multiple drive modes including heading-hold, closed-loop correction, and auto-correction
- **Auto-Correction System**: Post-movement position correction using odometry feedback
- **Subsystem Integration**: Coordinated control of intake, outtake, wings (pneumatics), and scoring mechanisms
- **Color Sorting**: Optical sensor-based ball sorting for alliance-specific game pieces
- **Slew Rate Control**: Smooth acceleration/deceleration for drivetrain and mechanisms

### Hardware Requirements

| Component | Quantity | Purpose |
|-----------|----------|---------|
| V5 Smart Motors (600 RPM) | 6 | Drivetrain (3L + 3R) |
| V5 Smart Motors (600 RPM) | 5 | Intake/Outtake mechanisms |
| V5 Smart Motor (100 RPM) | 1 | Descore arm |
| V5 Inertial Sensor | 1 | Heading/rotation tracking |
| V5 Rotation Sensors | 2 | Odometry tracking wheels |
| V5 Optical Sensor | 1 | Ball color detection |
| 3-Wire Digital Output | 1 | Pneumatic wings |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         MAIN PROGRAM                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────────┐  │
│  │  pre_auton  │  │ autonomous  │  │       usercontrol           │  │
│  │  (setup)    │  │  (autons)   │  │       (manual)              │  │
│  └──────┬──────┘  └──────┬──────┘  └──────────────┬──────────────┘  │
└─────────┼────────────────┼───────────────────────┼──────────────────┘
          │                │                       │
          ▼                ▼                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      CONTROL LAYER                                  │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │                   MotionController                          │    │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌──────────────┐   │    │
│  │  │ distPID │  │ headPID │  │ turnPID │  │ autoCorrect  │   │    │
│  │  └─────────┘  └─────────┘  └─────────┘  └──────────────┘   │    │
│  └─────────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────────┘
          │                │                       │
          ▼                ▼                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      SENSING LAYER                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────────┐  │
│  │   Odometry  │  │   Sensors   │  │      Color Sorting          │  │
│  │   (odom)    │  │  (sensors)  │  │      (intakeTask)           │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
          │                │                       │
          ▼                ▼                       ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      HARDWARE LAYER                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────────┐  │
│  │ Drivetrain  │  │ Subsystems  │  │      robot_config           │  │
│  │   (drive)   │  │ (subsystems)│  │    (hardware defs)          │  │
│  └─────────────┘  └─────────────┘  └─────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

### Concurrent Tasks

The system runs multiple concurrent tasks for real-time operation:

| Task | Update Rate | Purpose |
|------|-------------|---------|
| Odometry | 10ms (100 Hz) | Position tracking |
| Intake/Sorter | 20ms (50 Hz) | Color-based ball sorting |
| Main Loop | 20ms (50 Hz) | Control and user input |

---

## File Structure

```
project/
├── include/
│   ├── vex.h              # VEX SDK includes and macros
│   ├── robot_config.h     # Hardware definitions and constants
│   ├── PID.h              # PID controller class
│   ├── odom.h             # Odometry system interface
│   ├── motion.h           # Motion controller class
│   ├── drive.h            # Low-level drive functions
│   ├── subsystems.h       # Mechanism control interfaces
│   ├── sensors.h          # Sensor utility functions
│   ├── utils.h            # Math and helper utilities
│   ├── autons.h           # Autonomous routine declarations
│   ├── manual.h           # Manual control interface
│   └── pre_auton.h        # Pre-autonomous setup
│
└── src/
    ├── main.cpp           # Program entry point
    ├── robot_config.cpp   # Hardware instantiation
    ├── odom.cpp           # Odometry implementation
    ├── motion.cpp         # Motion control algorithms
    ├── drive.cpp          # Drive functions
    ├── subsystems.cpp     # Mechanism implementations
    ├── autons.cpp         # Autonomous routines
    ├── manual.cpp         # Teleop control loop
    └── pre_auton.cpp      # Initialization routines
```

---

## Core Components

### PID Controller

The `PID` class (`PID.h`) implements a full-featured proportional-integral-derivative controller with advanced features for robust control in real-world robotics applications.

#### Theoretical Background

PID control is a feedback mechanism that continuously calculates an error value as the difference between a desired setpoint and a measured process variable. The controller attempts to minimize the error by adjusting the process control inputs.

The standard PID equation is:

```
u(t) = Kp·e(t) + Ki·∫₀ᵗ e(τ)dτ + Kd·(de/dt)
```

Where:
- `u(t)` is the control output
- `e(t)` is the error (setpoint - measurement)
- `Kp` is the proportional gain
- `Ki` is the integral gain
- `Kd` is the derivative gain

Each term serves a specific purpose:

| Term | Purpose | Effect of Increasing |
|------|---------|---------------------|
| **Proportional (P)** | Responds to current error | Faster response, but may oscillate |
| **Integral (I)** | Eliminates steady-state error | Removes offset, but adds overshoot |
| **Derivative (D)** | Predicts future error | Dampens oscillations, but amplifies noise |

#### Class Interface

```cpp
class PID {
public:
    enum class DerivativeMode { OnError, OnMeasurement };

    // Constructor
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0);

    // Gain configuration
    void setGains(double kp, double ki, double kd);

    // Setpoint management
    void setSetpoint(double sp);
    double getSetpoint() const;

    // Output limiting
    void setOutputLimits(double outMin, double outMax);

    // Integral management
    void setIntegralLimits(double iMin, double iMax);
    void setIntegralZone(double zone);

    // Derivative configuration
    void setDerivativeFilterTf(double tfSec);
    void setDerivativeMode(DerivativeMode mode);

    // Advanced features
    void setFeedForward(double ff);
    void setAntiWindupTau(double tauSec);
    void setErrorDeadband(double deadband);

    // State accessors
    double getError() const;
    double getOutput() const;
    double getIntegral() const;

    // Reset methods
    void reset(double measurement = 0.0);
    void resetBumpless(double measurement, double holdOutput = 0.0);

    // Main update function
    double update(double measurement, double dtSec);
};
```

#### Advanced Features Explained

##### 1. Derivative Mode Selection

The derivative term can be calculated two ways:

**Derivative on Error (Standard)**:
```cpp
dSignal = (error - prevError) / dt;
```
- Responds to setpoint changes immediately
- Can cause "derivative kick" when setpoint changes suddenly
- Best when setpoint changes are smooth

**Derivative on Measurement**:
```cpp
dSignal = -(measurement - prevMeasurement) / dt;
```
- Ignores setpoint changes for derivative calculation
- Prevents derivative kick on setpoint changes
- **Recommended for most robotics applications**

```cpp
// Usage
pid.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
```

##### 2. Derivative Low-Pass Filter

Derivative amplifies high-frequency noise. A first-order low-pass filter smooths the derivative signal:

```cpp
// Filter equation
alpha = dt / (Tf + dt);
dFiltered += alpha * (dSignal - dFiltered);
```

Where `Tf` is the filter time constant. Larger values = more smoothing but slower response.

```cpp
// Typical values: 0.02 - 0.15 seconds
pid.setDerivativeFilterTf(0.06);  // 60ms filter
```

**Frequency Response**:
| Tf (sec) | Cutoff Frequency | Use Case |
|----------|------------------|----------|
| 0.02 | ~8 Hz | Fast response, minimal filtering |
| 0.06 | ~2.7 Hz | Balanced filtering |
| 0.10 | ~1.6 Hz | Heavy filtering for noisy sensors |
| 0.15 | ~1.1 Hz | Very noisy environments |

##### 3. Integral Zone

The integral zone limits when the integral term accumulates. Integration only occurs when `|error| < iZone`:

```cpp
if (iZone == 0.0 || fabs(error) < iZone) {
    integral += error * dt;
}
```

Benefits:
- Prevents integral windup during large errors
- Integral only "kicks in" close to setpoint
- Useful for systems that travel far from setpoint

```cpp
// Only integrate within 15 degrees of target
turnPID.setIntegralZone(15.0);
```

##### 4. Integral Limits (Clamping)

Hard limits on the integral accumulator prevent extreme windup:

```cpp
integral = clamp(integral, iMin, iMax);
```

```cpp
// Limit integral contribution
pid.setIntegralLimits(-50.0, 50.0);
```

##### 5. Anti-Windup (Back-Calculation)

When output saturates, the integral term is adjusted to prevent continued windup:

```cpp
if (output != outputUnsaturated) {
    // Back-calculate integral correction
    delta = (outputSaturated - outputUnsaturated);
    integral += (delta / Ki) * (dt / tau);
}
```

The time constant `tau` controls how quickly anti-windup acts:
- Smaller tau = faster anti-windup
- Larger tau = gentler correction

```cpp
// Typical values: 0.1 - 0.3 seconds
pid.setAntiWindupTau(0.18);
```

##### 6. Error Deadband

Small errors within the deadband are treated as zero:

```cpp
if (fabs(error) < deadband) error = 0.0;
```

Benefits:
- Prevents oscillation around setpoint due to noise
- Reduces motor "hunting" at target
- Conserves power when "close enough"

```cpp
// Ignore errors smaller than 6mm
distPID.setErrorDeadband(0.006);

// Ignore errors smaller than 0.2 degrees
turnPID.setErrorDeadband(0.2);
```

##### 7. Feed Forward

Adds a constant offset to the output, useful for:
- Gravity compensation
- Friction compensation
- Known system biases

```cpp
output = feedForward + P + I + D;
```

```cpp
// Compensate for gravity on an arm
pid.setFeedForward(15.0);  // 15% to hold position
```

##### 8. Bumpless Transfer

When switching between manual and automatic control, or changing setpoints, bumpless transfer prevents output "bumps":

```cpp
void resetBumpless(double measurement, double holdOutput) {
    error = setpoint - measurement;
    
    // Back-calculate integral to maintain current output
    integral = (holdOutput - feedForward - (Kp * error)) / Ki;
    integral = clamp(integral, iMin, iMax);
    
    prevMeas = measurement;
    prevError = error;
    dFiltered = 0.0;
}
```

Use cases:
- Switching from manual to autonomous control
- Engaging a controller that was previously disabled
- Smooth setpoint changes

```cpp
// Engage controller smoothly, maintaining 30% output
pid.resetBumpless(currentPosition, 30.0);
```

#### Control Loop Implementation Details

The `update()` function implements the complete PID algorithm:

```cpp
double update(double measurement, double dtSec) {
    // Ensure minimum dt to prevent division by zero
    dtSec = std::max(dtSec, 1e-6);

    // Calculate error with deadband
    double error = setpoint_ - measurement;
    if (errDeadband_ > 0.0 && std::fabs(error) < errDeadband_) 
        error = 0.0;
    lastError_ = error;

    // Derivative calculation based on mode
    double dSignal = 0.0;
    if (!first_) {
        if (dMode_ == DerivativeMode::OnMeasurement) {
            // Derivative on measurement (no kick on setpoint change)
            const double dMeas = (measurement - prevMeas_) / dtSec;
            dSignal = -dMeas;
        } else {
            // Standard derivative on error
            const double dErr = (error - prevError_) / dtSec;
            dSignal = dErr;
        }
    }

    // Apply derivative low-pass filter
    if (first_) {
        dFilt_ = 0.0;
        first_ = false;
    } else if (dFilterTf_ > 0.0) {
        const double alpha = dtSec / (dFilterTf_ + dtSec);
        dFilt_ += alpha * (dSignal - dFilt_);
    } else {
        dFilt_ = dSignal;
    }

    // Integral accumulation with zone limiting
    const bool canIntegrate = (std::fabs(kI_) > 1e-12);
    if (canIntegrate && (iZone_ == 0.0 || std::fabs(error) < iZone_)) {
        integral_ += error * dtSec;
        integral_ = clamp(integral_, iMin_, iMax_);
    }

    // Calculate PID terms
    const double P = kP_ * error;
    const double I = kI_ * integral_;
    const double D = kD_ * dFilt_;
    
    // Sum with feed forward
    const double outUnsat = feedForward_ + P + I + D;
    const double outSat = clamp(outUnsat, outMin_, outMax_);

    // Back-calculation anti-windup
    if (canIntegrate && awTau_ > 0.0) {
        const double delta = (outSat - outUnsat);
        integral_ += (delta / kI_) * (dtSec / awTau_);
        integral_ = clamp(integral_, iMin_, iMax_);
    }

    // Store state for next iteration
    prevMeas_ = measurement;
    prevError_ = error;
    lastOutput_ = outSat;
    
    return outSat;
}
```

#### Complete Usage Example

```cpp
// Create and configure a position controller
PID positionPID(0.38, 0.01, 0.04);

// Derivative configuration
positionPID.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
positionPID.setDerivativeFilterTf(0.08);

// Integral configuration
positionPID.setIntegralZone(0.10);        // Only integrate within 10cm
positionPID.setIntegralLimits(-30, 30);   // Limit integral contribution
positionPID.setAntiWindupTau(0.15);       // Fast anti-windup

// Output configuration
positionPID.setOutputLimits(-100, 100);
positionPID.setErrorDeadband(0.005);      // 5mm deadband

// Set target and initialize
positionPID.setSetpoint(1.5);  // 1.5 meters
positionPID.reset(currentPosition);

// Control loop
while (!atTarget) {
    double measurement = getPosition();
    double output = positionPID.update(measurement, 0.010);  // 10ms dt
    setMotorPower(output);
    wait(10, msec);
}
```

#### Implementation Details from Code

The actual PID values used in this codebase:

```cpp
// Distance PID - controls forward/backward movement
distPID_(0.38, 0.0, 0.0041)
// Note: No integral term - relies on proportional + derivative only
// Very small derivative for damping without noise amplification

// Heading Hold PID - maintains heading during straight drives  
headPID_(0.25, 0.001, 0.001)
// Small integral to correct persistent heading drift
// Small derivative for stability
// Output limits ±16% to avoid aggressive corrections

// Turn PID - controls point turns
turnPID_(0.25, 0.001, 0.001)
// Integral zone of 15° - only integrates when close
// Higher integral limits (±50) for eliminating turn error
```

---

### Odometry System

The odometry system (`odom.h`, `odom.cpp`) provides real-time robot position tracking using dead reckoning with two perpendicular tracking wheels and an IMU for heading measurement.

#### Theoretical Background

Odometry estimates position by integrating wheel encoder data over time. The basic principle:

1. **Measure wheel rotations** → calculate distance traveled
2. **Measure heading change** → determine direction of travel
3. **Integrate position** → accumulate x, y coordinates

The challenge is handling rotation during movement. When a robot rotates while moving, it follows an arc, not a straight line.

#### Coordinate System

```
                +Y (Forward)
                 ▲
                 │
                 │
            ◄────┼────►  +X (Right)
                 │
                 │
                 ▼
        
   θ = 0° pointing +Y (forward)
   θ increases counter-clockwise (standard mathematical convention)
   
   Robot reference frame:
   ┌─────────────────┐
   │        ▲        │
   │        │ +Y     │
   │        │        │
   │   ◄────┼────►   │
   │   +X   │        │
   │        │        │
   │    [H]─┼─[V]    │  H = Horizontal tracking wheel
   │        │        │  V = Vertical tracking wheel
   └────────┴────────┘
```

#### Tracking Wheel Configuration

This system uses a **two-wheel + IMU** configuration:

| Component | Purpose | Measurement |
|-----------|---------|-------------|
| Vertical Wheel | Forward/backward movement | Y-axis displacement |
| Horizontal Wheel | Lateral (sideways) movement | X-axis displacement |
| IMU | Absolute heading | θ (rotation angle) |

**Why use an IMU instead of a third wheel?**
- More accurate heading (not affected by wheel slip)
- Drift correction built into sensor
- Simpler mechanical design

#### Pose Structure

```cpp
struct Pose {
    double x;      // X position in meters (positive = right)
    double y;      // Y position in meters (positive = forward)
    double theta;  // Heading in radians (positive = counter-clockwise)
};

extern Pose robotPose;  // Global pose, updated at 100Hz
```

#### Algorithm Deep Dive

##### Step 1: Sensor Reading

```cpp
// Convert encoder degrees to meters
inline double degToMeters(double deg) {
    return (deg * config::TRACKING_WHEEL_CIRCUMFERENCE_M) / 360.0;
}

const double v_m = degToMeters(verticalDeg());    // Vertical wheel
const double h_m = degToMeters(horizontalDeg());  // Horizontal wheel
const double rotDeg = rotationDeg();               // IMU rotation
```

##### Step 2: Calculate Deltas

```cpp
const double dV = v_m - prevV_m;      // Change in vertical
const double dH = h_m - prevH_m;      // Change in horizontal
const double dRotDeg = rotDeg - prevRotDeg;  // Change in heading
const double dT = degToRad(dRotDeg);  // Convert to radians
```

##### Step 3: Tracking Wheel Offset Compensation

Tracking wheels are typically not at the robot's center. When the robot rotates, the wheels trace an arc even if the robot center doesn't move:

```cpp
// Compensate for wheel offset from rotation center
const double dHc = dH - config::SIDE_OFFSET_M * dT;
const double dVc = dV - config::VERT_OFFSET_M * dT;
```

Where:
- `SIDE_OFFSET_M` = horizontal wheel distance from center (positive if right of center)
- `VERT_OFFSET_M` = vertical wheel distance from center (positive if forward of center)

**Visual explanation**:
```
Rotation causes tracking wheel to move even if robot center is stationary:

        [Center]
           │
           │ offset
           │
        [Wheel]───► Arc during rotation

Arc length = offset × rotation_angle
```

##### Step 4: Arc Integration

For small rotations, movement can be approximated as straight lines. For larger rotations, we use arc-based integration:

**Small rotation (|dθ| < ε)**:
```cpp
// Straight line approximation
localX = dHc;
localY = dVc;
```

**Significant rotation**:
```cpp
// Arc approximation using chord length
const double chord = 2.0 * sin(dT / 2.0);
localX = chord * (dHc / dT);
localY = chord * (dVc / dT);
```

**Why the chord formula?**

When traveling distance `d` along an arc with angle `θ`, the straight-line displacement is:
```
chord = 2 × r × sin(θ/2)
     = 2 × (d/θ) × sin(θ/2)
     = d × (2 × sin(θ/2) / θ)
```

This corrects for the fact that arc length ≠ straight-line distance.

##### Step 5: Transform to Global Frame

The local displacements (in robot frame) must be rotated to global coordinates:

```cpp
// Use average heading during movement for more accuracy
const double avgT = theta + 0.5 * dT;

// Rotation matrix transformation
robotPose.x += localX * cos(avgT) + localY * sin(avgT);
robotPose.y += localY * cos(avgT) - localX * sin(avgT);

// Update heading
theta += dT;
robotPose.theta = theta;
```

**Rotation matrix**:
```
┌ globalX ┐   ┌ cos(θ)   sin(θ) ┐   ┌ localX ┐
│         │ = │                 │ × │        │
└ globalY ┘   └ -sin(θ)  cos(θ) ┘   └ localY ┘
```

Note: This assumes Y-forward convention. The signs differ from standard mathematical rotation.

#### Complete Algorithm Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    ODOMETRY UPDATE CYCLE                        │
│                       (every 10ms)                              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │  Read Sensors   │
                    │  - Vertical enc │
                    │  - Horiz enc    │
                    │  - IMU rotation │
                    └────────┬────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ Calculate Deltas│
                    │  dV, dH, dθ     │
                    └────────┬────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ Offset Compen-  │
                    │ sation          │
                    │  dVc, dHc       │
                    └────────┬────────┘
                              │
                              ▼
                  ┌───────────┴───────────┐
                  │    |dθ| < 1e-6?       │
                  └───────────┬───────────┘
                    Yes │           │ No
                        ▼           ▼
              ┌──────────────┐  ┌──────────────┐
              │ Linear Approx│  │ Arc Integral │
              │ lX = dHc     │  │ chord = 2sin │
              │ lY = dVc     │  │ lX,lY scaled │
              └──────┬───────┘  └──────┬───────┘
                     │                 │
                     └────────┬────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ Rotate to Global│
                    │ x += transform  │
                    │ y += transform  │
                    └────────┬────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ Update State    │
                    │ θ += dθ         │
                    │ Store previous  │
                    └─────────────────┘
```

#### Resetting Odometry

```cpp
void resetOdometry() {
    robotPose = {0.0, 0.0, 0.0};

    // Reset IMU heading and rotation
    inertial_sensor.setHeading(0, deg);
    inertial_sensor.setRotation(0, deg);

    // Reset tracking wheels
    verticalRot.resetPosition();
    horizontalRot.resetPosition();

    // Reinitialize state
    s.prevV_m = degToMeters(verticalDeg());
    s.prevH_m = degToMeters(horizontalDeg());
    s.prevRotDeg = rotationDeg();
    s.theta = degToRad(s.prevRotDeg);
    robotPose.theta = s.theta;
}
```

#### Configuration Parameters

```cpp
namespace config {
    // Tracking wheel circumference (meters)
    // π × diameter = π × 0.049m ≈ 0.154m
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.049 * M_PI;

    // Tracking wheel offsets from robot center (meters)
    // Positive SIDE_OFFSET = wheel is to the right of center
    // Positive VERT_OFFSET = wheel is forward of center
    const double SIDE_OFFSET_M = 0.0;
    const double VERT_OFFSET_M = 0.0;
}
```

#### Error Sources and Mitigation

| Error Source | Impact | Mitigation |
|--------------|--------|------------|
| Wheel slip | Position drift | Use low-slip wheels, proper weight distribution |
| Encoder resolution | Quantization error | High-resolution encoders (V5 rotation sensors) |
| IMU drift | Heading error | V5 IMU has built-in drift correction |
| Wheel diameter error | Systematic position error | Calibrate wheel circumference |
| Offset measurement error | Rotation artifacts | Careful measurement, spin calibration |
| Update rate too slow | Integration error | 100Hz update rate |

#### Odometry Accuracy Expectations

Under good conditions (clean wheels, flat surface):
- **Position accuracy**: ±2-5cm over 2 meters
- **Heading accuracy**: ±1-2° over multiple rotations
- **Repeatability**: ±1cm for repeated movements

Factors that degrade accuracy:
- Field tiles with seams
- Debris on wheels
- Battery voltage variations
- Wheel wear

---

### Motion Controller

The `MotionController` class (`motion.h`, `motion.cpp`) provides high-level motion primitives for autonomous navigation, built on top of the PID controllers and odometry system.

#### Design Philosophy

The motion controller provides multiple "layers" of control sophistication:

1. **Basic**: Simple drive/turn with encoder feedback
2. **Heading Hold**: Maintains heading during straight movement
3. **Closed-Loop (CC)**: Uses odometry for position feedback
4. **Auto-Correct (AC)**: Adds post-movement position correction

#### Method Reference

##### Basic Driving

```cpp
void drive(double distM, int timeoutMs = 4000, double maxSpeedPct = 80.0);
```
- Drives straight using encoder feedback
- Automatically holds current heading
- Parameters:
  - `distM`: Distance in meters (positive = forward)
  - `timeoutMs`: Maximum time for movement
  - `maxSpeedPct`: Speed limit (0-100)

```cpp
void driveHeading(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg);
```
- Drives while maintaining a specific heading
- Use when you need to drive at an angle different from current heading

##### Point Turns

```cpp
void turnTo(double targetDeg, int timeoutMs = 4000);
```
- Turns to an absolute heading (0-360°)
- Uses IMU for feedback

```cpp
void turnBy(double deltaDeg, int timeoutMs = 4000);
```
- Turns by a relative amount
- Uses IMU rotation (unbounded) to avoid wrap-around issues

##### Closed-Loop Driving (CC variants)

```cpp
void driveCC(double distM, int timeoutMs = 4000, double maxSpeedPct = 80.0);
void driveHeadingCC(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg);
```
- Uses odometry for position feedback instead of just encoder distance
- Corrects for lateral drift during movement
- Better accuracy for longer distances

##### Auto-Correct Variants (AC)

```cpp
void driveAC(double distM, int timeoutMs = 4000, double maxSpeedPct = 80.0,
             int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

void driveHeadingAC(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg,
                    int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

void turnToAC(double targetDeg, int timeoutMs = 4000,
              int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

void turnByAC(double deltaDeg, int timeoutMs = 4000,
              int correctTimeoutMs = 900, double correctSpeedPct = 30.0);
```
- Performs movement, then runs `autoCorrect()` to fine-tune position
- Best accuracy for critical positioning
- Must enable with `setAutoCorrectEnabled(true)`

##### Auto-Correction Algorithm

```cpp
void autoCorrect(double targetX, double targetY, double targetHeadingDeg,
                 int timeoutMs = 900, double maxSpeedPct = 30.0);
```

The auto-correct algorithm:

1. **Check if correction needed**: Compare current pose to target
2. **Fix heading first**: Turn to target heading if off by more than threshold
3. **Backup if necessary**: If target is behind, back up first
4. **Lateral correction**: Strafe or turn-drive-turn to fix X error
5. **Forward correction**: Drive forward/backward to fix Y error
6. **Final heading check**: Ensure heading is correct

```
┌─────────────────────────────────────────────────────────────────┐
│                   AUTO-CORRECT ALGORITHM                        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │ Calculate Error │
                    │ dx, dy, dθ      │
                    └────────┬────────┘
                              │
                              ▼
              ┌───────────────────────────────┐
              │ Within tolerance?             │
              │ dist < 7cm AND |dθ| < 3°     │
              └───────────────┬───────────────┘
                    Yes │           │ No
                        ▼           ▼
                    [DONE]    ┌─────────────────┐
                              │ Fix Heading     │
                              │ if |dθ| > 6°    │
                              └────────┬────────┘
                                       │
                                       ▼
                              ┌─────────────────┐
                              │ Target behind?  │
                              │ Back up 5cm     │
                              └────────┬────────┘
                                       │
                                       ▼
                              ┌─────────────────┐
                              │ Fix Lateral     │
                              │ Turn-Drive-Turn │
                              └────────┬────────┘
                                       │
                                       ▼
                              ┌─────────────────┐
                              │ Fix Forward     │
                              │ Drive to Y      │
                              └────────┬────────┘
                                       │
                                       ▼
                              ┌─────────────────┐
                              │ Repeat if needed│
                              │ (max 2 iters)   │
                              └─────────────────┘
```

#### AutoCorrect Threshold Constants

```cpp
// Entry thresholds - when to begin correction
const double ENTER_DIST = 0.12;  // 12cm from target position
const double ENTER_ANG  = 6.0;   // 6° from target heading
const double ENTER_POS  = 0.06;  // 6cm error in single axis

// Exit thresholds - when correction is complete
const double EXIT_DIST  = 0.07;  // 7cm acceptable final error
const double EXIT_ANG   = 3.0;   // 3° acceptable heading error
const double EXIT_POS   = 0.03;  // 3cm acceptable axis error

// Movement limits
const double MAX_STEP_M = 0.20;  // Maximum single correction distance
const double BACKUP_M   = 0.05;  // Backup distance when target is behind
```

#### Settling Detection

Movements use sophisticated settling detection combining multiple criteria:

**Drive Settling**:
```cpp
// Stop-and-hold latch with hysteresis
const double stopEnter = 0.030;  // Enter hold at 3cm
const double stopExit  = 0.055;  // Exit hold at 5.5cm
const int stopSettleMs = 120;    // Hold for 120ms

// Position tolerance + velocity check
if (|distErr| < 0.02m && |vCmd| < 6%) {
    settledMs += dt;
    if (settledMs >= 40ms) break;
}
```

**Turn Settling**:
```cpp
// Heading tolerance + angular rate check
const double rate = dHead / dt;
rateFilt += alpha * (rate - rateFilt);  // Low-pass filter on rate

if (|err| < 1.0° && |rateFilt| < 8°/s) {
    settledMs += dt;
    if (settledMs >= 150ms) break;
}
```

**Zero-Crossing Detection**:
```cpp
// Detect when robot crosses target (prevents hunting)
if ((distErrPrev > 0.0 && distErr < 0.0) || 
    (distErrPrev < 0.0 && distErr > 0.0)) {
    crossed = true;
}

// Use tighter threshold after crossing
if (crossed && |distErr| < stopExit) {
    stopLatch = true;  // Enter hold mode
}
```

#### Slew Rate Limiting

All outputs are slew-rate limited for smooth acceleration:

```cpp
const double dvPerSec = 160.0;  // Linear velocity slew (%/s)
const double dwPerSec = 260.0;  // Angular velocity slew (%/s)

// Calculate maximum change per update
const double dvMax = dvPerSec * dt;  // e.g., 1.6% per 10ms
const double dwMax = dwPerSec * dt;  // e.g., 2.6% per 10ms

// Apply slew limiting
vCmd += clamp(v - vCmd, -dvMax, dvMax);
wCmd += clamp(w - wCmd, -dwMax, dwMax);
```

This prevents:
- Wheel slip from sudden acceleration
- Mechanical stress on drivetrain
- Jerky motion that disrupts odometry

#### Dynamic Speed Capping

Speed is reduced near the target to prevent overshoot:

```cpp
// Minimum cap ensures robot can always move
const double minCap = 10.0;

// Speed cap proportional to remaining distance
double cap = minCap + 200.0 * |distErr|;
cap = min(cap, maxSpeedPct);
distPID.setOutputLimits(-cap, cap);
```

Example progression:
| Distance to Target | Speed Cap |
|--------------------|-----------|
| 0.50m | min(80, 10+100) = 80% |
| 0.20m | min(80, 10+40) = 50% |
| 0.10m | min(80, 10+20) = 30% |
| 0.05m | min(80, 10+10) = 20% |
| 0.02m | min(80, 10+4) = 14% |

#### Heading Correction Scaling

During driving, heading corrections are scaled based on forward speed:

```cpp
// Reduce heading correction at low speeds
const double vAbs = |v|;
double wScale = min(1.0, vAbs / 18.0);
wScale = max(wScale, 0.25);  // Minimum 25% correction

// Allow more correction if heading error is large at low speed
if (vAbs < 12.0 && |headErr| > 2.0) {
    wScale = max(wScale, 0.22);
}

w *= wScale;
```

This prevents:
- Aggressive turning when nearly stopped
- Heading oscillation at low speeds
- But still allows correction when significantly off-heading

#### Minimum Speed Floor

Ensures robot continues moving despite friction:

```cpp
// If close but not at target, ensure minimum power
if (|distErr| > 0.12) {
    const double floor = 8.0;
    if (|v| < floor) {
        v = (distErr > 0.0) ? floor : -floor;
    }
}
```

#### Anti-Reverse Logic

Prevents oscillation around target:

```cpp
// If very close and moving wrong direction, stop
if (|distErr| < 0.03 && |v| < 12.0 && (v * distErr) < 0.0) {
    v = 0.0;  // Stop instead of reversing
}
```

#### Stop-and-Hold Behavior

When reaching target, the controller enters a hold phase:

```cpp
if (stopLatch) {
    // Only correct heading during hold (no forward motion)
    const double currHead = inertial_sensor.heading(deg);
    const double headErr = angleDiffDeg(holdHead, currHead);

    double w = headPID_.update(-headErr, dt);

    // If heading is very close, stop turning too
    const double headAbs = |headErr|;
    if (headAbs < 1.0) {
        w = 0.0;
        wCmd = 0.0;
    } else {
        wCmd += clamp(w - wCmd, -dwMax, dwMax);
        w = wCmd;
    }

    tankDrive(w, -w);  // Pure rotation

    stopHoldMs += dtMs;
    if (stopHoldMs >= stopSettleMs) break;
}
```

---

### Drive System

The drive system (`drive.h`, `drive.cpp`) provides low-level motor control functions.

#### Functions

```cpp
void tankDrive(double leftPct, double rightPct);
```
- Direct control of left and right motor groups
- Values clamped to [-100, 100]

```cpp
void arcadeDrive(double fwdPct, double turnPct);
```
- Arcade-style control (forward + turn mixing)
- Converted to tank internally: `left = fwd + turn`, `right = fwd - turn`

```cpp
void stopDrive(brakeType mode = brake);
```
- Stops both motor groups
- Modes: `coast`, `brake`, `hold`

#### Implementation

```cpp
void tankDrive(double leftPct, double rightPct) {
    LeftMotorGroup.spin(fwd, clampPct(leftPct), percent);
    RightMotorGroup.spin(fwd, clampPct(rightPct), percent);
}

void arcadeDrive(double fwdPct, double turnPct) {
    tankDrive(fwdPct + turnPct, fwdPct - turnPct);
}

void stopDrive(brakeType mode) {
    LeftMotorGroup.stop(mode);
    RightMotorGroup.stop(mode);
}
```

---

### Subsystems

The subsystems module (`subsystems.h`, `subsystems.cpp`) manages all non-drivetrain mechanisms.

#### Wings (Pneumatics)

Single-acting pneumatic pistons for wing deployment:

```cpp
class Wings {
public:
    Wings() : state(false) {}
    
    void toggle() {
        state = !state;
        wingsPiston.set(state);
    }
    
    void set(bool s) {
        state = s;
        wingsPiston.set(state);
    }
    
    bool isExtended() const { return state; }
    
private:
    bool state;
};

extern Wings wings;
```

#### Intake System

Three-motor intake for game piece collection:

```cpp
void runIntake(double speedPct = 100.0) {
    IntakeLeft.spin(fwd, speedPct, pct);
    IntakeRight.spin(fwd, speedPct, pct);
    Midtake.spin(fwd, speedPct, pct);
}

void reverseIntake(double speedPct = 100.0) {
    IntakeLeft.spin(reverse, speedPct, pct);
    IntakeRight.spin(reverse, speedPct, pct);
    Midtake.spin(reverse, speedPct, pct);
}

void stopIntake() {
    IntakeLeft.stop(coast);
    IntakeRight.stop(coast);
    Midtake.stop(coast);
}
```

#### Outtake System

Three-motor outtake for scoring:

```cpp
void runOutake(double speedPct = 100.0);
void reverseOutake(double speedPct = 100.0);
void stopOutake();
```

- Uses coast brake mode to allow balls to roll freely when stopped

#### Descore Arm

Single motor with high gear ratio for descoring:

```cpp
void moveArmRight(double speedPct = 100.0) {
    DescoreMotor.spin(fwd, speedPct, pct);
}

void moveArmLeft(double speedPct = 100.0) {
    DescoreMotor.spin(reverse, speedPct, pct);
}

void stopArm() {
    DescoreMotor.stop(hold);  // Hold position when stopped
}
```

#### Color Sorting

Automatic ball sorting based on alliance color:

```cpp
Alliance myAlliance = BLUE;  // Set before match

// Median filter for stable hue readings
static double filteredHue() {
    static std::vector<double> buf;
    buf.push_back(ballSensor.hue());
    if (buf.size() > 5) buf.erase(buf.begin());

    auto sorted = buf;
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
}

int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    while (true) {
        if (ballSensor.isNearObject()) {
            double hue = filteredHue();

            // Red: hue < 20 or hue > 340
            // Blue: hue 200-250
            bool isRed  = (hue < 20 || hue > 340);
            bool isBlue = (hue > 200 && hue < 250);

            if ((myAlliance == RED  && isBlue) ||
                (myAlliance == BLUE && isRed)) {
                // Eject opponent's ball
                reverseIntake(100);
                reverseOutake(100);
                wait(300, msec);
                runIntake(100);
                runOutake(100);
            }
        }
        wait(20, msec);
    }
    return 0;
}
```

---

## Robot Configuration

### Motor Ports

| Motor | Port | Ratio | Reversed | Purpose |
|-------|------|-------|----------|---------|
| LeftA | 3 | 6:1 | No | Left drive front |
| LeftB | 19 | 6:1 | No | Left drive middle |
| LeftC | 10 | 6:1 | No | Left drive rear |
| RightA | 11 | 6:1 | Yes | Right drive front |
| RightB | 13 | 6:1 | Yes | Right drive middle |
| RightC | 15 | 6:1 | Yes | Right drive rear |
| IntakeLeft | 16 | 6:1 | No | Left intake roller |
| IntakeRight | 5 | 6:1 | Yes | Right intake roller |
| Midtake | 8 | 18:1 | No | Middle intake stage |
| RightOutake | 7 | 6:1 | Yes | Right scoring roller |
| LeftOutake | 2 | 6:1 | No | Left scoring roller |
| MidOutake | 20 | 6:1 | Yes | Middle scoring roller |
| DescoreMotor | 18 | 36:1 | Yes | Descoring arm |

### Sensor Ports

| Sensor | Port | Purpose |
|--------|------|---------|
| Inertial | 1 | Heading/rotation |
| Vertical Rotation | 9 | Forward tracking |
| Horizontal Rotation | 6 | Lateral tracking |
| Optical | 21 | Ball color detection |
| Wings Piston | 3-Wire B | Pneumatic control |

### Physical Constants

```cpp
namespace config {
    const double TRACK_WIDTH_M = 0.320;  // 320mm between wheels
    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.049 * M_PI;  // ~154mm
    const double SIDE_OFFSET_M = 0.0;    // Horizontal wheel offset
    const double VERT_OFFSET_M = 0.0;    // Vertical wheel offset
}
```

---

## Autonomous Routines

### Available Routines

| Routine | Description |
|---------|-------------|
| `NONE` | No autonomous |
| `TEST` | Testing routine |
| `RED_LEFT` | Red alliance, left starting position |
| `RED_RIGHT` | Red alliance, right starting position |
| `BLUE_LEFT` | Blue alliance, left starting position |
| `BLUE_RIGHT` | Blue alliance, right starting position |
| `SKILLS` | Skills challenge routine |
| `AUTO_CORRECT_*` | Routines with auto-correction enabled |

### Selecting a Routine

```cpp
// In autons.cpp
AutonRoutine selectedAuton = AutonRoutine::AUTO_CORRECT_BLUE_RIGHT;
```

### Example Routine

```cpp
static void autoCorrectBlueRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    // Drive forward 80cm at full speed
    m.drive(0.80, 7200, 100);
    wait(10, vex::msec);

    // Turn to face 90 degrees
    m.turnTo(90, 7000);
    wait(10, vex::msec);

    // Run intake and approach goal
    runIntake(100);
    m.drive(0.380, 5000, 90);
    wait(1.0, sec);

    // Back up and turn around
    m.drive(-0.3, 4500, 90);
    m.turnBy(175, 5500);
    wait(10, msec);

    // Deploy wings and score
    stopIntake();
    wings.toggle();
    m.drive(0.590, 5000, 90);

    runIntake(100);
    runOutake(100);
    wait(4, sec);

    stopIntake();
    stopOutake();
}
```

---

## Manual Control

### Control Scheme

| Control | Action |
|---------|--------|
| **Left Stick Y (Axis 3)** | Forward/Backward |
| **Right Stick X (Axis 1)** | Turn Left/Right |
| **R1** | Toggle speed mode (Fast/Slow) |
| **Up** | Toggle wings |
| **L1 / L2** | Run intake |
| **R2** | Reverse intake + Outtake |
| **Left** | Move arm left |
| **Right** | Move arm right |
| **X** | Reset odometry |
| **Y** | Toggle odometry display |

### Drive Curves

Exponential response curves for fine control:

```cpp
static double computeCurve(double inputPct, double curve) {
    double v = inputPct / 100.0;
    return ((curve * pow(v, 3)) + ((1.0 - curve) * v)) * 100.0;
}

// Forward: 0.2 curve (mostly linear, slight curve)
// Turn: 0.285 curve (more exponential for fine turning)
```

**Curve behavior**:
- `curve = 0.0`: Fully linear
- `curve = 1.0`: Fully cubic
- `curve = 0.2`: 20% cubic, 80% linear
- `curve = 0.285`: 28.5% cubic, 71.5% linear

### Speed Modes

| Mode | Forward Scale | Turn Scale |
|------|---------------|------------|
| Fast | 80% | 80% |
| Slow | 40% | 40% |

### Left/Right Bias

Compensates for mechanical asymmetry:

```cpp
const double LEFT_BIAS  = 0.945;  // Left side slightly weaker
const double RIGHT_BIAS = 1.0;    // Right side reference
```

---

## Tuning Guide

This section provides comprehensive guidance for tuning all aspects of the robot's control systems.

### Philosophy of Tuning

Good tuning is iterative and systematic:

1. **Start simple**: Begin with P-only control
2. **One thing at a time**: Change one parameter, observe, repeat
3. **Understand the physics**: Know what each parameter does before changing it
4. **Test consistently**: Same conditions each time
5. **Document everything**: Record what works and what doesn't
6. **Expect iteration**: First guess is rarely optimal

### Understanding PID Behavior

Before tuning, understand how each term affects the system:

```
┌──────────────────────────────────────────────────────────────────────┐
│                    EFFECT OF PID GAINS                               │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  PROPORTIONAL (kP):                                                  │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ Low kP:          Target ────────────────────                    │ │
│  │                  Response ─────────___________......            │ │
│  │                  (slow, steady-state error)                     │ │
│  ├─────────────────────────────────────────────────────────────────┤ │
│  │ Good kP:         Target ────────────────────                    │ │
│  │                  Response ────/‾‾‾‾‾‾‾‾‾‾‾‾‾                    │ │
│  │                  (fast, slight overshoot)                       │ │
│  ├─────────────────────────────────────────────────────────────────┤ │
│  │ High kP:         Target ────────────────────                    │ │
│  │                  Response ───/\  /\  /\ ────                    │ │
│  │                  (oscillating)              \/  \/  \/          │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  DERIVATIVE (kD):                                                    │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ Low kD:          Oscillation continues                          │ │
│  │                  ───/\  /\  /\  /\ ────                         │ │
│  ├─────────────────────────────────────────────────────────────────┤ │
│  │ Good kD:         Oscillation damped                             │ │
│  │                  ───/\_/‾‾‾‾‾‾‾‾‾‾‾‾                            │ │
│  ├─────────────────────────────────────────────────────────────────┤ │
│  │ High kD:         Sluggish, may amplify noise                    │ │
│  │                  ───___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾                          │ │
│  └─────────────────────────────────────────────────────────────────┘ │
│                                                                      │
│  INTEGRAL (kI):                                                      │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ No kI:           Steady-state error remains                     │ │
│  │                  Target  ────────────────────                   │ │
│  │                  Response────────___________                    │ │
│  ├─────────────────────────────────────────────────────────────────┤ │
│  │ Good kI:         Eliminates steady-state error                  │ │
│  │                  Target  ────────────────────                   │ │
│  │                  Response────────/‾‾‾‾‾‾‾‾‾‾                    │ │
│  ├─────────────────────────────────────────────────────────────────┤ │
│  │ High kI:         Overshoot and slow oscillation                 │ │
│  │                  Response────/‾‾\__/‾‾‾‾‾‾‾‾                    │ │
│  └─────────────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────────────┘
```

### PID Tuning Procedures

#### Distance PID Tuning (`distPID_`)

**Purpose**: Controls forward/backward movement based on encoder feedback.

**Test Setup**:
- Clear, flat surface (at least 3 meters)
- Fully charged battery
- Robot pointed at distant target for visual reference

**Step 1: Establish Baseline**

```cpp
// Start with these values
distPID_(0.1, 0.0, 0.0)
.setOutputLimits(-100, 100)
.setDerivativeMode(PID::DerivativeMode::OnMeasurement)
```

**Step 2: Tune Proportional Gain (kP)**

| Observation | kP Adjustment | Reasoning |
|-------------|---------------|-----------|
| Robot barely moves | Increase by 100% | Not enough power |
| Stops well short (>10cm) | Increase by 50% | Insufficient gain |
| Stops slightly short (2-10cm) | Increase by 20% | Getting close |
| Overshoots and returns | Good starting point | Found critical gain |
| Oscillates continuously | Decrease by 30% | Too much gain |
| Oscillates and grows | Decrease by 50% | Unstable |

**Procedure**:
1. Command 1.0m drive
2. Observe final position
3. Adjust kP according to table
4. Repeat until slight overshoot on first approach
5. Record this kP as "critical gain"
6. Final kP = critical gain × 0.7

**Step 3: Tune Derivative Gain (kD)**

Starting point: `kD = kP × 0.01` (very conservative for distance)

| Observation | kD Adjustment | Reasoning |
|-------------|---------------|-----------|
| Still oscillates | Increase by 50% | Need more damping |
| Overshoot reduced but present | Increase by 20% | Getting better |
| No overshoot, smooth approach | Good value | Optimal damping |
| Sluggish approach | Decrease by 30% | Over-damped |
| Jittery motor output | Decrease kD, increase filter | Noise amplification |

**Procedure**:
1. Start with kD = kP × 0.01
2. Command same 1.0m drive
3. Observe overshoot and settling
4. Adjust according to table
5. If jittery, add derivative filter

**Step 4: Configure Derivative Filter**

```cpp
.setDerivativeFilterTf(0.10)  // Start with 100ms
```

| Observation | Filter Adjustment |
|-------------|-------------------|
| Motor sounds rough | Increase Tf |
| Slow to respond near target | Decrease Tf |
| Smooth but effective | Keep current value |

Typical range: 0.06 - 0.15 seconds

**Step 5: Tune Integral Gain (kI) - Often Unnecessary**

For distance control, P+D is usually sufficient. Only add integral if:
- Robot consistently stops 1-3cm short
- Friction or slope causes persistent error

If needed:
```cpp
distPID_.setIntegralZone(0.10);  // Only integrate within 10cm
distPID_.setIntegralLimits(-20, 20);  // Limit contribution
distPID_.setAntiWindupTau(0.15);

// Start very small
kI = 0.01
```

**Step 6: Final Verification**

Test at multiple distances:
- 0.25m (short)
- 0.50m (medium)
- 1.00m (long)
- 2.00m (very long)

Robot should:
- Stop within ±2cm at all distances
- Not oscillate
- Complete in reasonable time

**Current Optimized Values**:
```cpp
distPID_(0.38, 0.0, 0.0041)
.setDerivativeMode(PID::DerivativeMode::OnMeasurement)
.setDerivativeFilterTf(0.10)
.setAntiWindupTau(0.15)
.setErrorDeadband(0.006)  // 6mm
.setOutputLimits(-100, 100)
```

---

#### Turn PID Tuning (`turnPID_`)

**Purpose**: Controls point turns to absolute or relative headings.

**Challenges specific to turning**:
- Rotational inertia continues after motors stop
- IMU has processing latency
- Static friction creates stick-slip behavior
- Asymmetric friction (CW vs CCW)

**Test Setup**:
- Mark cardinal directions on floor
- Robot in center of clear area
- Verify IMU calibration complete

**Step 1: Establish Baseline**

```cpp
turnPID_(0.1, 0.0, 0.0)
.setOutputLimits(-60, 60)  // Lower than drive!
.setDerivativeMode(PID::DerivativeMode::OnMeasurement)
```

**Step 2: Tune Proportional Gain (kP)**

Test with 90° turns (both directions):

| Observation | kP Adjustment |
|-------------|---------------|
| Turn is very slow | Increase by 100% |
| Undershoots by >15° | Increase by 50% |
| Undershoots by 5-15° | Increase by 25% |
| Overshoots once, settles | Good starting point |
| Oscillates 2-3 times | Decrease by 20% |
| Oscillates continuously | Decrease by 40% |

**Step 3: Tune Derivative Gain (kD)**

Turning needs more derivative than driving (higher ratio):

Starting point: `kD = kP × 0.05`

| Observation | kD Adjustment |
|-------------|---------------|
| Still overshoots significantly | Increase by 30% |
| Slight overshoot | Increase by 15% |
| Clean stop, no overshoot | Good value |
| Stops early, creeps to target | Decrease by 20% |

**Step 4: Configure Derivative Filter**

```cpp
.setDerivativeFilterTf(0.06)  // Start with 60ms
```

IMU can be noisy during acceleration. Watch for:
- Jittery turn motion → increase filter
- Slow reaction at end → decrease filter

**Step 5: Tune Integral Gain (kI)**

Integral is MORE important for turns than drives because:
- Static friction prevents small corrections
- Mechanical asymmetry causes consistent errors

```cpp
turnPID_.setIntegralZone(15.0);  // Only integrate within 15°
turnPID_.setIntegralLimits(-50.0, 50.0);  // Allow significant contribution
turnPID_.setAntiWindupTau(0.18);
```

Start with `kI = 0.001` and increase until:
- Both CW and CCW turns end within 1°
- No oscillation during final approach

**Step 6: Test Comprehensive Turn Cases**

| Test | Expected Result |
|------|-----------------|
| 90° CW | Within 1° |
| 90° CCW | Within 1° |
| 180° | Within 1.5° |
| 45° | Within 1° |
| 15° (small) | Within 0.5° |
| 360° (full rotation) | Within 2° |

**Step 7: Verify Rate Limiting**

Ensure the robot doesn't spin too fast:
```cpp
.setOutputLimits(-60, 60)  // Adjust if too slow or too fast
```

**Current Optimized Values**:
```cpp
turnPID_(0.25, 0.001, 0.001)
.setDerivativeMode(PID::DerivativeMode::OnMeasurement)
.setDerivativeFilterTf(0.06)
.setAntiWindupTau(0.18)
.setIntegralZone(15.0)
.setIntegralLimits(-50.0, 50.0)
.setErrorDeadband(0.2)
.setOutputLimits(-60, 60)
```

---

#### Heading Hold PID Tuning (`headPID_`)

**Purpose**: Maintains heading during straight-line driving. This is a "correction" PID, not a "position" PID.

**Key Insight**: This PID needs to be GENTLE. Aggressive heading corrections cause:
- Weaving during straight drives
- Reduced effective forward speed
- Wasted energy

**Test Setup**:
- Long clear path (3+ meters)
- Mark starting heading
- Robot should drive perfectly straight

**Step 1: Establish Baseline**

```cpp
headPID_(0.1, 0.0, 0.0)
.setOutputLimits(-16, 16)  // Very low limits!
.setDerivativeMode(PID::DerivativeMode::OnMeasurement)
```

**Step 2: Tune for Straight Driving**

Command a 2m drive and observe:

| Observation | Adjustment |
|-------------|------------|
| Robot drifts significantly (>10cm lateral) | Increase kP by 50% |
| Robot drifts slightly (5-10cm lateral) | Increase kP by 25% |
| Robot drives straight | Good kP |
| Robot weaves back and forth | Decrease kP by 30% |
| Robot oscillates in heading | Decrease kP, add kD |

**Step 3: Add Derivative for Stability**

Starting point: `kD = kP × 0.01`

The derivative should:
- Prevent heading oscillation
- Not cause sluggish correction

**Step 4: Add Small Integral (Optional)**

Only if robot consistently drifts one direction:

```cpp
headPID_.setIntegralLimits(-2.0, 2.0);  // Very tight limits
kI = 0.001  // Very small
```

**Step 5: Verify Output Limits**

Critical for heading hold:
```cpp
.setOutputLimits(-16, 16)  // Max 16% correction
```

This ensures heading corrections are subtle and don't dominate forward motion.

**Step 6: Test at Various Speeds**

Heading hold should work at:
- Low speed (30%)
- Medium speed (60%)
- High speed (100%)

**Current Optimized Values**:
```cpp
headPID_(0.25, 0.001, 0.001)
.setDerivativeMode(PID::DerivativeMode::OnMeasurement)
.setDerivativeFilterTf(0.10)  // More filtering than turn
.setAntiWindupTau(0.20)
.setIntegralLimits(-2.0, 2.0)  // Very tight
.setErrorDeadband(0.3)  // Ignore <0.3° error
.setOutputLimits(-16, 16)  // Subtle corrections only
```

---

### Odometry Calibration

#### Tracking Wheel Circumference Calibration

**Why It Matters**: 1% error in circumference = 1% error in ALL distance measurements.

**Equipment Needed**:
- Measuring tape (at least 2m)
- Masking tape for floor marking
- Flat, smooth surface

**Procedure**:

1. **Prepare the Test Area**
   - Mark start line with tape
   - Measure exactly 2.000m from start
   - Mark end line with tape

2. **Position Robot**
   - Align tracking wheel with start line
   - Reset encoder: `verticalRot.resetPosition();`
   - Record initial reading (should be 0)

3. **Collect Data**
   - Push robot SLOWLY and STRAIGHT to end line
   - Ensure wheel stays in contact throughout
   - Record final encoder value in degrees

4. **Calculate Circumference**
   ```
   circumference = (360 / encoder_degrees) × 2.0 meters
   ```

5. **Repeat 3 Times**
   - Take average of measurements
   - Variation should be <1%

**Example Calculation**:
```
Trial 1: 4165.2° → C = (360/4165.2) × 2.0 = 0.17290 m
Trial 2: 4162.8° → C = (360/4162.8) × 2.0 = 0.17300 m
Trial 3: 4168.1° → C = (360/4168.1) × 2.0 = 0.17278 m

Average: 0.17289 m

Compare to theoretical:
Diameter = 49mm → C = π × 0.049 = 0.15394 m
Measured is larger → possible causes:
- Wheel compression under weight
- Encoder gear ratio error
- Measurement error
```

6. **Update Configuration**
   ```cpp
   const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.17289;
   ```

7. **Verify**
   - Drive robot exactly 1.00m
   - Odometry should read 1.00m ± 0.01m

**Repeat for Horizontal Wheel** - may have different circumference due to:
- Different wheel material
- Different loading

---

#### Tracking Wheel Offset Calibration

**Why It Matters**: Incorrect offsets cause position drift during rotation.

**Theory**:
When the robot rotates, a tracking wheel not at the rotation center traces an arc:
```
Arc length = offset × rotation_angle (radians)
```

This arc appears as translation if not compensated.

**Symptoms of Wrong Offsets**:
- X drifts during pure rotation → `SIDE_OFFSET_M` is wrong
- Y drifts during pure rotation → `VERT_OFFSET_M` is wrong

**Procedure**:

1. **Prepare**
   - Clear area for spinning
   - Reset odometry: `resetOdometry();`
   - Record initial X, Y (should be 0, 0)

2. **Spin Test**
   - Command 10 complete rotations (3600°)
   - Use `turnBy(3600, 60000)` or spin manually
   - Verify rotation with IMU

3. **Record Results**
   ```
   After 10 rotations:
   X = 0.15m (positive = drifted right)
   Y = -0.03m (negative = drifted backward)
   ```

4. **Calculate Offset Corrections**
   ```
   Total rotation = 10 × 2π = 62.83 radians
   
   X drift = SIDE_OFFSET_M × total_rotation
   SIDE_OFFSET_correction = X_drift / 62.83
   
   Similarly for Y drift and VERT_OFFSET_M
   ```

   **Example**:
   ```
   SIDE_OFFSET_correction = 0.15 / 62.83 = 0.00239 m
   VERT_OFFSET_correction = -0.03 / 62.83 = -0.00048 m
   ```

5. **Apply Corrections**
   ```cpp
   // If previously was 0.0:
   const double SIDE_OFFSET_M = 0.00239;
   const double VERT_OFFSET_M = -0.00048;
   
   // If previously had values, add the corrections
   ```

6. **Iterate**
   - Repeat spin test
   - X and Y should now be <1cm after 10 rotations
   - May need 2-3 iterations

**Sign Convention**:
```
SIDE_OFFSET_M:
  Positive = wheel is RIGHT of center
  If X drifts positive (right), offset needs to INCREASE

VERT_OFFSET_M:
  Positive = wheel is FORWARD of center
  If Y drifts positive (forward), offset needs to INCREASE
```

---

### Slew Rate Tuning

**Purpose**: Limits acceleration to prevent wheel slip and smooth motion.

#### Finding Maximum Safe Acceleration

**Symptoms of Too-High Slew Rate**:
- Wheels chirp on acceleration
- Odometry becomes inaccurate (wheel slip)
- Robot lurches at start
- Inconsistent autonomous

**Symptoms of Too-Low Slew Rate**:
- Sluggish response
- Can't complete movements in time
- Driver feels disconnect from controls

**Procedure**:

1. **Find Slip Threshold**
   - On actual field surface
   - From standstill, command 100% power
   - Note at what %/s wheels slip

2. **Calculate Safe Rate**
   ```
   If slip occurs transitioning 0% → 50% in 0.1s:
   slip_rate = 50 / 0.1 = 500 %/s
   
   Safe rate = slip_rate × 0.7 = 350 %/s
   ```

3. **Test Different Surface Conditions**
   - Fresh field tiles (high grip)
   - Worn field tiles (lower grip)
   - Use lowest value as your limit

**Current Values**:
```cpp
// Motion controller (conservative for accuracy)
const double dvPerSec = 160.0;  // Linear velocity
const double dwPerSec = 260.0;  // Angular velocity

// Manual control (more aggressive for responsiveness)
const double ACCEL_PCT_PER_S = 350.0;
const double DECEL_PCT_PER_S = 290.0;
```

**Deceleration Note**:
- Can usually be higher than acceleration
- Motor braking assists deceleration
- Typical ratio: decel = 0.8 × accel

---

### Settling Parameter Tuning

**Purpose**: Determine when a movement is "done" and safe to proceed.

#### Position Tolerance

```cpp
const double stopBand = 0.015;  // 15mm
```

**Tuning Considerations**:

| Use Case | Tolerance | Reasoning |
|----------|-----------|-----------|
| Fast autonomous | 20-30mm | Speed over precision |
| Standard match | 15-20mm | Balanced |
| Skills (time available) | 10-15mm | Maximum precision |
| Scoring alignment | 5-10mm | Critical accuracy |

#### Settle Time

```cpp
const int settleTime = 40;  // 40ms for drive
const int turnSettleTime = 150;  // 150ms for turn
```

**Why Turns Need Longer**:
- Rotational inertia continues after command
- IMU processing delay
- Mechanical oscillation

**Tuning Process**:
1. Set settle time to 0
2. Observe false triggers (proceeds while still moving)
3. Increase until no false triggers
4. Add 20% safety margin

#### Velocity Threshold

```cpp
const double velocityThreshold = 6.0;  // %
const double rateTolerance = 8.0;  // °/s
```

**Setting Velocity Threshold**:
1. Stop robot and read commanded velocity
2. Noise floor is typically 1-3%
3. Set threshold 2x above noise floor

---

### Auto-Correct Threshold Tuning

```cpp
// Entry thresholds
const double ENTER_DIST = 0.12;  // 12cm
const double ENTER_ANG  = 6.0;   // 6°
const double ENTER_POS  = 0.06;  // 6cm

// Exit thresholds  
const double EXIT_DIST  = 0.07;  // 7cm
const double EXIT_ANG   = 3.0;   // 3°
const double EXIT_POS   = 0.03;  // 3cm
```

**Hysteresis (ENTER vs EXIT)**:
- EXIT should be tighter than ENTER (typical ratio: 0.5)
- Prevents oscillating between correct/not-correct states
- If EXIT = ENTER, may get stuck in correction loop

**Time Budget Considerations**:

| Scenario | ENTER | EXIT | Notes |
|----------|-------|------|-------|
| Time-limited match | 15cm | 8cm | Minimize correction time |
| Skills autonomous | 10cm | 4cm | Maximum precision |
| Scoring position | 8cm | 3cm | Critical alignment |
| General navigation | 12cm | 7cm | Standard values |

---

### Motor Bias Tuning

**Purpose**: Compensate for mechanical asymmetry in drivetrain.

**Causes of Asymmetry**:
- Motor performance variation
- Gear mesh differences
- Weight distribution
- Friction variation

**Procedure**:

1. **Test Straight Driving**
   - Command equal power (no turn input)
   - Drive 3 meters
   - Measure lateral deviation

2. **Calculate Bias**
   ```
   If robot drifts 10cm RIGHT over 3m:
   - Left side is 3.3% weaker
   - Apply: LEFT_BIAS = 1.0, RIGHT_BIAS = 0.967
   
   Or equivalently:
   - LEFT_BIAS = 1.033, RIGHT_BIAS = 1.0
   ```

3. **Apply and Verify**
   ```cpp
   const double LEFT_BIAS  = 0.945;
   const double RIGHT_BIAS = 1.0;
   ```

4. **Test at Multiple Speeds**
   - Bias may vary with speed
   - Use average value
   - Or implement speed-dependent bias

---

### Drive Curve Tuning

**Purpose**: Adjust joystick response for driver preference.

**The Curve Function**:
```cpp
output = (curve × input³) + ((1-curve) × input)
```

**Visual Response**:
```
100% ┌──────────────────────────────────────┐
     │                              ___----│  curve = 0 (linear)
     │                         __--‾       │
     │                    __--‾            │
     │               __--‾                 │
 50% │          __--‾                      │
     │     __--‾                           │
     │__--‾                                │
     │  ___-------‾‾‾‾‾‾‾‾‾‾‾‾‾            │  curve = 0.3
     │_-‾                                  │
  0% └──────────────────────────────────────┘
     0%                 50%               100%
                    INPUT
```

**Driver Preference Guide**:

| Driver Style | Forward Curve | Turn Curve | Notes |
|--------------|---------------|------------|-------|
| Precise/Methodical | 0.15-0.20 | 0.25-0.30 | More linear response |
| Aggressive | 0.30-0.40 | 0.35-0.45 | More deadzone at center |
| New driver | 0.25-0.35 | 0.40-0.50 | Forgiving at low speeds |

**Current Values**:
```cpp
const double FWD_CURVE  = 0.2;   // Mostly linear
const double TURN_CURVE = 0.285; // More curve for fine control
```

---

### Complete Tuning Checklist

#### Pre-Season Tuning (Do Once)

- [ ] Calibrate vertical wheel circumference
- [ ] Calibrate horizontal wheel circumference
- [ ] Calibrate tracking wheel offsets (spin test)
- [ ] Verify IMU calibration procedure
- [ ] Tune distance PID (P → D → filter)
- [ ] Tune turn PID (P → D → I → filter)
- [ ] Tune heading hold PID
- [ ] Set slew rates for typical battery voltage
- [ ] Configure motor biases
- [ ] Set driver curve preferences
- [ ] Test auto-correct thresholds
- [ ] Validate settling parameters

#### Weekly Maintenance Tuning

- [ ] Verify odometry accuracy (drive known distances)
- [ ] Check turn accuracy (multiple 90° turns)
- [ ] Inspect tracking wheels for wear
- [ ] Re-measure wheel circumference if worn
- [ ] Verify motor bias still correct
- [ ] Test at various battery voltages

#### Pre-Match Checklist

- [ ] All sensors responding
- [ ] Motors at safe temperature
- [ ] Pneumatic pressure adequate
- [ ] IMU calibration complete
- [ ] Odometry reset at starting position
- [ ] Correct autonomous selected
- [ ] Alliance color set correctly

#### Post-Match Analysis

- [ ] Review autonomous video
- [ ] Note any position errors
- [ ] Check for odometry drift
- [ ] Identify movements needing adjustment
- [ ] Document any mechanical issues
- [ ] Plan parameter updates

---

### Tuning Quick Reference Card

**PID Gains**:
| Parameter | Start | Increase If | Decrease If |
|-----------|-------|-------------|-------------|
| Distance kP | 0.2 | Undershoots | Oscillates |
| Distance kD | 0.02 | Oscillates | Sluggish |
| Turn kP | 0.15 | Undershoots | Oscillates |
| Turn kD | 0.01 | Overshoots | Sluggish |
| Turn kI | 0.001 | Steady error | Oscillates |
| Heading kP | 0.1 | Drifts | Weaves |

**Filters and Limits**:
| Parameter | Typical Range | Increase If | Decrease If |
|-----------|---------------|-------------|-------------|
| D Filter Tf | 0.04-0.15s | Noisy output | Slow response |
| Anti-Windup τ | 0.1-0.3s | Slow windup recovery | Overshoot |
| Integral Zone | 10-20° | Early saturation | Steady error |
| Output Limits | ±60-100% | Can't reach target | Too aggressive |

**Motion Parameters**:
| Parameter | Conservative | Standard | Aggressive |
|-----------|--------------|----------|------------|
| Slew Rate | 120 %/s | 180 %/s | 300 %/s |
| Settle Time | 80ms | 50ms | 30ms |
| Position Tolerance | 10mm | 20mm | 35mm |
| Angle Tolerance | 0.5° | 1.0° | 2.0° |

---

## Usage

### Building the Project

1. Open in VEXcode Pro V5
2. Select target robot configuration
3. Build (Ctrl+Shift+B)
4. Download to robot brain

### Pre-Match Checklist

1. Verify all motors spin correct direction
2. Confirm sensor ports match configuration
3. Set `myAlliance` to correct value
4. Select appropriate `selectedAuton`
5. Calibrate IMU (automatic on startup)
6. Reset odometry at starting position

### Testing New Routines

1. Start with `AUTO_CORRECT_*` variants for position correction
2. Use controller Y button to monitor odometry
3. Test in small increments
4. Verify each movement before adding next

---

## Troubleshooting

### Motor Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Motors not spinning | Wrong port | Check `robot_config.cpp` ports |
| Motors spin wrong way | Reversed flag wrong | Toggle reversed parameter |
| Robot pulls to one side | Bias incorrect | Adjust `LEFT_BIAS`/`RIGHT_BIAS` |
| Motors stutter | PID oscillation | Reduce gains, increase D filter |
| Motors overheat | Sustained high load | Check for mechanical binding |

### Sensor Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| IMU drift | Bad calibration | Recalibrate, don't move during cal |
| Erratic heading | Magnetic interference | Move away from motors/metal |
| Encoder skipping | Loose connection | Check cable connections |
| Odometry inaccurate | Wheel slip | Use better traction wheels |
| Position jumps | Wrong circumference | Recalibrate wheel circumference |

### Control Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Overshooting target | kP too high | Reduce kP, increase kD |
| Never reaches target | kP too low | Increase kP |
| Oscillating at target | kD too low | Increase kD, add deadband |
| Drifts during drive | Heading PID weak | Increase heading kP |
| Jerky motion | Slew rate too high | Reduce acceleration limits |
| Slow response | Slew rate too low | Increase acceleration limits |

### Autonomous Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Off course | Odometry drift | Recalibrate, use auto-correct |
| Timing out | Distances wrong | Measure actual distances |
| Inconsistent | Battery voltage | Test at competition voltage |
| Collisions | Wrong starting position | Mark exact start position |

---

## API Reference

### Utility Functions (`utils.h`)

```cpp
double clampD(double v, double lo, double hi);  // Clamp value to range
double clampPct(double v);                       // Clamp to [-100, 100]
double degToRad(double deg);                     // Degrees to radians
double radToDeg(double rad);                     // Radians to degrees
double wrap180(double deg);                      // Wrap to [-180, 180]
```

### Sensor Functions (`sensors.h`)

```cpp
void initSensors();          // Initialize and calibrate sensors
double headingDeg();         // Get IMU heading [0, 360)
double rotationDeg();        // Get IMU rotation (unbounded)
double verticalDeg();        // Get vertical encoder degrees
double horizontalDeg();      // Get horizontal encoder degrees
```

---

## License

This code is provided for educational and competition use. Modify and distribute as needed for your robotics projects.

---

## Acknowledgments

- VEX Robotics for the V5 platform
- The robotics community for PID and odometry algorithms
- Team members and mentors for testing and feedback
