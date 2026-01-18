# VEX V5 Robot - Mermaid Flowcharts

These diagrams can be rendered using any Mermaid-compatible viewer (GitHub, VS Code, etc.)

---

## 1. System Boot Sequence

```mermaid
flowchart TD
    A[Power On] --> B[VEX RTOS Init]
    B --> C[main]
    C --> D[Register autonomous callback]
    C --> E[Register drivercontrol callback]
    C --> F[pre_auton]
    
    F --> G[initSensors]
    G --> G1[IMU Calibrate<br>BLOCKING: 2-3s]
    G --> G2[Set Heading = 0]
    G --> G3[Reset Encoders]
    
    G1 --> H[resetOdometry]
    G2 --> H
    G3 --> H
    
    H --> I[Start odomTask]
    H --> J[Start sorterTask]
    
    I --> K[Display 'Sensors ready']
    J --> K
    
    K --> L[Main Loop<br>wait 100ms]
    L --> L
```

---

## 2. Competition State Machine

```mermaid
stateDiagram-v2
    [*] --> Disabled
    Disabled --> Autonomous: Competition Switch
    Disabled --> DriverControl: Competition Switch
    Autonomous --> Disabled: Period End
    DriverControl --> Disabled: Period End
    
    state Autonomous {
        [*] --> RunAutonomous
        RunAutonomous --> SelectRoutine
        SelectRoutine --> ExecuteRoutine
        ExecuteRoutine --> [*]
    }
    
    state DriverControl {
        [*] --> TeleopLoop
        TeleopLoop --> ProcessInputs
        ProcessInputs --> UpdateActuators
        UpdateActuators --> TeleopLoop
    }
```

---

## 3. Odometry Task

```mermaid
flowchart TD
    A[odomTaskFn Start] --> B[Initialize State]
    B --> C{Loop Forever}
    
    C --> D[Read Sensors]
    D --> D1[v_m = verticalDeg]
    D --> D2[h_m = horizontalDeg]
    D --> D3[rotDeg = IMU rotation]
    
    D1 --> E[Calculate Deltas]
    D2 --> E
    D3 --> E
    
    E --> E1[dV = v_m - prev]
    E --> E2[dH = h_m - prev]
    E --> E3[dT = degToRad dRotDeg]
    
    E1 --> F[Compensate Offsets]
    E2 --> F
    E3 --> F
    
    F --> F1[dVc = dV - VERT_OFF × dT]
    F --> F2[dHc = dH - SIDE_OFF × dT]
    
    F1 --> G{abs dT < 1e-6?}
    F2 --> G
    
    G -->|Yes| H[Linear Approx<br>lX = dHc<br>lY = dVc]
    G -->|No| I[Arc Geometry<br>chord = 2×sin dT/2<br>lX = chord × dHc/dT<br>lY = chord × dVc/dT]
    
    H --> J[Global Transform]
    I --> J
    
    J --> J1[avgT = theta + dT/2]
    J1 --> J2[x += lX×cos + lY×sin]
    J2 --> J3[y += lY×cos - lX×sin]
    J3 --> J4[theta += dT]
    
    J4 --> K[Update State]
    K --> L[wait 10ms]
    L --> C
```

---

## 4. PID Controller Update

```mermaid
flowchart TD
    A[update measurement, dt] --> B[dt = max dt, 1e-6]
    B --> C[error = setpoint - measurement]
    
    C --> D{abs error < deadband?}
    D -->|Yes| E[error = 0]
    D -->|No| F[Keep error]
    
    E --> G{Derivative Mode?}
    F --> G
    
    G -->|OnMeasurement| H[dSignal = -meas-prevMeas / dt]
    G -->|OnError| I[dSignal = error-prevError / dt]
    
    H --> J{first_ == true?}
    I --> J
    
    J -->|Yes| K[dFilt = 0<br>first_ = false]
    J -->|No| L{filterTf > 0?}
    
    L -->|Yes| M[alpha = dt / filterTf+dt<br>dFilt += alpha × dSignal-dFilt]
    L -->|No| N[dFilt = dSignal]
    
    K --> O{Can Integrate?}
    M --> O
    N --> O
    
    O -->|Yes| P[integral += error × dt<br>clamp integral]
    O -->|No| Q[Skip integration]
    
    P --> R[Calculate Output]
    Q --> R
    
    R --> R1[P = Kp × error]
    R --> R2[I = Ki × integral]
    R --> R3[D = Kd × dFilt]
    R1 --> R4[outUnsat = ff + P + I + D]
    R2 --> R4
    R3 --> R4
    
    R4 --> S[outSat = clamp outUnsat]
    
    S --> T{Anti-windup needed?}
    T -->|Yes| U[delta = outSat - outUnsat<br>integral += delta/Ki × dt/awTau]
    T -->|No| V[Skip anti-windup]
    
    U --> W[Store State<br>Return outSat]
    V --> W
```

---

## 5. Motion Controller - drive

```mermaid
flowchart TD
    A[drive distM, timeout, maxSpeed] --> B[Initialize]
    B --> B1[startM = encoder]
    B --> B2[holdHead = IMU heading]
    B --> B3[Reset PIDs]
    B --> B4[vCmd = 0, settledMs = 0]
    
    B1 --> C{Loop}
    B2 --> C
    B3 --> C
    B4 --> C
    
    C --> D{timer >= timeout?}
    D -->|Yes| Z[stopDrive brake<br>EXIT]
    D -->|No| E[Read Sensors]
    
    E --> E1[traveled = current - start]
    E1 --> E2[distErr = distM - traveled]
    
    E2 --> F[Dynamic Speed Cap]
    F --> F1[cap = 10 + 200 × abs distErr]
    F1 --> F2[cap = min cap, maxSpeed]
    
    F2 --> G[v = distPID.update]
    
    G --> H{abs distErr < 0.015?}
    H -->|Yes| I[v = 0, vCmd = 0]
    H -->|No| J{abs distErr > 0.05?}
    
    J -->|Yes| K{abs v < 8?}
    K -->|Yes| L[v = ±8 floor]
    K -->|No| M[Keep v]
    J -->|No| M
    
    I --> N[Anti-oscillation check]
    L --> N
    M --> N
    
    N --> O[Slew Rate Limit<br>vCmd = slewStep]
    
    O --> P[Heading Correction]
    P --> P1[headErr = angleDiff]
    P1 --> P2[w = headPID.update]
    P2 --> P3[Scale w by velocity]
    
    P3 --> Q[tankDrive v+w, v-w]
    
    Q --> R{Settling Check}
    R --> R1{abs distErr < 0.02<br>AND abs vCmd < 6?}
    
    R1 -->|Yes| S[settledMs += 10]
    R1 -->|No| T[settledMs = 0]
    
    S --> U{settledMs >= 40?}
    U -->|Yes| Z
    U -->|No| V[wait 10ms]
    T --> V
    
    V --> C
```

---

## 6. Motion Controller - turnTo

```mermaid
flowchart TD
    A[turnTo targetDeg, timeout] --> B[Initialize]
    B --> B1[turnPID.setSetpoint 0]
    B --> B2[prevHead = IMU heading]
    B --> B3[rateFilt = 0]
    B --> B4[resetBumpless -err0, 0]
    
    B1 --> C{Loop}
    B2 --> C
    B3 --> C
    B4 --> C
    
    C --> D{timer >= timeout?}
    D -->|Yes| Z[stopDrive brake<br>EXIT]
    D -->|No| E[Read IMU Heading]
    
    E --> F[err = angleDiff targetDeg, curr]
    
    F --> G[Angular Rate Filter]
    G --> G1[dHead = wrap180 curr - prevHead]
    G1 --> G2[rate = dHead / dt]
    G2 --> G3[rateFilt += alpha × rate - rateFilt]
    
    G3 --> H[turnOut = turnPID.update -err, dt]
    
    H --> I[tankDrive turnOut, -turnOut]
    
    I --> J{Settling Check}
    J --> J1{abs err < 1.0<br>AND abs rateFilt < 8.0?}
    
    J1 -->|Yes| K[settledMs += 10]
    J1 -->|No| L[settledMs = 0]
    
    K --> M{settledMs >= 150?}
    M -->|Yes| Z
    M -->|No| N[wait 10ms]
    L --> N
    
    N --> C
```

---

## 7. Auto-Correction System

```mermaid
flowchart TD
    A[autoCorrect x, y, heading] --> B[Set Thresholds]
    B --> B1[ENTER_DIST = 0.12m]
    B --> B2[EXIT_DIST = 0.07m]
    B --> B3[ENTER_ANG = 6°]
    B --> B4[EXIT_ANG = 3°]
    
    B1 --> C{Iteration Loop<br>max 2}
    B2 --> C
    B3 --> C
    B4 --> C
    
    C --> D[Calculate Errors]
    D --> D1[dx = targetX - robotPose.x]
    D --> D2[dy = targetY - robotPose.y]
    D --> D3[distNow = sqrt dx² + dy²]
    D --> D4[headErr = angleDiff]
    
    D1 --> E{Within Exit Range?}
    D2 --> E
    D3 --> E
    D4 --> E
    
    E -->|Yes| Z[DONE<br>EXIT]
    E -->|No| F{headErr > ENTER_ANG?}
    
    F -->|Yes| G[turnTo targetHeading]
    F -->|No| H[Skip heading correction]
    
    G --> I[Decompose to Robot Frame]
    H --> I
    
    I --> I1[fwd = dx×sin h + dy×cos h]
    I --> I2[lat = dx×cos h - dy×sin h]
    
    I1 --> J{abs lat > ENTER_POS?}
    I2 --> J
    
    J -->|Yes| K[Lateral Correction]
    K --> K1[Turn 90° toward lat]
    K1 --> K2[Drive abs lat]
    K2 --> K3[Turn back to target]
    J -->|No| L[Skip lateral]
    
    K3 --> M[Recalculate fwd]
    L --> M
    
    M --> N{abs fwd > ENTER_POS?}
    
    N -->|Yes| O[Forward Correction]
    O --> O1[Drive fwd]
    N -->|No| P[Skip forward]
    
    O1 --> Q[Check final position]
    P --> Q
    
    Q --> C
```

---

## 8. Teleop Main Loop

```mermaid
flowchart TD
    A[usercontrol] --> B[Initialize]
    B --> B1[isFast = true]
    B --> B2[showOdom = false]
    B --> B3[Set brake mode]
    
    B1 --> C{Loop Forever}
    B2 --> C
    B3 --> C
    
    C --> D[Drive Control]
    D --> D1[Read joysticks]
    D1 --> D2[Apply deadband]
    D2 --> D3[Apply curve]
    D3 --> D4[Apply scale]
    D4 --> D5[Slew rate limit]
    D5 --> D6[Output to motors]
    
    D6 --> E[Button Handling]
    E --> E1{R1 edge?}
    E1 -->|Yes| E2[Toggle isFast]
    E --> E3{Up edge?}
    E3 -->|Yes| E4[wings.toggle]
    E --> E5{X edge?}
    E5 -->|Yes| E6[resetOdometry]
    E --> E7{Y edge?}
    E7 -->|Yes| E8[Toggle showOdom]
    
    E2 --> F[Arm Control]
    E4 --> F
    E6 --> F
    E8 --> F
    
    F --> F1{Left && !Right?}
    F1 -->|Yes| F2[armTarget = UP<br>armActive = true]
    F1 -->|No| F3{Right && !Left?}
    F3 -->|Yes| F4[armTarget = DOWN<br>armActive = true]
    F3 -->|No| F5[armActive = false<br>stop hold]
    
    F2 --> F6{armActive?}
    F4 --> F6
    F5 --> F6
    
    F6 -->|Yes| F7[armUpdateFastSafe]
    F6 -->|No| G[Intake/Outtake Control]
    F7 --> G
    
    G --> H[Screen Update]
    H --> H1[screenTimer += 20]
    H1 --> H2{needsUpdate OR<br>timer >= period?}
    H2 -->|Yes| H3[updateControllerScreen]
    H2 -->|No| I[wait 20ms]
    H3 --> I
    
    I --> C
```

---

## 9. Arm Control Algorithm

```mermaid
flowchart TD
    A[armUpdateFastSafe] --> B[currentDeg = Descore.angle]
    B --> C[Calculate Safe Limits]
    C --> C1[safeUp = 77 + 2 = 79]
    C --> C2[safeDown = 191 - 2 = 189]
    
    C1 --> D[Clamp Target]
    C2 --> D
    D --> E[error = target - current]
    
    E --> F{At Hard Limit?}
    F --> F1{current <= safeUp<br>AND error < 0?}
    F --> F2{current >= safeDown<br>AND error > 0?}
    
    F1 -->|Yes| Z1[stop brake<br>RETURN]
    F2 -->|Yes| Z1
    
    F1 -->|No| G{abs error < DEADBAND?}
    F2 -->|No| G
    
    G -->|Yes| Z2[stop hold<br>RETURN]
    G -->|No| H[Calculate Velocity]
    
    H --> I[PD Control]
    I --> I1[out = KP × error - KD × velocity]
    
    I1 --> J[Soft Zone Limiting]
    J --> J1[distNearest = min distToUp, distToDown]
    J1 --> J2[soft = clamp distNearest / SOFT_ZONE]
    J2 --> J3[softCurve = soft²]
    J3 --> J4[maxNow = MAX_PWR × MIN_SCALE + 1-MIN_SCALE × softCurve]
    
    J4 --> K[Apply Power Limits]
    K --> K1[out = clamp out, -maxNow, maxNow]
    
    K1 --> L{abs out < MIN_PWR?}
    L -->|Yes| M[out = ±MIN_PWR]
    L -->|No| N[Keep out]
    
    M --> O[Final Clamp -100, 100]
    N --> O
    
    O --> P[DescoreMotor.spin out]
```

---

## 10. Color Sorting Task

```mermaid
flowchart TD
    A[intakeTaskFn Start] --> B[ballSensor.setLightPower 100]
    
    B --> C{Loop Forever}
    
    C --> D{ballSensor.isNearObject?}
    D -->|No| E[wait 20ms]
    E --> C
    
    D -->|Yes| F[Median Filter]
    F --> F1[Add hue to buffer]
    F1 --> F2[Keep last 5 samples]
    F2 --> F3[Sort buffer]
    F3 --> F4[hue = median value]
    
    F4 --> G[Color Detection]
    G --> G1[isRed = hue < 20 OR hue > 340]
    G --> G2[isBlue = hue > 200 AND hue < 250]
    
    G1 --> H{Opposing Alliance?}
    G2 --> H
    
    H --> H1{myAlliance == RED<br>AND isBlue?}
    H --> H2{myAlliance == BLUE<br>AND isRed?}
    
    H1 -->|Yes| I[Eject Sequence]
    H2 -->|Yes| I
    H1 -->|No| E
    H2 -->|No| E
    
    I --> I1[reverseIntake 100]
    I1 --> I2[reverseOutake 100]
    I2 --> I3[wait 300ms]
    I3 --> I4[runIntake 100]
    I4 --> I5[runOutake 100]
    
    I5 --> E
```

---

## 11. Slew Rate Limiter

```mermaid
flowchart TD
    A[slewStep target, current] --> B[delta = target - current]
    
    B --> C{abs target > abs current?}
    
    C -->|Yes Accelerating| D[maxStep = ACCEL_PCT_PER_S × DT]
    C -->|No Decelerating| E[maxStep = DECEL_PCT_PER_S × DT]
    
    D --> F[Clamp Delta]
    E --> F
    
    F --> G{delta > maxStep?}
    G -->|Yes| H[delta = maxStep]
    G -->|No| I{delta < -maxStep?}
    
    I -->|Yes| J[delta = -maxStep]
    I -->|No| K[Keep delta]
    
    H --> L[return current + delta]
    J --> L
    K --> L
```

---

## Summary Timing Diagram

```mermaid
gantt
    title Task Timing (100ms window)
    dateFormat X
    axisFormat %L
    
    section Odometry
    Odom Update :a1, 0, 1
    Odom Update :a2, 10, 1
    Odom Update :a3, 20, 1
    Odom Update :a4, 30, 1
    Odom Update :a5, 40, 1
    Odom Update :a6, 50, 1
    Odom Update :a7, 60, 1
    Odom Update :a8, 70, 1
    Odom Update :a9, 80, 1
    Odom Update :a10, 90, 1
    
    section Color Sorter
    Sort Check :b1, 0, 2
    Sort Check :b2, 20, 2
    Sort Check :b3, 40, 2
    Sort Check :b4, 60, 2
    Sort Check :b5, 80, 2
    
    section Teleop
    Control Loop :c1, 0, 2
    Control Loop :c2, 20, 2
    Control Loop :c3, 40, 2
    Control Loop :c4, 60, 2
    Control Loop :c5, 80, 2
    
    section Main
    Watchdog :d1, 100, 1
```

---

*Mermaid Flowcharts for VEX V5 Competition Robot*
