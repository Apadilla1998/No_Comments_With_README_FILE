# VEX Robotics - Algorithm Flowmap

```mermaid
flowchart LR
    subgraph Odometry["Odometry (10ms)"]
        direction TB
        enc["Δvertical, Δhorizontal"] --> arc["Arc Integration"]
        imu["IMU Δθ"] --> arc
        arc --> pose["x += Δs·sin(θ)<br/>y += Δs·cos(θ)"]
    end

    subgraph PID["PID Controller"]
        direction TB
        err["error = setpoint - measured"] --> P["P = Kp·error"]
        err --> I["I += Ki·error·dt"]
        err --> D["D = Kd·(filtered dError/dt)"]
        P & I & D --> sum["output = P + I + D"]
        sum --> clamp["clamp(outMin, outMax)"]
        clamp --> aw["Anti-windup backtrack"]
    end

    subgraph Motion["Motion Control"]
        direction TB
        drive["drive()"] --> distPID["distPID → velocity"]
        drive --> headPID["headPID → correction"]
        distPID & headPID --> tank["tank(v+w, v-w)"]
        
        turn["turnTo/By()"] --> turnPID["turnPID → rotation"]
        turnPID --> tankTurn["tank(out, -out)"]
        
        ac["autoCorrect()"] --> decomp["Decompose error<br/>fwd = Δx·sin(h) + Δy·cos(h)<br/>lat = Δx·cos(h) - Δy·sin(h)"]
        decomp --> correct["turnTo → drive → turnTo"]
    end

    subgraph Driver["Driver Input"]
        direction TB
        joy["joystick %"] --> curve["curve = c·v³ + (1-c)·v"]
        curve --> dead["deadband < 5% → 0"]
        dead --> slew["slew: Δ ≤ rate·dt"]
        slew --> bias["L×0.98, R×1.0"]
    end

    subgraph Arm["Arm Control"]
        direction TB
        armErr["error = target - angle"] --> armPD["PD = Kp·err - Kd·velocity"]
        armPD --> soft["Soft limit scaling<br/>near edges → reduce power"]
        soft --> hard["Hard limit cutoff"]
    end

    subgraph Sort["Ball Sorting"]
        direction TB
        hue["optical hue"] --> median["median filter (5 samples)"]
        median --> check{"red<20 or >340?<br/>blue 200-250?"}
        check -->|"wrong alliance"| eject["reverse 300ms"]
    end

    Odometry --> Motion
    PID --> Motion
    PID --> Arm
    Driver --> Motors["Motors"]
    Motion --> Motors
    Arm --> Motors
    Sort --> Motors
```

## Key Algorithms

| Algorithm | Formula/Logic |
|-----------|---------------|
| **Exponential Curve** | `out = c·v³ + (1-c)·v` where c=0.1 fwd, 0.3 turn |
| **Slew Rate** | `Δcmd ≤ 300%/s · dt` (accel/decel limited) |
| **Arc Odometry** | `chord = 2·sin(Δθ/2)`, integrate local→global |
| **PID** | `P + I + D` with derivative filter τ=0.05-0.10s |
| **Anti-windup** | Back-calculate integral on saturation |
| **Heading Hold** | Maintain heading during straight drive |
| **Auto-correct** | Decompose XY error into fwd/lateral, correct iteratively |
| **Arm Soft Limits** | `maxPower *= (dist/zone)²` near boundaries |
| **Ball Sort** | Median-filtered hue, alliance-based rejection |