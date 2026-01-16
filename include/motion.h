#ifndef MOTION_H
#define MOTION_H

#include "vex.h"
#include "PID.h"

class MotionController {
public:
    MotionController();

    static double wrap180(double a);
    static double angleDiffDeg(double targetDeg, double currentDeg);

    void drive(double distM, int timeoutMs = 3000, double maxSpeedPct = 100.0);
    void turnTo(double targetDeg, int timeoutMs = 3000);
    void turnBy(double deltaDeg, int timeoutMs = 3000);

private:
    PID distPID_;
    PID headPID_;
    PID turnPID_;
};

#endif