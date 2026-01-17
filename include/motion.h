// motion.h
#ifndef MOTION_H
#define MOTION_H

#include "vex.h"
#include "PID.h"

class MotionController {
public:
    MotionController();

    double wrap180(double a);
    double angleDiffDeg(double targetDeg, double currentDeg);

    void drive(double distM, int timeoutMs, double maxSpeedPct = 80.0);
    void turnTo(double targetDeg, int timeoutMs);
    void turnBy(double deltaDeg, int timeoutMs);

    void autoCorrect(double targetX, double targetY, double targetHeadingDeg,
                     int timeoutMs, double maxSpeedPct = 45.0);

    void driveAC(double distM, int timeoutMs, double maxSpeedPct = 80.0,
                 int correctTimeoutMs = 1200, double correctSpeedPct = 45.0);

    void turnToAC(double targetDeg, int timeoutMs,
                  int correctTimeoutMs = 900, double correctSpeedPct = 40.0);

    void turnByAC(double deltaDeg, int timeoutMs,
                  int correctTimeoutMs = 900, double correctSpeedPct = 40.0);

    void setAutoCorrectEnabled(bool en) { autoCorrectEnabled_ = en; }
    bool getAutoCorrectEnabled() const { return autoCorrectEnabled_; }

private:
    PID distPID_;
    PID headPID_;
    PID turnPID_;
    bool autoCorrectEnabled_ = true;
};

#endif
