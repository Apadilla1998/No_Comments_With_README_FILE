// motion.h
#ifndef MOTION_H
#define MOTION_H

#include "PID.h"

class MotionController {
public:
    MotionController();

    void setAutoCorrectEnabled(bool enabled) { autoCorrectEnabled_ = enabled; }
    bool isAutoCorrectEnabled() const { return autoCorrectEnabled_; }

    static double wrap180(double a);
    static double angleDiffDeg(double targetDeg, double currentDeg);

    void drive(double distM, int timeoutMs = 4000, double maxSpeedPct = 80.0);
    void driveHeading(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg);

    void driveCC(double distM, int timeoutMs = 4000, double maxSpeedPct = 80.0);
    void driveHeadingCC(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg);

    void turnTo(double targetDeg, int timeoutMs = 4000);
    void turnBy(double deltaDeg, int timeoutMs = 4000);

    void autoCorrect(double targetX, double targetY, double targetHeadingDeg,
                     int timeoutMs = 900, double maxSpeedPct = 30.0);

    void driveAC(double distM, int timeoutMs = 4000, double maxSpeedPct = 80.0,
                 int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

    void driveHeadingAC(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg,
                        int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

    void turnToAC(double targetDeg, int timeoutMs = 4000,
                  int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

    void turnByAC(double deltaDeg, int timeoutMs = 4000,
                  int correctTimeoutMs = 900, double correctSpeedPct = 30.0);

private:
    PID distPID_;
    PID headPID_;
    PID turnPID_;
    bool autoCorrectEnabled_ = false;
};

#endif
