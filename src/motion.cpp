#include "motion.h"
#include "drive.h"
#include "robot_config.h"
#include "utils.h"
#include "odom.h"
#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

static inline double rotDegToM(double deg) {
    return (deg * config::TRACKING_WHEEL_CIRCUMFERENCE_M) / 360.0;
}

static inline double norm360(double a) {
    a = std::fmod(a, 360.0);
    if (a < 0.0) a += 360.0;
    return a;
}

double MotionController::wrap180(double a) {
    while (a > 180.0)  a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

double MotionController::angleDiffDeg(double targetDeg, double currentDeg) {
    return wrap180(targetDeg - currentDeg);
}

MotionController::MotionController()
    : distPID_(5, 3.0, 0.004),
      headPID_(0.25, 0.001, 0.001),
      turnPID_(0.25, 0.001, 0.001)
{
    distPID_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    distPID_.setDerivativeFilterTf(0.10);
    distPID_.setAntiWindupTau(0.15);
    distPID_.setErrorDeadband(0.006);
    distPID_.setOutputLimits(-100, 100);

    headPID_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    headPID_.setDerivativeFilterTf(0.10);
    headPID_.setAntiWindupTau(0.20);
    headPID_.setIntegralZone(0.0);
    headPID_.setIntegralLimits(-2.0, 2.0);
    headPID_.setErrorDeadband(0.3);
    headPID_.setOutputLimits(-16, 16);

    turnPID_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    turnPID_.setDerivativeFilterTf(0.06);
    turnPID_.setAntiWindupTau(0.18);
    turnPID_.setIntegralZone(15.0);
    turnPID_.setIntegralLimits(-50.0, 50.0);
    turnPID_.setErrorDeadband(0.2);
    turnPID_.setOutputLimits(-60, 60);
}

void MotionController::driveHeading(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg) {
    const double startM = rotDegToM(verticalRot.position(deg));
    const double holdHead = norm360(holdHeadingDeg);

    distPID_.setSetpoint(distM);
    distPID_.resetBumpless(0.0, 0.0);

    headPID_.setSetpoint(0.0);
    headPID_.resetBumpless(0.0, 0.0);

    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    timer t; t.reset();
    int settledMs = 0;
    int stopHoldMs = 0;

    const double minCap = 10.0;
    const double stopBand = 0.020;

    const double stopEnter = 0.010;
    const double stopExit  = 0.020;
    const int stopSettleMs = 120;

    bool stopLatch = false;
    double distErrPrev = distM;
    bool crossed = false;

    double vCmd = 0.0;
    const double dvPerSec = 160.0;
    const double dvMax = dvPerSec * dt;

    double wCmd = 0.0;
    const double dwPerSec = 260.0;
    const double dwMax = dwPerSec * dt;

    while (t.time(msec) < timeoutMs) {
        const double currM = rotDegToM(verticalRot.position(deg));
        const double traveled = currM - startM;
        const double distErr = distM - traveled;

        if ((distErrPrev > 0.0 && distErr < 0.0) || (distErrPrev < 0.0 && distErr > 0.0)) crossed = true;
        distErrPrev = distErr;

        if (!stopLatch) {
            if (std::fabs(distErr) < stopEnter || (crossed && std::fabs(distErr) < stopExit)) {
                stopLatch = true;
                vCmd = 0.0;
                wCmd = 0.0;
                stopHoldMs = 0;
                distPID_.resetBumpless(traveled, 0.0);
            }
        } else {
            if (std::fabs(distErr) > stopExit) {
                stopLatch = false;
                stopHoldMs = 0;
            }
        }

        if (stopLatch) {
            const double currHead = inertial_sensor.heading(deg);
            const double headErr = angleDiffDeg(holdHead, currHead);

            double w = headPID_.update(-headErr, dt);

            const double headAbs = std::fabs(headErr);
            if (headAbs < 1.0) {
                w = 0.0;
                wCmd = 0.0;
            } else {
                wCmd += clampD(w - wCmd, -dwMax, dwMax);
                w = wCmd;
            }

            tankDrive(w, -w);

            stopHoldMs += dtMs;
            if (stopHoldMs >= stopSettleMs) break;

            wait(dtMs, msec);
            continue;
        }

        double cap = minCap + 200.0 * std::fabs(distErr);
        cap = std::min(cap, maxSpeedPct);
        distPID_.setOutputLimits(-cap, cap);

        double v = distPID_.update(traveled, dt);

        if (std::fabs(distErr) < stopBand) {
            v = 0.0;
            vCmd = 0.0;
        } else {
            if (std::fabs(distErr) > 0.12) {
                const double floor = 8.0;
                if (std::fabs(v) < floor) v = (distErr > 0.0) ? floor : -floor;
            }
        }

        if (std::fabs(distErr) < 0.03 && std::fabs(v) < 12.0 && (v * distErr) < 0.0) {
            v = 0.0;
        }

        vCmd += clampD(v - vCmd, -dvMax, dvMax);
        v = vCmd;

        const double currHead = inertial_sensor.heading(deg);
        const double headErr = angleDiffDeg(holdHead, currHead);

        double w = headPID_.update(-headErr, dt);

        const double vAbs = std::fabs(v);
        double wScale = std::min(1.0, vAbs / 18.0);
        wScale = std::max(wScale, 0.25);
        if (vAbs < 12.0 && std::fabs(headErr) > 2.0) wScale = std::max(wScale, 0.22);
        w *= wScale;

        wCmd += clampD(w - wCmd, -dwMax, dwMax);
        w = wCmd;

        tankDrive(v + w, v - w);

        if (std::fabs(distErr) < 0.02 && std::fabs(vCmd) < 6.0) {
            settledMs += dtMs;
            if (settledMs >= 40) break;
        } else {
            settledMs = 0;
        }

        wait(dtMs, msec);
    }

    stopDrive(brake);
}

void MotionController::drive(double distM, int timeoutMs, double maxSpeedPct) {
    driveHeading(distM, timeoutMs, maxSpeedPct, inertial_sensor.heading(deg));
}

void MotionController::turnTo(double targetDeg, int timeoutMs) {
    turnPID_.setSetpoint(0.0);

    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    timer t; t.reset();
    int settledMs = 0;

    double prevHead = inertial_sensor.heading(deg);
    double rateFilt = 0.0;
    const double rateTau = 0.06;
    const double alpha = dt / (rateTau + dt);
    const double rateTol = 8.0;

    {
        const double curr = inertial_sensor.heading(deg);
        const double err0 = angleDiffDeg(targetDeg, curr);
        turnPID_.resetBumpless(-err0, 0.0);
    }

    while (t.time(msec) < timeoutMs) {
        const double curr = inertial_sensor.heading(deg);
        const double err = angleDiffDeg(targetDeg, curr);

        const double dHead = wrap180(curr - prevHead);
        prevHead = curr;
        const double rate = dHead / dt;
        rateFilt += alpha * (rate - rateFilt);

        const double turnOut = turnPID_.update(-err, dt);
        tankDrive(turnOut, -turnOut);

        if (std::fabs(err) < 1.0 && std::fabs(rateFilt) < rateTol) {
            settledMs += dtMs;
            if (settledMs >= 150) break;
        } else {
            settledMs = 0;
        }

        wait(dtMs, msec);
    }

    stopDrive(brake);
}

void MotionController::turnBy(double deltaDeg, int timeoutMs) {
    const double startRot = inertial_sensor.rotation(vex::deg);
    const double targetRot = startRot + deltaDeg;

    timer t; t.reset();
    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    turnPID_.setSetpoint(0.0);

    double prevRot = inertial_sensor.rotation(vex::deg);
    double rateFilt = 0.0;
    const double rateTau = 0.06;
    const double alpha = dt / (rateTau + dt);
    const double rateTol = 8.0;

    {
        const double err0 = targetRot - inertial_sensor.rotation(vex::deg);
        turnPID_.resetBumpless(-err0, 0.0);
    }

    int settledMs = 0;

    while (t.time(vex::msec) < timeoutMs) {
        const double rot = inertial_sensor.rotation(vex::deg);
        const double err = targetRot - rot;

        const double dRot = rot - prevRot;
        prevRot = rot;
        const double rate = dRot / dt;
        rateFilt += alpha * (rate - rateFilt);

        const double out = turnPID_.update(-err, dt);
        tankDrive(out, -out);

        if (std::fabs(err) < 1.0 && std::fabs(rateFilt) < rateTol) {
            settledMs += dtMs;
            if (settledMs >= 150) break;
        } else {
            settledMs = 0;
        }

        wait(dtMs, vex::msec);
    }

    stopDrive(vex::brake);
}

void MotionController::autoCorrect(double targetX, double targetY, double targetHeadingDeg,
                                   int timeoutMs, double maxSpeedPct) {
    const double ENTER_DIST = 0.12;
    const double EXIT_DIST  = 0.07;

    const double ENTER_ANG  = 6.0;
    const double EXIT_ANG   = 3.0;

    const double ENTER_POS  = 0.06;
    const double EXIT_POS   = 0.03;

    const double MAX_STEP_M = 0.20;

    const double BACKUP_M   = 0.05;

    const double corrSpeed = std::min(maxSpeedPct, 30.0);

    int turnTimeout   = std::max(450, timeoutMs * 4 / 10);
    int driveTimeout  = std::max(650, timeoutMs * 5 / 10);
    int backupTimeout = std::max(450, timeoutMs * 4 / 10);

    bool engagedPos = false;
    bool didBackup  = false;

    for (int iter = 0; iter < 2; ++iter) {
        wait(20, msec);

        double dx = targetX - robotPose.x;
        double dy = targetY - robotPose.y;

        double distNow = std::hypot(dx, dy);

        double currHead = inertial_sensor.heading(vex::deg);
        double headErrAbs = std::fabs(angleDiffDeg(targetHeadingDeg, currHead));

        if (distNow < EXIT_DIST && headErrAbs < EXIT_ANG) break;

        const double h0 = degToRad(targetHeadingDeg);
        const double fwdRel0 = dx * std::sin(h0) + dy * std::cos(h0);

        const bool needsCorrection =
            (distNow > ENTER_DIST) || (headErrAbs > ENTER_ANG) ||
            (std::fabs(dx) > ENTER_POS) || (std::fabs(dy) > ENTER_POS);

        if (needsCorrection && !didBackup && fwdRel0 > 0.02) {
            driveHeading(-BACKUP_M, backupTimeout, corrSpeed, targetHeadingDeg);
            wait(20, msec);
            turnTo(targetHeadingDeg, turnTimeout);
            wait(20, msec);
            didBackup = true;
        } else if (headErrAbs > ENTER_ANG) {
            turnTo(targetHeadingDeg, turnTimeout);
            wait(20, msec);
        }

        dx = targetX - robotPose.x;
        dy = targetY - robotPose.y;
        distNow = std::hypot(dx, dy);

        const double h = degToRad(targetHeadingDeg);
        double fwd = dx * std::sin(h) + dy * std::cos(h);
        double lat = dx * std::cos(h) - dy * std::sin(h);

        if (!engagedPos) {
            if (distNow > ENTER_DIST) engagedPos = true;
            if (!engagedPos && (std::fabs(lat) > ENTER_POS || std::fabs(fwd) > ENTER_POS)) engagedPos = true;
        }

        if (engagedPos) {
            if (std::fabs(lat) < EXIT_POS) lat = 0.0;
            if (std::fabs(fwd) < EXIT_POS) fwd = 0.0;

            if (std::fabs(lat) > ENTER_POS) {
                double latStep = clampD(lat, -MAX_STEP_M, MAX_STEP_M);
                double slideHeading = wrap180(targetHeadingDeg + (latStep > 0.0 ? +90.0 : -90.0));

                turnTo(slideHeading, turnTimeout);
                driveHeading(std::fabs(latStep), driveTimeout, corrSpeed, slideHeading);
                turnTo(targetHeadingDeg, turnTimeout);

                wait(20, msec);
            }

            dx = targetX - robotPose.x;
            dy = targetY - robotPose.y;

            const double h2 = degToRad(targetHeadingDeg);
            fwd = dx * std::sin(h2) + dy * std::cos(h2);

            if (std::fabs(fwd) < EXIT_POS) fwd = 0.0;

            if (std::fabs(fwd) > ENTER_POS) {
                turnTo(targetHeadingDeg, turnTimeout);
                wait(20, msec);

                double fwdStep = clampD(fwd, -MAX_STEP_M, MAX_STEP_M);
                driveHeading(fwdStep, driveTimeout, corrSpeed, targetHeadingDeg);
                wait(20, msec);
            }
        }

        dx = targetX - robotPose.x;
        dy = targetY - robotPose.y;
        distNow = std::hypot(dx, dy);

        currHead = inertial_sensor.heading(vex::deg);
        headErrAbs = std::fabs(angleDiffDeg(targetHeadingDeg, currHead));

        if (distNow < EXIT_DIST && headErrAbs < EXIT_ANG) break;
    }
}

void MotionController::driveAC(double distM, int timeoutMs, double maxSpeedPct,
                              int correctTimeoutMs, double correctSpeedPct) {
    if (!autoCorrectEnabled_) {
        drive(distM, timeoutMs, maxSpeedPct);
        return;
    }

    Pose s = robotPose;
    double holdHead = inertial_sensor.heading(vex::deg);

    drive(distM, timeoutMs, maxSpeedPct);

    double gx = s.x + distM * std::sin(s.theta);
    double gy = s.y + distM * std::cos(s.theta);

    autoCorrect(gx, gy, holdHead, correctTimeoutMs, correctSpeedPct);
}

void MotionController::driveHeadingAC(double distM, int timeoutMs, double maxSpeedPct, double holdHeadingDeg,
                                     int correctTimeoutMs, double correctSpeedPct) {
    if (!autoCorrectEnabled_) {
        driveHeading(distM, timeoutMs, maxSpeedPct, holdHeadingDeg);
        return;
    }

    Pose s = robotPose;

    driveHeading(distM, timeoutMs, maxSpeedPct, holdHeadingDeg);

    double gx = s.x + distM * std::sin(s.theta);
    double gy = s.y + distM * std::cos(s.theta);

    autoCorrect(gx, gy, holdHeadingDeg, correctTimeoutMs, correctSpeedPct);
}

void MotionController::turnToAC(double targetDeg, int timeoutMs,
                               int correctTimeoutMs, double correctSpeedPct) {
    if (!autoCorrectEnabled_) {
        turnTo(targetDeg, timeoutMs);
        return;
    }

    Pose s = robotPose;

    turnTo(targetDeg, timeoutMs);

    autoCorrect(s.x, s.y, targetDeg, correctTimeoutMs, correctSpeedPct);
}

void MotionController::turnByAC(double deltaDeg, int timeoutMs,
                               int correctTimeoutMs, double correctSpeedPct) {
    if (!autoCorrectEnabled_) {
        turnBy(deltaDeg, timeoutMs);
        return;
    }

    Pose s = robotPose;
    double startHead = inertial_sensor.heading(vex::deg);
    double targetHead = norm360(startHead + deltaDeg);

    turnBy(deltaDeg, timeoutMs);

    autoCorrect(s.x, s.y, targetHead, correctTimeoutMs, correctSpeedPct);
}
