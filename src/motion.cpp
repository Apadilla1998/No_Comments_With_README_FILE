#include "motion.h"
#include "drive.h"
#include "robot_config.h"
#include "utils.h"
#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

static inline double rotDegToM(double deg) {
    return (deg * config::TRACKING_WHEEL_CIRCUMFERENCE_M) / 360.0;
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
    : distPD_(0.1, 0.0, 0.0),
      headPD_(1.5, 0.0, 0.04),
      turnPD_(3.5, 0.0, 0.35)
{
    distPD_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    distPD_.setDerivativeFilterTf(0.10);
    distPD_.setErrorDeadband(0.003);
    distPD_.setOutputLimits(-100, 100);

    headPD_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    headPD_.setDerivativeFilterTf(0.05);
    headPD_.setOutputLimits(-12, 12);

    turnPD_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    turnPD_.setDerivativeFilterTf(0.06);
    turnPD_.setErrorDeadband(0.3);
    turnPD_.setOutputLimits(-60, 60);
}

void MotionController::drive(double distM, int timeoutMs, double maxSpeedPct) {
    const double startM = rotDegToM(verticalRot.position(deg));
    const double holdHead = inertial_sensor.heading(deg);

    distPD_.setSetpoint(distM);
    distPD_.resetBumpless(0.0, 0.0);

    headPD_.setSetpoint(0.0);
    headPD_.resetBumpless(0.0, 0.0);

    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    timer t; t.reset();
    int settledMs = 0;

    const double minCap   = 10.0;
    const double stopBand = 0.008;
    const double kFloor   = 450.0;

    double vCmd = 0.0;
    const double dvPerSec = 160.0;
    const double dvMax = dvPerSec * dt;

    while (t.time(msec) < timeoutMs) {
        const double currM = rotDegToM(verticalRot.position(deg));
        const double traveled = currM - startM;
        const double distErr = distM - traveled;

        double cap = minCap + 200.0 * std::fabs(distErr);
        cap = std::min(cap, maxSpeedPct);
        distPD_.setOutputLimits(-cap, cap);

        double v = distPD_.update(traveled, dt);

        if (std::fabs(distErr) < stopBand) {
            v = 0.0;
        } else {
            const double floor = std::min(minCap, kFloor * std::fabs(distErr));
            if (std::fabs(v) < floor) v = (distErr > 0.0) ? floor : -floor;
        }

        if (std::fabs(distErr) < 0.03 && std::fabs(v) < 12.0 && (v * distErr) < 0.0) {
            v = 0.0;
        }

        vCmd += clampD(v - vCmd, -dvMax, dvMax);
        v = vCmd;

        const double currHead = inertial_sensor.heading(deg);
        const double headErr  = angleDiffDeg(holdHead, currHead);
        double w = headPD_.update(-headErr, dt);

        if (std::fabs(distErr) < 0.04) {
            w = 0.0;
        } else {
            const double vAbs = std::fabs(v);
            const double wScale = std::min(1.0, vAbs / 18.0);
            w *= wScale;
        }

        tankDrive(v + w, v - w);

        if (std::fabs(distErr) < 0.015 && std::fabs(v) < 10.0) {
            settledMs += dtMs;
            if (settledMs >= 120) break;
        } else {
            settledMs = 0;
        }

        wait(dtMs, msec);
    }

    stopDrive(brake);
}

void MotionController::turnTo(double targetDeg, int timeoutMs) {
    turnPD_.setSetpoint(0.0);

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
        turnPD_.resetBumpless(-err0, 0.0);
    }

    while (t.time(msec) < timeoutMs) {
        const double curr = inertial_sensor.heading(deg);
        const double err  = angleDiffDeg(targetDeg, curr);

        const double dHead = wrap180(curr - prevHead);
        prevHead = curr;
        const double rate = dHead / dt;
        rateFilt += alpha * (rate - rateFilt);

        const double turnOut = turnPD_.update(-err, dt);
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
    const double startRot  = inertial_sensor.rotation(vex::deg);
    const double targetRot = startRot + deltaDeg;

    timer t; t.reset();
    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    turnPD_.setSetpoint(0.0);

    double prevRot = inertial_sensor.rotation(vex::deg);
    double rateFilt = 0.0;
    const double rateTau = 0.06;
    const double alpha = dt / (rateTau + dt);
    const double rateTol = 8.0;

    {
        const double err0 = targetRot - inertial_sensor.rotation(vex::deg);
        turnPD_.resetBumpless(-err0, 0.0);
    }

    int settledMs = 0;

    while (t.time(vex::msec) < timeoutMs) {
        const double rot = inertial_sensor.rotation(vex::deg);
        const double err = targetRot - rot;

        const double dRot = rot - prevRot;
        prevRot = rot;
        const double rate = dRot / dt;
        rateFilt += alpha * (rate - rateFilt);

        const double out = turnPD_.update(-err, dt);
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