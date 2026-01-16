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
    : distPID_(0.2, 0.0, 0.0),
      headPID_(0.2, 0.0, 0.0),
      turnPID_(0.2, 0.0, 0.0)
{
    distPID_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    distPID_.setDerivativeFilterTf(0.10);
    distPID_.setAntiWindupTau(0.15);
    distPID_.setErrorDeadband(0.010);
    distPID_.setOutputLimits(-100, 100);

    headPID_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    headPID_.setDerivativeFilterTf(0.05);
    headPID_.setAntiWindupTau(0.20);
    headPID_.setIntegralZone(0.0);
    headPID_.setIntegralLimits(-2.0, 2.0);
    headPID_.setOutputLimits(-12, 12);

    turnPID_.setDerivativeMode(PID::DerivativeMode::OnMeasurement);
    turnPID_.setDerivativeFilterTf(0.06);
    turnPID_.setAntiWindupTau(0.18);
    turnPID_.setIntegralZone(15.0);
    turnPID_.setIntegralLimits(-50.0, 50.0);
    turnPID_.setErrorDeadband(0.3);
    turnPID_.setOutputLimits(-60, 60);
}

void MotionController::drive(double distM, int timeoutMs, double maxSpeedPct) {
    const double startM = rotDegToM(verticalRot.position(deg));
    const double holdHead = inertial_sensor.heading(deg);

    distPID_.setSetpoint(distM);
    distPID_.resetBumpless(0.0, 0.0);

    headPID_.setSetpoint(0.0);
    headPID_.resetBumpless(0.0, 0.0);

    const int dtMs = 10;
    const double dt = dtMs / 1000.0;

    timer t; t.reset();
    int settledMs = 0;

    const double minCap   = 10.0;
    const double stopBand = 0.015;
    // const double kFloor   = 450.0;

    double vCmd = 0.0;
    const double dvPerSec = 160.0;
    const double dvMax = dvPerSec * dt;

    while (t.time(msec) < timeoutMs) {
        const double currM = rotDegToM(verticalRot.position(deg));
        const double traveled = currM - startM;
        const double distErr = distM - traveled;

        double cap = minCap + 200.0 * std::fabs(distErr);
        cap = std::min(cap, maxSpeedPct);
        distPID_.setOutputLimits(-cap, cap);

        double v = distPID_.update(traveled, dt);

        if (std::fabs(distErr) < stopBand) {
            v = 0.0;
            vCmd = 0.0;
        } else {
            if(std::fabs(distErr) > 0.05){
                const double floor = 8.0;
                if(std::fabs(v) < floor) v = (distErr > 0.0) ? floor : -floor;
            }
        }

        if (std::fabs(distErr) < 0.03 && std::fabs(v) < 12.0 && (v * distErr) < 0.0) {
            v = 0.0;
        }

        vCmd += clampD(v - vCmd, -dvMax, dvMax);
        v = vCmd;

        const double currHead = inertial_sensor.heading(deg);
        const double headErr  = angleDiffDeg(holdHead, currHead);
        double w = headPID_.update(-headErr, dt);

        if (std::fabs(distErr) < 0.04) {
            w = 0.0;
        } else {
            const double vAbs = std::fabs(v);
            const double wScale = std::min(1.0, vAbs / 18.0);
            w *= wScale;
        }

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
        const double err  = angleDiffDeg(targetDeg, curr);

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
    const double startRot  = inertial_sensor.rotation(vex::deg);
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