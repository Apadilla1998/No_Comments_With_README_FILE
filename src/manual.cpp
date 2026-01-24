#include "manual.h"
#include "robot_config.h"
#include "subsystems.h"
#include "odom.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

using namespace vex;

static const int OUTTAKE_NORMAL_PCT   = 40;
static const int OUTTAKE_WINGS_UP_PCT = 100;

static const double OUTTAKE_ACCEL_PCT_PER_S = 600.0;
static const double OUTTAKE_DECEL_PCT_PER_S = 900.0;

static double outtakeCmd = 0.0;

static const double FWD_CURVE  = 0.2;
static const double TURN_CURVE = 0.250;
static const double LEFT_BIAS  = 0.95;
static const double RIGHT_BIAS = 1.0;

const double ACCEL_PCT_PER_S = 350.0;
const double DECEL_PCT_PER_S = 290.0;
const double DT = 0.02;

static const double DRIVE_SCALE_FAST = 0.80;
static const double DRIVE_SCALE_SLOW = 0.40;

static const double TURN_SCALE_FAST  = 0.55;
static const double TURN_SCALE_SLOW  = 0.35;
static const double TURN_MAX_PCT     = 80.0;

static const double TURN_ACCEL_PCT_PER_S = 900.0;
static const double TURN_DECEL_PCT_PER_S = 1100.0;

static double fwdCmd = 0.0;
static double trnCmd = 0.0;

static double computeCurve(double inputPct, double curve) {
    double v = inputPct / 100.0;
    return ((curve * std::pow(v, 3)) + ((1.0 - curve) * v)) * 100.0;
}

static inline double wrap360(double d) {
    while (d >= 360.0) d -= 360.0;
    while (d < 0.0)    d += 360.0;
    return d;
}

static void updateControllerScreen(bool isFast, bool showOdom) {
    Controller1.Screen.clearLine(1);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.clearLine(3);

    if (!showOdom) {
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("SPEED: %s", isFast ? "FAST" : "SLOW");

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("WINGS: %s", wings.isExtended() ? "UP" : "DOWN");
    } else {
        const double x_cm  = robotPose.x * 100.0;
        const double y_cm  = robotPose.y * 100.0;
        const double thDeg = wrap360(radToDeg(robotPose.theta));

        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("X:%6.1f cm", x_cm);
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("Y:%6.1f cm", y_cm);
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("T:%6.1f deg", thDeg);
    }
}

void usercontrol() {
    bool prevR1 = false, prevUp = false, prevX = false, prevY = false;

    bool isFast = true;
    bool showOdom = false;

    int screenTimer = 0;
    updateControllerScreen(isFast, showOdom);

    auto slewAxis = [&](double target, double current, double accel, double decel) -> double {
        double delta = target - current;
        bool increasingMag = (std::fabs(target) > std::fabs(current));
        double maxStep = (increasingMag ? accel : decel) * DT;
        delta = clampD(delta, -maxStep, maxStep);
        return current + delta;
    };

    LeftMotorGroup.setStopping(coast);
    RightMotorGroup.setStopping(coast);

    bool driveStopped = true;

    while (true) {
        double fwdIn = computeCurve(Controller1.Axis3.position(pct), FWD_CURVE);
        double trnIn = computeCurve(Controller1.Axis1.position(pct), TURN_CURVE);

        if (std::fabs(fwdIn) < 5.0) fwdIn = 0.0;
        if (std::fabs(trnIn) < 5.0) trnIn = 0.0;

        const bool neutralInput = (fwdIn == 0.0 && trnIn == 0.0);

        const double driveScale = isFast ? DRIVE_SCALE_FAST : DRIVE_SCALE_SLOW;
        const double turnScale  = isFast ? TURN_SCALE_FAST  : TURN_SCALE_SLOW;

        double fwdReq = fwdIn * driveScale;
        double trnReq = trnIn * turnScale;
        trnReq = clampD(trnReq, -TURN_MAX_PCT, TURN_MAX_PCT);

        if (neutralInput) {
            fwdReq = 0.0;
            trnReq = 0.0;
        }

        fwdCmd = slewAxis(fwdReq, fwdCmd, ACCEL_PCT_PER_S, DECEL_PCT_PER_S);
        trnCmd = slewAxis(trnReq, trnCmd, TURN_ACCEL_PCT_PER_S, TURN_DECEL_PCT_PER_S);

        double lOut = clampPct((fwdCmd + trnCmd) * LEFT_BIAS);
        double rOut = clampPct((fwdCmd - trnCmd) * RIGHT_BIAS);

        if (neutralInput) {
            if (std::fabs(fwdCmd) < 0.8 && std::fabs(trnCmd) < 0.8) {
                if (!driveStopped) {
                    LeftMotorGroup.stop();
                    RightMotorGroup.stop();
                    driveStopped = true;
                }
            } else {
                driveStopped = false;
                LeftMotorGroup.spin(forward, lOut, pct);
                RightMotorGroup.spin(forward, rOut, pct);
            }
        } else {
            driveStopped = false;
            LeftMotorGroup.spin(forward, lOut, pct);
            RightMotorGroup.spin(forward, rOut, pct);
        }

        bool needsUpdate = false;

        bool r1 = Controller1.ButtonR1.pressing();
        if (r1 && !prevR1) {
            isFast = !isFast;
            Controller1.rumble(".");
            needsUpdate = true;
        }
        prevR1 = r1;

        bool up = Controller1.ButtonUp.pressing();
        if (up && !prevUp) {
            wings.toggle();
            needsUpdate = true;
        }
        prevUp = up;

        bool x = Controller1.ButtonX.pressing();
        if (x && !prevX) {
            resetOdometry();
            Controller1.rumble("-");
            needsUpdate = true;
        }
        prevX = x;

        bool y = Controller1.ButtonY.pressing();
        if (y && !prevY) {
            showOdom = !showOdom;
            needsUpdate = true;
        }
        prevY = y;

        if (Controller1.ButtonL1.pressing() || Controller1.ButtonL2.pressing()) {
            runIntake(100);
        } else if (Controller1.ButtonR2.pressing()) {
            reverseIntake(100);
        } else {
            stopIntake();
        }

        if (Controller1.ButtonLeft.pressing()) {
            moveArmLeft(25);
        } else if (Controller1.ButtonRight.pressing()) {
            moveArmRight(25);
        } else {
            stopArm();
        }

        const int outtakeBase = wings.isExtended() ? OUTTAKE_WINGS_UP_PCT : OUTTAKE_NORMAL_PCT;

        double outtakeTarget = 0.0;
        if (Controller1.ButtonL2.pressing()) {
            outtakeTarget = +outtakeBase;
        } else if (Controller1.ButtonR2.pressing()) {
            outtakeTarget = -outtakeBase;
        } else {
            outtakeTarget = 0.0;
        }

        {
            double delta = outtakeTarget - outtakeCmd;
            bool increasingMag = (std::fabs(outtakeTarget) > std::fabs(outtakeCmd));
            double rate = increasingMag ? OUTTAKE_ACCEL_PCT_PER_S : OUTTAKE_DECEL_PCT_PER_S;
            double maxStep = rate * DT;

            if (delta >  maxStep) delta =  maxStep;
            if (delta < -maxStep) delta = -maxStep;

            outtakeCmd += delta;
        }

        if (std::fabs(outtakeCmd) < 1.0) {
            stopOutake();
            outtakeCmd = 0.0;
        } else if (outtakeCmd > 0) {
            runOutake((int)std::fabs(outtakeCmd));
        } else {
            reverseOutake((int)std::fabs(outtakeCmd));
        }

        screenTimer += 20;
        const int screenPeriodMs = showOdom ? 200 : 4000;

        if (needsUpdate || screenTimer >= screenPeriodMs) {
            updateControllerScreen(isFast, showOdom);
            screenTimer = 0;
        }

        wait(20, msec);
    }
}
