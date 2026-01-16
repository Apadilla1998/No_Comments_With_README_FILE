#include "manual.h"
#include "robot_config.h"
#include "subsystems.h"
#include "odom.h"
#include "utils.h"
#include <cmath>
#include <algorithm>

using namespace vex;

static const double FWD_CURVE  = 0.1;
static const double TURN_CURVE = 0.1;
static const double LEFT_BIAS  = 0.98;
static const double RIGHT_BIAS = 1.0;

const double ACCEL_PCT_PER_S = 300.0;
const double DECEL_PCT_PER_S = 300.0;
const double DT = 0.02;

static const double ARM_DOWN_DEG = 190.5;
static const double ARM_UP_DEG   = 76.0;

static const double ARM_MIN_PWR = 12.0;
static const double ARM_MAX_PWR = 60.0;

static const double ARM_DEADBAND = 3.0;

static double armTargetDeg = ARM_DOWN_DEG;
static bool   armActive    = false;

static double lCmd = 0.0;
static double rCmd = 0.0;

static void armUpdateSimple() {
    double currentDeg = Descore.angle(degrees);
    double error = armTargetDeg - currentDeg;

    if (currentDeg <= ARM_UP_DEG && error < 0) {
        DescoreMotor.stop(hold);
        return;
    }
    if (currentDeg >= ARM_DOWN_DEG && error > 0) {
        DescoreMotor.stop(hold);
        return;
    }

    double absErr = std::fabs(error);

    if (absErr < ARM_DEADBAND) {
        DescoreMotor.stop(hold);
        return;
    }

    double range = (ARM_DOWN_DEG - ARM_UP_DEG);
    if (range < 1e-6) range = 1.0;

    double pos = (currentDeg - ARM_UP_DEG) / range;
    if (pos < 0.0) pos = 0.0;
    if (pos > 1.0) pos = 1.0;

    double midFactor = 1.0 - 2.0 * std::fabs(pos - 0.5);
    if (midFactor < 0.0) midFactor = 0.0;

    midFactor = midFactor * midFactor;

    double mag = ARM_MIN_PWR + midFactor * (ARM_MAX_PWR - ARM_MIN_PWR);

    double power = (error > 0) ? mag : -mag;

    if (power >  100) power =  100;
    if (power < -100) power = -100;

    DescoreMotor.spin(fwd, power, pct);
}

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
        Controller1.Screen.print("A:%.0f  T:%.0f", Descore.angle(degrees), armTargetDeg);

        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("SPEED: %s", isFast ? "FAST" : "SLOW");

        Controller1.Screen.setCursor(3, 1);
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

    bool prevLeft  = Controller1.ButtonLeft.pressing();
    bool prevRight = Controller1.ButtonRight.pressing();

    bool isFast = true;
    bool showOdom = false;

    double startAngle = Descore.angle(degrees);
    if (std::fabs(startAngle - ARM_UP_DEG) < std::fabs(startAngle - ARM_DOWN_DEG)) {
        armTargetDeg = ARM_UP_DEG;
    } else {
        armTargetDeg = ARM_DOWN_DEG;
    }

    int screenTimer = 0;
    updateControllerScreen(isFast, showOdom);

    auto slewStep = [&](double target, double current) -> double {
        double delta = target - current;
        bool increasingMag = (std::fabs(target) > std::fabs(current));
        double maxStep = (increasingMag ? ACCEL_PCT_PER_S : DECEL_PCT_PER_S) * DT;
        if (delta >  maxStep) delta =  maxStep;
        if (delta < -maxStep) delta = -maxStep;
        return current + delta;
    };

    while (true) {
        double fwd = computeCurve(Controller1.Axis3.position(pct), FWD_CURVE);
        double trn = computeCurve(Controller1.Axis1.position(pct), TURN_CURVE);

        if (std::fabs(fwd) < 5.0) fwd = 0;
        if (std::fabs(trn) < 5.0) trn = 0;

        double speedScale = isFast ? 0.8 : 0.4;
        double lReq = (fwd + trn) * speedScale * LEFT_BIAS;
        double rReq = (fwd - trn) * speedScale * RIGHT_BIAS;

        lCmd = slewStep(lReq, lCmd);
        rCmd = slewStep(rReq, rCmd);

        LeftMotorGroup.spin(forward, lCmd, pct);
        RightMotorGroup.spin(forward, rCmd, pct);

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

        bool leftNow  = Controller1.ButtonLeft.pressing();
        bool rightNow = Controller1.ButtonRight.pressing();

        if ((leftNow != prevLeft) || (rightNow != prevRight)) {
            needsUpdate = true;
        }

        if (leftNow && !rightNow) {
            armTargetDeg = ARM_UP_DEG;
            armActive = true;
        }
        else if (rightNow && !leftNow) {
            armTargetDeg = ARM_DOWN_DEG;
            armActive = true;
        }
        else {
            armActive = false;
            DescoreMotor.stop(hold);
        }

        prevLeft  = leftNow;
        prevRight = rightNow;

        if (armActive) {
            armUpdateSimple();
        }

        if (Controller1.ButtonL1.pressing()) {
            runIntake(100); stopOutake();
        } else if (Controller1.ButtonL2.pressing()) {
            runIntake(100); runOutake(100);
        } else if (Controller1.ButtonR2.pressing()) {
            reverseIntake(100); reverseOutake(100);
        } else {
            stopIntake(); stopOutake();
        }

        screenTimer += 20;
        const int screenPeriodMs = showOdom ? 200 : 2000;

        if (needsUpdate || screenTimer >= screenPeriodMs) {
            updateControllerScreen(isFast, showOdom);
            screenTimer = 0;
        }

        wait(20, msec);
    }
}