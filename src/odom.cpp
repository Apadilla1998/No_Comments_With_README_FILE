#include "odom.h"
#include "robot_config.h"
#include "utils.h"
#include "sensors.h"
#include "vex.h"
#include <cmath>

using namespace vex;

Pose robotPose{0.0, 0.0, 0.0};

struct OdomState {
    double prevV_m = 0.0;
    double prevH_m = 0.0;
    double prevRotDeg = 0.0;
    double theta = 0.0;
};

static OdomState s;

static inline double degToMeters(double deg) {
    return (deg * config::TRACKING_WHEEL_CIRCUMFERENCE_M) / 360.0;
}

void resetOdometry() {
    robotPose = {0.0, 0.0, 0.0};

    inertial_sensor.setHeading(0, deg);
    inertial_sensor.setRotation(0, deg);

    verticalRot.resetPosition();
    horizontalRot.resetPosition();

    s.prevV_m = degToMeters(verticalDeg());
    s.prevH_m = degToMeters(horizontalDeg());
    s.prevRotDeg = rotationDeg();
    s.theta = degToRad(s.prevRotDeg);
    robotPose.theta = s.theta;
}

static int odomLoop() {
    s.prevV_m = degToMeters(verticalDeg());
    s.prevH_m = degToMeters(horizontalDeg());
    s.prevRotDeg = rotationDeg();
    s.theta = degToRad(s.prevRotDeg);
    robotPose.theta = s.theta;

    while (true) {
        const double v_m = degToMeters(verticalDeg());
        const double h_m = degToMeters(horizontalDeg());
        const double rotDeg = rotationDeg();

        const double dV = v_m - s.prevV_m;
        const double dH = h_m - s.prevH_m;

        const double dRotDeg = rotDeg - s.prevRotDeg;
        const double dT = degToRad(dRotDeg);
        const double avgT = s.theta + 0.5 * dT;

        const double dHc = dH - config::SIDE_OFFSET_M * dT;
        const double dVc = dV - config::VERT_OFFSET_M * dT;

        double lX = 0.0;
        double lY = 0.0;

        if (std::fabs(dT) < 1e-6) {
            lX = dHc;
            lY = dVc;
        } else {
            const double chord = 2.0 * std::sin(dT / 2.0);
            lX = chord * (dHc / dT);
            lY = chord * (dVc / dT);
        }

        robotPose.x += lX * std::cos(avgT) + lY * std::sin(avgT);
        robotPose.y += lY * std::cos(avgT) - lX * std::sin(avgT);

        s.theta += dT;
        robotPose.theta = s.theta;

        s.prevV_m = v_m;
        s.prevH_m = h_m;
        s.prevRotDeg = rotDeg;

        wait(10, msec);
    }
    return 0;
}

int odomTaskFn() {
    return odomLoop();
}

int odomTaskFn(void*) {
    return odomLoop();
}