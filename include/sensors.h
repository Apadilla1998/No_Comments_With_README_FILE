#ifndef SENSORS_H
#define SENSORS_H

#include "vex.h"
#include "robot_config.h"

using namespace vex;

inline void initSensors() {
    inertial_sensor.calibrate();
    while (inertial_sensor.isCalibrating()) wait(20, msec);

    inertial_sensor.setHeading(0, deg);
    inertial_sensor.setRotation(0, deg);

    verticalRot.resetPosition();
    horizontalRot.resetPosition();
}

inline double headingDeg()   { return inertial_sensor.heading(deg); }
inline double rotationDeg()  { return inertial_sensor.rotation(deg); }

inline double verticalDeg()   { return verticalRot.position(deg); }
inline double horizontalDeg() { return horizontalRot.position(deg); }

#endif