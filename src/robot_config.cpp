#include "robot_config.h"
#include <cmath>

using namespace vex;

brain Brain;
controller Controller1;

motor LeftA(PORT3, ratio6_1, false);
motor LeftB(PORT19, ratio6_1, false);
motor LeftC(PORT10, ratio6_1, false);
motor_group LeftMotorGroup(LeftA, LeftB, LeftC);

motor RightA(PORT11, ratio6_1, true);
motor RightB(PORT13, ratio6_1, true);
motor RightC(PORT15, ratio6_1, true);
motor_group RightMotorGroup(RightA, RightB, RightC);

motor IntakeLeft(PORT16, ratio6_1, false);
motor IntakeRight(PORT5, ratio6_1, true);
motor Midtake(PORT8, ratio18_1, false);

motor RightOutake(PORT7, ratio6_1, true);
motor LeftOutake(PORT2, ratio6_1, false);
motor MidOutake(PORT20, ratio6_1, true);

digital_out wingsPiston(Brain.ThreeWirePort.B);
inertial inertial_sensor(PORT1);

rotation verticalRot(PORT9, false);
rotation horizontalRot(PORT6, false);

optical ballSensor(PORT21);

motor DescoreMotor(PORT17, ratio36_1, true);

namespace config {
    const double TRACK_WIDTH_M = 0.320;

    const double TRACKING_WHEEL_CIRCUMFERENCE_M = 0.050  * M_PI;

    const double ARCADE_DEADBAND = 0.0;

    const double SIDE_OFFSET_M = 0.0;
    const double VERT_OFFSET_M = 0.0;
}