#include "vex.h"
#include "robot_config.h"
#include "pre_auton.h"
#include "autons.h"
#include "manual.h"

using namespace vex;

competition Competition;

static void autonomous() {
    runAutonomous();
}

int main() {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();

    while (true) {
        wait(100, msec);
    }
}