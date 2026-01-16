#include "pre_auton.h"
#include "robot_config.h"
#include "sensors.h"
#include "odom.h"
#include "subsystems.h"
#include "vex.h"

using namespace vex;

static task* odomTask = nullptr;
static task* sorterTask = nullptr;

void pre_auton() {
    initSensors();
    resetOdometry();

    if (!odomTask)   odomTask   = new task(odomTaskFn);
    if (!sorterTask) sorterTask = new task(intakeTaskFn);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Sensors ready");
}