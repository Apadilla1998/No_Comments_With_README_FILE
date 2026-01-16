#include "drive.h"
#include "robot_config.h"
#include "utils.h"

using namespace vex;

void tankDrive(double leftPct, double rightPct) {
    LeftMotorGroup.spin(fwd, clampPct(leftPct), percent);
    RightMotorGroup.spin(fwd, clampPct(rightPct), percent);
}

void arcadeDrive(double fwdPct, double turnPct) {
    tankDrive(fwdPct + turnPct, fwdPct - turnPct);
}

void stopDrive(brakeType mode) {
    LeftMotorGroup.stop(mode);
    RightMotorGroup.stop(mode);
}