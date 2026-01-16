#ifndef DRIVE_H
#define DRIVE_H

#include "vex.h"

void tankDrive(double leftPct, double rightPct);
void arcadeDrive(double fwdPct, double turnPct);
void stopDrive(vex::brakeType mode = vex::brakeType::brake);

#endif