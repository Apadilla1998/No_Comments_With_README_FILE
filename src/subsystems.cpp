#include "subsystems.h"
#include "robot_config.h"
#include <algorithm>
#include <vector>

using namespace vex;

Alliance myAlliance = BLUE;
Wings wings;


void Wings::toggle() {
    state = !state;
    wingsPiston.set(state);
}

void Wings::set(bool s) {
    state = s;
    wingsPiston.set(state);
}

void runIntake(double speedPct) {
    IntakeLeft.spin(fwd, speedPct, pct);
    IntakeRight.spin(fwd, speedPct, pct);
    Midtake.spin(fwd, speedPct, pct);
}
void reverseIntake(double speedPct) {
    IntakeLeft.spin(reverse, speedPct, pct);
    IntakeRight.spin(reverse, speedPct, pct);
    Midtake.spin(reverse, speedPct, pct);
}
void stopIntake() {
    IntakeLeft.stop(coast);
    IntakeRight.stop(coast);
    Midtake.stop(coast);
}

void runOutake(double speedPct) {
    RightOutake.spin(fwd, speedPct, pct);
    LeftOutake.spin(fwd, speedPct, pct);
    MidOutake.spin(fwd, speedPct, pct);
}
void reverseOutake(double speedPct) {
    RightOutake.spin(reverse, speedPct, pct);
    LeftOutake.spin(reverse, speedPct, pct);
    MidOutake.spin(reverse, speedPct, pct);
}
void stopOutake() {
    RightOutake.stop(coast);
    LeftOutake.stop(coast);
    MidOutake.stop(coast);
}

void moveArmRight(double speedPct){
    DescoreMotor.spin(fwd, speedPct, pct);
}

void moveArmLeft(double speedPct){
    DescoreMotor.spin(reverse, speedPct, pct);
}

void stopArm(){
    DescoreMotor.stop(hold);
}

static double filteredHue() {
    static std::vector<double> buf;
    buf.push_back(ballSensor.hue());
    if (buf.size() > 5) buf.erase(buf.begin());

    auto sorted = buf;
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2];
}

int intakeTaskFn() {
    ballSensor.setLightPower(100, percent);

    while (true) {
        if (ballSensor.isNearObject()) {
            double hue = filteredHue();

            bool isRed  = (hue < 20 || hue > 340);
            bool isBlue = (hue > 200 && hue < 250);

            if ((myAlliance == RED  && isBlue) ||
                (myAlliance == BLUE && isRed)) {
                reverseIntake(100);
                reverseOutake(100);
                wait(300, msec);
                runIntake(100);
                runOutake(100);
            }
        }
        wait(20, msec);
    }
    return 0;
}