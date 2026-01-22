#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "vex.h"

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::AUTO_CORRECT_BLUE_RIGHT;

static void blueRight() {
    MotionController m;

    m.drive(0.82, 6000, 80); //timeout and its percentage;
    wait(10, msec);

    m.turnTo(90, 5000);
    wait(10, msec);

    m.drive(0.29, 5000, 80);
    wait(10, msec);

    // runIntake(100);
    // wait(1, sec);

    // stopIntake();
    // wait(10, msec);
}

static void autoCorrectBlueRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    m.drive(0.885, 5200, 90); //timeout and percentage
    wait(10, vex::msec);

    m.turnTo(90, 5000);
    wait(10, vex::msec);

    runIntake(100);
    m.drive(0.380, 5000, 90); //was 0.365
    wait(10, msec);

    runIntake(100);
    wait(1.0, sec);

    m.drive(-0.3, 4500, 90);
    m.turnBy(175, 5500); //was 173 all issues start here
    wait(10, msec);

    stopIntake();
    wings.toggle();
    m.drive(0.590, 5000, 90); //was 0.557
    wait(10, msec);

    runIntake(100);
    runOutake(100);
    wait(4, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    m.drive(-0.20, 5000, 90);
    m.turnBy(180, 5000);
    wait(10, msec);

}

static void autoCorrectRedLeft(){
    MotionController m;
    m.setAutoCorrectEnabled(true);

    m.driveAC(0.82, 4000, 80); //timeout and percentage
    wait(10, vex::msec);

    m.turnTo(90, 4000);
    wait(10, vex::msec);

    m.driveAC(0.30, 4000, 80);
    wait(5000, msec);

    // runIntake(100);
    // wait(5, sec);  

    // stopIntake();
    m.driveHeadingAC(-0.05, 4000, 80, 90); //tmeour, speed pct, angle to hold
    wait(10, msec);


}

static void autoCorrectBlueLeft() {}

static void autoCorrectRedRight(){}

static void blueLeft(){}

void runAutonomous() {
    switch (selectedAuton) {
        case AutonRoutine::NONE:       break;
        case AutonRoutine::TEST:       //autonTest(); break;
        case AutonRoutine::RED_LEFT:   //redLeft(); break;
        case AutonRoutine::RED_RIGHT:  //redRight(); break;
        case AutonRoutine::BLUE_LEFT:  blueLeft(); break;
        case AutonRoutine::BLUE_RIGHT:  blueRight(); break;
        case AutonRoutine::AUTO_CORRECT_BLUE_RIGHT: autoCorrectBlueRight(); break;
        case AutonRoutine::AUTO_CORRECT_BLUE_LEFT: autoCorrectBlueLeft(); break;
        case AutonRoutine::AUTO_CORRECT_RED_RIGHT: autoCorrectRedRight(); break;
        case AutonRoutine::AUTO_CORRECT_RED_LEFT: autoCorrectRedLeft(); break;
        case AutonRoutine::SKILLS:     //skills(); break;
        default: break;
    }
}

