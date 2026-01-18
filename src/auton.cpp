#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "vex.h"

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::AUTO_CORRECT_BLUE_LEFT;

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

    m.driveAC(0.82, 4000, 80); //timeout and percentage
    wait(10, vex::msec);

    m.turnTo(90, 4000);
    wait(10, vex::msec);

    m.driveAC(0.30, 4000, 80);
    wait(10, msec);

    runIntake(100);
    wait(5, sec);  

    stopIntake();
    m.driveAC(0.40, 4000, 80);
    wait(10, msec);

    m.turnBy(180, 4000);
    wait(10, msec);

    // m.turnTo(90, 4000);
    // wait(10, msec);

    wings.toggle();
    m.driveAC(0.40, 4000, 80);
    wait(10, msec);

    runIntake(100);
    runOutake(100);
    wait(2000, msec);

    stopIntake();
    stopOutake();
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
    wait(10, msec);

    runIntake(100);
    wait(5, sec);  

    stopIntake();
    m.driveAC(-0.40, 4000, 80);
    wait(10, msec);

    m.turnBy(180, 4000);
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

