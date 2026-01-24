#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "vex.h"

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::AUTO_CORRECT_BLUE_RIGHT;

// static void SimpleAutonLeft(){
//     MotionController m;
//     m.setAutoCorrectEnabled(true);

//     m.drive(0.80, 5500, 100); //timeout and percentage
//     wait(10, vex::msec);

//     m.turnTo(90, 4000);
//     wait(10, vex::msec);

//     wings.toggle();
//     m.drive(0.74, 5000, 100);
//     wait(10, msec);

//     runOutake(80);
//     runIntake(100);
//     wait(1, sec);

//     stopIntake();
//     stopOutake();
//     wait(10, msec);
// }

static void SimpleAutonRight(){
    MotionController m;
    m.setAutoCorrectEnabled(true);

    moveArmLeft(100);
    wait(0.800, msec);

    stopArm();
    wait(10, msec);

    m.drive(1.12, 6000, 100); //1.38
    m.turnTo(-41.0, 6000);
    wait(10, msec);

    m.drive(0.10, 4500, 100);
    wait(10, msec);

    reverseOutake(100);
    reverseIntake(100);
    wait(2, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    m.drive(-0.40, 5000, 100);
    m.turnBy(180, 4000);
    wait(10, msec);
}

static void SimpleAutonLeft(){
    MotionController m;
    m.setAutoCorrectEnabled(true);

    moveArmLeft(100);
    wait(0.800, msec);

    stopArm();
    wait(10, msec);

    m.drive(1.22, 6000, 100); //1.38
    m.turnTo(40.5, 5000);
    wait(10, msec);

    runIntake(100);
    runOutake(40);
    wait(2, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

}

static void blueRight() {
    MotionController m;

    m.drive(0.82, 6000, 80); //timeout and its percentage;
    wait(10, msec);

    m.turnTo(90, 5000);
    wait(10, msec);

    m.drive(0.29, 5000, 80);
    wait(10, msec);
}

static void autoCorrectBlueRight() {
    MotionController m;
    m.setAutoCorrectEnabled(true);

    moveArmLeft(100);
    wait(10, msec);

    stopArm();
    wait(10, msec);

    m.drive(0.75, 5500, 100); //timeout and percentage
    wait(10, vex::msec);

    m.turnTo(90, 4000);
    wait(10, vex::msec);

    runIntake(100);
    runOutake(100);
    m.drive(0.29, 5500, 100); //was 0.365
    wait(10, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    m.drive(-0.3, 4500, 100);
    m.turnBy(-90, 4000);
    wait(10, msec);
    reverseIntake(100);
    wait(500, msec);
    stopIntake();
    wait(10, msec);
    m.turnBy(90, 4000);
    wait(10, msec);

    m.turnBy(132, 7500); //was 135 all issues start here
    wait(10, msec);
    
    // reverseIntake(100);
    // reverseOutake(100);
    // wait(500, msec);

    stopIntake();
    m.driveHeadingCC(1.18, 7000, 100, 225);
    wait(10, msec);

    reverseIntake(75);
    reverseOutake(100);
    wait(3, sec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    // m.drive(-0.4, 5000, 100);
    // m.turnBy(180, 4000);
    // wait(10, msec);

    // m.drive(0.7, 7000, 100);
    // wait(10, msec);
}

/*
    MotionController m;
    m.setAutoCorrectEnabled(true);

    m.drive(0.80, 5500, 100); //timeout and percentage
    wait(10, vex::msec);

    m.turnTo(90, 4000);
    wait(10, vex::msec);

    runIntake(100);
    runOutake(100);
    m.drive(0.36, 5000, 90); //was 0.365
    wait(10, msec);

    stopIntake();
    stopOutake();
    wait(10, msec);

    m.drive(-0.3, 4500, 90);
    m.turnBy(140, 5500); //was 135 all issues start here
    wait(10, msec);
    
    m.drive(1.60, 7000, 100);
    wait(10, msec);

    reverseIntake(100);
    wait(2, sec);

    stopIntake();
    wait(10, msec);

    m.drive(-0.4, 5000, 100);
    m.turnBy(180, 4000);
    wait(10, msec);

    m.drive(1, 7000, 100);
    wait(10, msec);
*/

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

static void autoCorrectBlueLeft() { runIntake(100); wait(2, sec); stopIntake(); wait(10, msec); }

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
        case AutonRoutine::SIMPLE_AUTON_LEFT: SimpleAutonLeft(); break;
        case AutonRoutine::SIMPLE_AUTON_RIGHT: SimpleAutonRight(); break;
        case AutonRoutine::SKILLS:     //skills(); break;
        default: break;
    }
}

