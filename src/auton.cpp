#include "autons.h"
#include "motion.h"
#include "subsystems.h"
#include "vex.h"

using namespace vex;

AutonRoutine selectedAuton = AutonRoutine::BLUE_RIGHT;

static void autonTest() {
    MotionController m;
    m.drive(0.8);
    m.turnBy(90);
    m.drive(0.4);
}

static void redLeft() {
    MotionController m;
    runIntake(100);
    m.drive(1.2);
    m.turnTo(90);
    m.drive(0.5);
    reverseIntake(100); reverseOutake(100);
    wait(800, msec);
    stopIntake(); stopOutake();
}

static void redRight() {
    MotionController m;
    m.drive(0.5, 25000, 70);
}

static void blueLeft() {
    MotionController m;
    wings.set(true);
    m.drive(1.0);
    m.turnBy(45);
    wings.set(false);
    m.drive(-0.6);
}

static void blueRight() {
    MotionController m;

    m.drive(0.992, 4000, 80); //timeout and its percentage;
    wait(10, msec);

    m.turnTo(90, 5000);
    wait(10, msec);

    m.drive(0.425, 4000, 80);
    wait(5, msec);

    runIntake(100);
    wait(2000, msec);

    stopIntake();
    m.drive(-0.400, 4000, 80);
    wait(10, msec);

    // m.turnBy(185, 4000);
    // wait(10, msec);

    // m.drive(0.40, 5000, 80);
    // wait(10, msec);

    // m.turnTo(-90, 5000);
    // wait(10, msec);

    // m.drive(0.43, 5000, 80);
    // wait(10, msec);

    // // runIntake(100);
    // // runOutake(100);
    // // wait(500, msec);

    // // stopIntake();
    // // stopOutake();
    // m.drive(-0.100, 4000, 80);
    // m.turnBy(180, 5000);
    // wait(10, msec);
}

static void skills() {
    MotionController m;
    runIntake(100);
    m.drive(1.0);
    stopIntake();
}

void runAutonomous() {
    switch (selectedAuton) {
        case AutonRoutine::NONE:       break;
        case AutonRoutine::TEST:       autonTest(); break;
        case AutonRoutine::RED_LEFT:   redLeft(); break;
        case AutonRoutine::RED_RIGHT:  redRight(); break;
        case AutonRoutine::BLUE_LEFT:  blueLeft(); break;
        case AutonRoutine::BLUE_RIGHT: blueRight(); break;
        case AutonRoutine::SKILLS:     skills(); break;

        case AutonRoutine::TURN_TEST: {
            MotionController m;
            m.turnBy(90);
        } break;

        case AutonRoutine::DRIVE_TEST: {
            MotionController m;
            m.drive(1.0);
        } break;

        case AutonRoutine::SQUARE_TEST: {
            MotionController m;
            for (int i=0;i<4;i++) { m.drive(0.8); m.turnBy(90); }
        } break;

        default: break;
    }
}