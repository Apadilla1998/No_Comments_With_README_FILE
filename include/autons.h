#ifndef AUTONS_H
#define AUTONS_H

enum class AutonRoutine {
    NONE,
    TEST,
    RED_LEFT,
    RED_RIGHT,
    BLUE_LEFT,
    BLUE_RIGHT,
    SKILLS,
    TURN_TEST,
    DRIVE_TEST,
    SQUARE_TEST
};

extern AutonRoutine selectedAuton;

void runAutonomous();

#endif