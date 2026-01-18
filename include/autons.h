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
    SQUARE_TEST,
    AUTO_CORRECT_BLUE_RIGHT,
    AUTO_CORRECT_BLUE_LEFT,
    AUTO_CORRECT_RED_RIGHT,
    AUTO_CORRECT_RED_LEFT
};

extern AutonRoutine selectedAuton;

void runAutonomous();

#endif