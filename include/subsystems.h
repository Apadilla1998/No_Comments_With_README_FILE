#ifndef SUBSYSTEMS_H
#define SUBSYSTEMS_H

#include "vex.h"

enum Alliance { RED, BLUE };
extern Alliance myAlliance;

class Wings {
public:
    Wings() : state(false) {}
    void toggle();
    void set(bool s);
    bool isExtended() const { return state; }
private:
    bool state;
};

class Subsystems {
public:
    Subsystems() : state(false) {}
    void toggle();
    void set(bool s);
    bool isExtended() const { return state; }
private:
    bool state;
};

extern Wings wings;

void runIntake(double speedPct = 100.0);
void reverseIntake(double speedPct = 100.0);
void stopIntake();

void runOutake(double speedPct = 100.0);
void reverseOutake(double speedPct = 100.0);
void stopOutake();

int intakeTaskFn();

#endif