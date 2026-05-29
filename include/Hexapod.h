#ifndef HEXAPOD_H
#define HEXAPOD_H

#define LEG_0 (0)
#define LEG_1 (1)

#include "MotionPlanner.h"
#include "CommunicationManager.h"

class Hexapod{
    private:
        MotionPlanner motion;
        CommunicationManager comm;
    public:
        Hexapod(MotionPlanner plan); // constructor 1 (all objects instantiated)
        void opMode(); // set operation mode: teleop (0), computer/robot (1)
        void gaitSet(); // set gait (2 dipswitch array for 4 combinations of gait)
        void checkPower(); // checks robot battery power / operates LEDs
        void walkFor(int steps, int direction);

};



#endif