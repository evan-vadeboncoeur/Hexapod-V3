#ifndef HEXAPOD_H
#define HEXAPOD_H

#define LEG_0 (0)
#define LEG_1 (1)
#define LEG_2 (2)
#define LEG_3 (3)
#define LEG_4 (4)
#define LEG_5 (5)
#define LEG_0_J0 (7)
#define LEG_0_J1 (8)
#define LEG_0_J2 (9)
#define LEG_1_J0 ()
#define LEG_1_J1 ()
#define LEG_1_J2 ()
#define LEG_2_J0 ()
#define LEG_2_J1 ()
#define LEG_2_J2 ()
#define LEG_3_J0 ()
#define LEG_3_J1 ()
#define LEG_3_J2 ()
#define LEG_4_J0 ()
#define LEG_4_J1 ()
#define LEG_4_J2 ()
#define LEG_5_J0 ()
#define LEG_5_J1 ()
#define LEG_5_J2 ()

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