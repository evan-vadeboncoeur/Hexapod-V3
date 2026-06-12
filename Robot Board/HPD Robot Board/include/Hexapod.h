#ifndef HEXAPOD_H
#define HEXAPOD_H
//#define GLOBAL_DEBUG

// #define LEG_0 (0)
// #define LEG_1 (1)
// #define LEG_2 (2)
// #define LEG_3 (3)
// #define LEG_4 (4)
// #define LEG_5 (5)
// #define LEG_0_J0 (28)
// #define LEG_0_J1 (29)
// #define LEG_0_J2 (30)
// #define LEG_1_J0 (22)
// #define LEG_1_J1 (23)
// #define LEG_1_J2 (24)
// #define LEG_2_J0 (14)
// #define LEG_2_J1 (15)
// #define LEG_2_J2 (16)
// #define LEG_3_J0 (25)
// #define LEG_3_J1 (26)
// #define LEG_3_J2 (27)
// #define LEG_4_J0 (31)
// #define LEG_4_J1 (32)
// #define LEG_4_J2 (33)
// #define LEG_5_J0 (34)
// #define LEG_5_J1 (35)
// #define LEG_5_J2 (36)

#include "MotionPlanner.h"
#include "CommunicationManager.h"
#include "BoardManager.h"

class Hexapod{
    private:
        MotionPlanner plan;
        CommunicationManager comm;
        BoardManager battery;
    public:
        Hexapod(MotionPlanner m, BoardManager b); // constructor 1 (all objects instantiated)
        void opMode(); // set operation mode: teleop (0), computer/robot (1)
        void gaitSet(); // set gait (2 dipswitch array for 4 combinations of gait)
        void checkPower(); // checks robot battery power / operates LEDs
        void walkFor(int steps, int direction);
};

#endif