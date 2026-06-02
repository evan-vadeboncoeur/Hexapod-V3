#ifndef MTN_PLNR_H
#define MTN_PLNR_H
//#define PLAN_DEBUG


#define NUM_LEGS (6)
#define FWD (1.0)
#define RVRS (-1.0)

#define HOME_J0 (0)
#define HOME_J1 (0)
#define HOME_J2 (0)
#define HOME_X ()
#define HOME_Y ()
#define HOME_Z

#define STORAGE_J0
#define STORAGE_J1
#define STORAGE_J2
#define STORAGE_X
#define STORAGE_Y
#define STORAGE_Z

#define LIFT_J0
#define LIFT_J1
#define LIFT_J2
#define LIFT_X
#define LIFT_Y
#define LIFT_Z

#define SWING_J0
#define SWING_J1
#define SWING_J2
#define SWING_X
#define SWING_Y
#define SWING_Z

#define PLANT_J0
#define PLANT_J1
#define PLANT_J2
#define PLANT_X
#define PLANT_Y
#define PLANT_Z

#define PUSH_J0
#define PUSH_J1
#define PUSH_J2
#define PUSH_X
#define PUSH_Y
#define PUSH_Z


#include "Leg.h"
#include <math.h>

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements

class MotionPlanner{
    private:
        Leg** legs;
        Leg* tp_1[3]; // points to leg pointers
        Leg* tp_2[3];
        float s;
        int steps, direction; 
        // position definitions
        J_Position home = J_Position();
        J_Position storage = J_Position();
        J_Position stance = J_Position();
        J_Position lift = J_Position();
        J_Position swing = J_Position();
        J_Position plant = J_Position();
    public:
        enum Gait {TRIPOD, RIPPLE, WAVE, QUADRUPED} gait=TRIPOD;
        MotionPlanner();
        MotionPlanner(Leg** l, int g);
        void setLocomotion(); // receives command from Hexapod after Hexapod receives transmission from controller. sets gait, direction, etc.
        void setGait(int);
        void setDirection(int);
        void setTargetSteps(int);
        bool tripodGait(int, int, int);
        void setupTripod(int, int, int);
        void setDistanceIncrement(int);
        void sortTripod();
        bool waveGait();
        bool rippleGait();
        bool quadrapedGait();
        bool moveHome();
        bool moveStorage(); // folded up position, power down
        
        
};

#endif