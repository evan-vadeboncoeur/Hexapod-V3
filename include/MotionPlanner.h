#ifndef MTN_PLNR_H
#define MTN_PLNR_H

#include "Tripod.h"
#include <math.h>

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements

class MotionPlanner{
    private:
        Leg* legs; // pointer to array of legs (6)
        Leg tp_1[3];
        Leg tp_2[3];
        float s, steps, dir; 
    public:
        enum Gait {TRIPOD, RIPPLE, WAVE, QUADRUPED} gait=TRIPOD;
        MotionPlanner();
        MotionPlanner(Leg* legs);
        void setLocomotion(); // receives command from Hexapod after Hexapod receives transmission from controller. sets gait, direction, etc.
        void setGait(int);
        void setDirection(int);
        void setTargetSteps(int);
        bool tripodGait(int, int, int);
        void setupTripod(int, int, int);
        void setDistanceIncrement(int);
        bool waveGait();
        bool rippleGait();
        bool quadrapedGait();
        bool moveHome();
        bool moveStorage(); // folded up position, power down
        
};

#endif