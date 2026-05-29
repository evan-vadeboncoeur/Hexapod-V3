#ifndef MTN_PLNR_H
#define MTN_PLNR_H

#define NUM_LEGS (6)

#include "Tripod.h"
#include <math.h>

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements

class MotionPlanner{
    private:
        Leg** legs;
        Leg* tp_1[3]; // points to leg pointers
        Leg* tp_2[3];
        float s;
        int steps, direction; 
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