#ifndef MTN_PLNR_H
#define MTN_PLNR_H
#define PLAN_DEBUG

#define NUM_LEGS (6)
#define FWD (1.0)
#define RVRS (-1.0)
#define TP_1 (1)
#define TP_2 (2)

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
#define LIFT_X (207.0)
#define LIFT_Y (0.0)
#define LIFT_Z (-34.0)

#define SWING_J0
#define SWING_J1
#define SWING_J2
#define SWING_X (200.0)
#define SWING_Y (53.575)
#define SWING_Z (-34.0)

#define PLANT_J0
#define PLANT_J1
#define PLANT_J2
#define PLANT_X (200.0)
#define PLANT_Y (53.575)
#define PLANT_Z (-80.0)

#define PUSH_J0
#define PUSH_J1
#define PUSH_J2
#define PUSH_X (200.0) // stance - radius movement (all in XY ground plane) [opposite of plant (Y axis reflection?)]
#define PUSH_Y (-SWING_Y) // probably right?
#define PUSH_Z (-80.0) // figure this out

#define MOTION_PLANNER_DELAY (50)

#include "Leg.h"
#include <math.h>

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements

class MotionPlanner{
    private:
        Leg** legs;
        Leg* tp_L[3]; // points to leg pointers
        Leg* tp_R[3];
        float s; // linear advance
        int target_steps, direction, total_steps=0; 
        // position definitions
        J_Position home = J_Position();
        J_Position storage = J_Position();
        J_Position stance = J_Position();
        J_Position lift = J_Position();
        J_Position swing = J_Position();
        J_Position plant = J_Position();
        C_Position liftC = C_Position(LIFT_X, LIFT_Y, LIFT_Z);
        C_Position swingC = C_Position(SWING_X, SWING_Y, SWING_Z);
        C_Position pushC = C_Position(PUSH_X, PUSH_Y, PUSH_Z); // relative to body, the push foot moves backward, therefore, push foot moved backward in local frame (ON GROUND)
        C_Position plantC = C_Position(SWING_X, SWING_Y, SWING_Z);
        bool halfTripod(Leg** l, Leg** p);
        bool liftLeg(Leg** trip, C_Position l);
        bool swingLeg(Leg** trip, C_Position s);
        bool plantLeg(Leg** trip, C_Position p);
        bool pushLeg(Leg** trip, C_Position p);
        long unsigned int move_time=0, prev_move=0;
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