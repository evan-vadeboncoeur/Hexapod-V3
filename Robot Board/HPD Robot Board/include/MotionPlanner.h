#ifndef MTN_PLNR_H
#define MTN_PLNR_H
#define PLAN_DEBUG
#define PLAN_T_DEBUG

#include "Twist.h"
#include "Vector.h"
#include "Body.h"
#include <math.h>

#define FWD (1.0)
#define RVRS (-1.0)
#define TP_1 (1)
#define TP_2 (2)

#define MOTION_PLANNER_DELAY (50)

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements
class MotionPlanner{
    private:
        Leg** legs; // 6 legs
        Leg* tp_L[3]; // 3 legs/tripod
        Leg* tp_R[3];
        Body b;
        Twist t; // twist command (v_x, v_y, w_z)
        float vx, vy, wz;
        float t_cyle = 1.5, duty_factor = 0.5, step_h = 12.0; // gait cycle hardcoded values

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
        bool powerOnSequence();
        bool powerOffSequence();
        bool moveHome(); // legs @ 0 pos (0, 0, 0)
        bool moveStorage(); // folded up position, power down
        bool moveIdle(); // wait for gait (setup pose)
        // v2.0 kinematics
        C_Position Body_TF_Leg(Leg* l, C_Position target); // TF body coordinates to leg coordinates
        Vector Foot_TF_Body(Leg* l, C_Position target); // TF foot coordinate to body coordinates
        C_Position Walking_TF_Leg(Leg* l, C_Position target); // TF walking coordinates to leg coordinates
        float computeWalkingAlpha(int id);
        // v2.1 kinematics
        void setTwist(Twist tw);
        void unpackTwist();
        void Body_r_Foot(); // calculate the postion vector of the feet in the body frame (RBTF)
        void Body_V_Foot(); // compute the foot velocities in the body frame
        void Foot_V_Foot(); // compute the foot velocities in the individual foot frame (use ) 

};

#endif