#ifndef MTN_PLNR_H
#define MTN_PLNR_H
#define PLAN_DEBUG
#define PLAN_T_DEBUG

#include "Twist.h"
#include "Vector.h"
#include "Body.h"
#include <math.h>

#define NWALK (false)
#define WALK (true)

#define FWD (1.0)
#define RVRS (-1.0)
#define TP_1 (1)
#define TP_2 (2)

#define MOTION_PLANNER_DELAY (50)
#define HALF_TRIPOD_DELAY (300)

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements
class MotionPlanner{
    private:
        Leg** legs; // 6 legs
        Leg* tp_L[3]; // 3 legs/tripod
        Leg* tp_R[3];
        Body b;
        Twist t; // twist command (v_x, v_y, w_z)
        float vx, vy, wz;
        float t_cycle, duty_factor, step_h, cycle_count; // gait cycle (hardcoded?) values
        bool walk_flag = NWALK;
        bool halfTripod(Leg** sw, Leg** st);
        bool push(Leg** l_st, Leg** l_sw);
        bool lift(Leg** l_l);
        long unsigned int half_c=0, half_c_prev=0, full_c=0, full_c_prev=0;
    public:
        enum Gait {TRIPOD, RIPPLE, WAVE, QUADRUPED} gait=TRIPOD;
        MotionPlanner();
        MotionPlanner(int g, float df, float tc, float sh);\
        void movement(int g, Vector tw);
        void setBodyVelocity(Vector tw);
        // Setters
        void setGait(int, bool);
        void setDutyFactor(int df){duty_factor = df;}
        void setCycleTime(int tc){t_cycle = tc;}
        void setStepHeight(int sh){step_h = sh;}
        // Gaits
        bool tripodGait();
        bool tripodGait(int cc);
        void setupTripod(int, int, int);
        void setDistanceIncrement(int);
        void sortTripod();
        bool waveGait();
        bool rippleGait();
        bool quadrapedGait();
        // Motion macros
        bool powerOnSequence();
        bool powerOffSequence();
        bool moveHome(); // legs @ 0 pos (0, 0, 0)
        bool moveStorage(); // folded up position, power down
        bool moveIdle(); // wait for gait (setup pose)
        // v2.1 kinematics
        void setTwist(Twist tw);
        void unpackTwist();
};

#endif