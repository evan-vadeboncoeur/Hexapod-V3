#ifndef MTN_PLNR_H
#define MTN_PLNR_H
#define PLAN_DEBUG
//#define PLAN_T_DEBUG
#define STATE_TIME_DEBUG

#include "Vector.h"
#include "Body.h"
#include <math.h>

#define NWALK (false)
#define WALK (true)

#define FWD (1.0)
#define RVRS (-1.0)
#define TP_1 (1)
#define TP_2 (2)
#define TEST_STEPS (10)

#define MOTION_PLANNER_DELAY (25)
#define HALF_TRIPOD_DELAY (100)
#define MACRO_DELAY (500)

#define OMEGA (5.2383)
#define MVMT_BFR (0)
#define OMEGA_C (0.95)

// motion planner class that recieves a command (gait, direction, # steps...). Controls order of leg movements
class MotionPlanner{
    private:
        Leg** legs; // 6 legs
        Leg** tripods[2];
        Body b;
        Vector t; // twist command (v_x, v_y, w_z)
        Vector omega = Vector(0.0, 0.0, 0.25); // constanct ccw rotation vector 
        float vx, vy, wz;
        float t_cycle, duty_factor, step_h, cycle_count; // gait cycle (hardcoded?) values
        int t_c_m=0; // millis version of cycle time
        int wave_ct=0;
        bool walk_flag = NWALK, even_forward = false, idle = true;
        bool halfTripod(Leg** sw, Leg** st);
        bool push(Leg** l_st, Leg** l_sw);
        bool lift(Leg** l_l);
        uint32_t moveTime(float t1, float t2);
        void stateTime();
        uint32_t stateTimeHelper(Vector v1, Vector v2, uint32_t m);
        #ifdef PLAN_DEBUG
        uint64_t half_c=0, half_c_prev=0, full_c=0, full_c_prev=0;
        #endif
        // tripod gait state variables
        uint32_t stance_lift=0, lift_swing=0, swing_stance=0;
        uint64_t gait_update_time=0;
        enum TripodPhase {EVEN_STANCE, ODD_STANCE} tp_phase=EVEN_STANCE;
        enum TripodStance {LIFT, PUSH} tp_stance=LIFT;
        uint8_t tp_index=0;
    public:
        enum Gait {TRIPOD, WAVE, RIPPLE, QUADRUPED} gait=TRIPOD;
        MotionPlanner();
        MotionPlanner(uint8_t g, float df, float tc, float sh);
        void movementSetup(int g, Vector tw);
        void setBodyVelocity(Vector tw);
        // Setters
        void setGait(int, bool);
        void setDutyFactor(int df){duty_factor = df;}
        void setCycleTime(int tc){t_cycle = tc;}
        void setStepHeight(int sh){step_h = sh;}
        // Gaits
        void updateTripod();
        void updateWave();
        bool tripodGait();
        bool tripodGait(int cc);
        bool waveGait(int cc);
        bool rippleGait(int cc);
        bool wavePush(int wc);
        bool quadrapedGait();
        // Motion macros
        bool powerOnSequence();
        bool powerOffSequence();
        bool moveHome(); // legs @ 0 pos (0, 0, 0)
        bool moveStorage(); // folded up position, power down
        bool moveIdle(); // wait for gait (setup pose)
        // Hexapod class functions
        void walk();
        void turn();
};

#endif