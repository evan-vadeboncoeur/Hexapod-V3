#ifndef MTN_PLNR_H
#define MTN_PLNR_H
#define PLAN_DEBUG
#define PLAN_T_DEBUG

#include "Twist.h"
#include "Vector.h"

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
        Leg** legs; // 6 legs
        Leg* tp_L[3]; // 3 legs/tripod
        Leg* tp_R[3];
        float s; // linear advance
        int target_steps, direction, total_steps=0; 
        
        // v2.1 kinematics ----
        Twist t; // twist command (v_x, v_y, w_z)
        float vx, vy, wz;
        float t_cyle = 1.5, duty_factor = 0.5; // gait cycle hardcoded values
        Vector v_R; // body velocity from input+
        Vector r0_FR, r1_FR, r2_FR, r3_FR, r4_FR, r5_FR; // current positions
        Vector v0_R, v1_R, v2_R, v3_R, v4_R, v5_R; // leg velocities in robot body frame {X_R, Y_R} (consider adding into leg class too?)
        Vector dP0_R, dP1_R, dP2_R, dP3_R, dP4_R, dP5_R; // displacement vectors in the robot frame
        Vector Pn0_R, Pn1_R, Pn2_R, Pn3_R, Pn4_R, Pn5_R; // new position vectors in the robot frame for each foot
        Vector r0_L0, r1_L1, r2_L2, r3_L3, r4_L4, r5_l5; // foot positions in local leg_i frame {X_L#, Y_L#} (adjust in leg class as well?)
        
        // leg/foot arrays to make calculating easier
        Vector R_r_foot[NUM_LEGS] = {r0_FR, r1_FR, r2_FR, r3_FR, r4_FR, r5_FR}; // foot positions in body frame
        Vector R_v_foot[NUM_LEGS] = {v0_R, v1_R, v2_R, v3_R, v4_R, v5_R}; // foot velocities in body frame
        Vector R_dP_foot[NUM_LEGS] = {dP0_R, dP1_R, dP2_R, dP3_R, dP4_R, dP5_R}; // position change vectors in body frame
        Vector R_Pn_foot[NUM_LEGS] = {Pn0_R, Pn1_R, Pn2_R, Pn3_R, Pn4_R, Pn5_R}; // new position vectors in body frame of each leg
        Vector L_Pn_foot[NUM_LEGS] = {r0_L0, r1_L1, r2_L2, r3_L3, r4_L4, r5_l5}; 
      
        // v2.1 end ----
        // position definitions
        J_Position home = J_Position(0.0, 0.0, 0.0); // rads
        J_Position storage = J_Position(0.0, -M_PI_3, -2.0);
        J_Position idle = J_Position(0, -0.69, 1.95);
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