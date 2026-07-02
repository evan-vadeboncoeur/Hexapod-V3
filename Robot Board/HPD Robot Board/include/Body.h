#ifndef BODY_H
#define BODY_H

#define SETUP_BK_DEBUG
#define VR_BK_DEBUG
#define SLS_DEBUG
//#define STANCE_BK_DEBUG

#include "Leg.h"

#define NUM_LEGS (6)
#define LEG_0 (0)
#define LEG_1 (1)
#define LEG_2 (2)
#define LEG_3 (3)
#define LEG_4 (4)
#define LEG_5 (5)
#define LEG_0_J0 (28)
#define LEG_0_J1 (29)
#define LEG_0_J2 (30)
#define LEG_1_J0 (22)
#define LEG_1_J1 (23)
#define LEG_1_J2 (24)
#define LEG_2_J0 (14)
#define LEG_2_J1 (15)
#define LEG_2_J2 (16)
#define LEG_3_J0 (25)
#define LEG_3_J1 (26)
#define LEG_3_J2 (27)
#define LEG_4_J0 (31)
#define LEG_4_J1 (32)
#define LEG_4_J2 (33)
#define LEG_5_J0 (34)
#define LEG_5_J1 (35)
#define LEG_5_J2 (36)

#define STEP_HEIGHT (12.0)
#define DUTY_FACTOR (0.5)
#define T_CYCLE (1.5)

// knows current and previous positions, change in positions, of legs. 
// knows body velocity and angular velocity
// receives velocity info and updates positions based on that
// intermediate layer of MotionPlanner: command -> MotionPlanner Unpacks -> prompts body to compute values -> MotionPlanner then walks using body / legs...?

class Body{
    public:
        Body();
        Body(float df, float t_c, float sh);
        void velocityCommand(Vector tw);
    private:
        // body members
        Vector twist;
        Vector body_velocity;
        float vx, vy, wz, bv, theta; // bv is magnitude of body velocity, theta is direction of velocity
        float duty_factor, t_cycle, t_stance; 
        float chassis_radius = 92.5; // radius of circumscribed leg circle
        float z_g; // ground "height" in body frame (where feet contact ground in body frame)
        float step_height; // step height parameter
        // leg members
        Leg legs[NUM_LEGS];
        Vector foot_idle_R[NUM_LEGS]; // idle foot positions in body frame
        Vector foot_swing_R[NUM_LEGS]; // swing foot positions in body frame
        Vector foot_stance_R[NUM_LEGS]; // stance foot positions in body frame
        Vector foot_lift_R[NUM_LEGS]; // lift (idle + z offset using linear interpolation)
        Vector foot_p_R[NUM_LEGS]; // foot positions in body frame
        Vector foot_v_R[NUM_LEGS]; // foot velocities in the body frame
        Vector foot_dp_R[NUM_LEGS]; // foot displacement vectors in the body frame
        Vector foot_pN_R[NUM_LEGS]; // new foot position in body frame
        Vector foot_p_L[NUM_LEGS]; // new foot positions in the leg frame 
        float alpha_i[NUM_LEGS]; // alpha offset for each leg, calculated at body instantiation
        float alpha_ci[NUM_LEGS], alpha_si[NUM_LEGS]; // cos, sin x and y components computed at startup for each leg
        void computeAlphaI();
        void unpackTwist(Vector tw);
        void compute_vR();
        void compute_dP();
        void compute_SLS();
        void compute_pN_L();
        Vector B_TF_L(Vector bc, int id);
        Vector L_TF_B(Vector v);
        
};

#endif