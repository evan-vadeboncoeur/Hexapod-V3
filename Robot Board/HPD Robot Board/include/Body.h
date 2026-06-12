#ifndef BODY_H
#define BODY_H

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

class Body{
    public:
        Body();
        Body(Leg l[NUM_LEGS]);
    private:
        // body members
        Vector twist;
        Vector body_velocity;
        float vx, vy, wt, bv, theta; // bv is magnitude of body velocity, theta is direction of velocity
        float duty_factor, t_cycle; 
        // leg members
        Leg legs[NUM_LEGS];
        Vector foot_p_R[NUM_LEGS]; // foot positions in body frame
        Vector foot_v_R[NUM_LEGS]; // foot velocities in the body frame
        Vector foot_dp_R[NUM_LEGS]; // foot displacement vectors in the body frame
        Vector foot_p_L[NUM_LEGS]; // new foot positions in the leg frame 
        float alpha_i[NUM_LEGS]; // alpha offset for each leg, calculated at body instantiation
        float d_i = 75.0; // constant for every leg, travel distance along x_i
        void computeAlphaI();
        void unpackTwist();
        void R_TF_L(float alph);
        
};

#endif