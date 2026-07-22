#ifndef BODY_H
#define BODY_H

// #define SETUP_BK_DEBUG
// #define VR_BK_DEBUG
// #define SLS_DEBUG
// #define DP_BK_DEBUG

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

#define TP_EVEN (0)
#define TP_ODD (1)

#define CART_MOVE (true)
#define JOINT_MOVE (false)

#define STEP_HEIGHT (12.0)
#define DUTY_FACTOR (0.5)
#define T_CYCLE (1.5)

// knows current and previous positions, change in positions, of legs. 
// knows body velocity and angular velocity
// receives velocity info and updates positions based on that
// intermediate layer of MotionPlanner: command -> MotionPlanner Unpacks -> prompts body to compute values -> MotionPlanner then walks using body / legs...?

class Body{
    public:
        // Constructors
        Body();
        Body(float df, float t_c, float sh);
        // Motion Planner / Hexapod Accessible Functions
        void velocityCommand(Vector tw);
        void moveLeg(Leg* l, Vector v, bool type, bool elbow); // bool: joint or cartesian
        // Getters
        Leg* getLeg(int ln){return (&(legs[ln]));} // return pointer to desired leg
        Leg* getLegTripod(int ln, int tp); // return pointer to leg in specific tripod
        Leg** getTripod(int tp);
        Leg** getLegList(){return leg_list;}
        Vector getLift(int id){return foot_lift_R[id];}
        Vector getSwing(int id){return foot_swing_R[id];}
        Vector getStance(int id){return foot_stance_R[id];}
        Vector B_TF_L(Vector bc, int id);
        Vector computeIK(Leg* l, Vector v, bool elbow);
        bool moveTripod(Leg** tp, Vector jv[NUM_LEGS/2]);
        bool moveLegs(Vector jv[NUM_LEGS]);
        float getTheta(){return theta;}
        void setLiftHeight(float sh){lift_z = sh;}
        float getLiftHeight(){return lift_z;}
        Vector getLiftJ(int i){return foot_lift_J[i];}
        Vector getSwingJ(int i){return foot_swing_J[i];}
        Vector getStanceJ(int i){return foot_stance_J[i];}
    private:
        // body members
        Vector twist;
        Vector body_velocity;
        float vx, vy, wz, bv, theta; // bv is magnitude of body velocity, theta is direction of velocity
        float duty_factor, t_cycle, t_stance; 
        float chassis_radius = 92.5; // radius of circumscribed leg circle
        float z_g; // ground "height" in body frame (where feet contact ground in body frame)
        float step_height, lift_z; // step height parameter, step height in robot frame
        // leg members
        Leg legs[NUM_LEGS];
        Leg* tp_even[NUM_LEGS/2];
        Leg* tp_odd[NUM_LEGS/2];
        Leg* leg_list[NUM_LEGS];
        Vector foot_idle_R[NUM_LEGS]; // idle foot positions in body frame
        Vector foot_swing_R[NUM_LEGS]; // swing foot positions in body frame
        Vector foot_swing_J[NUM_LEGS]; // lift joint vectors
        Vector foot_stance_R[NUM_LEGS]; // stance foot positions in body frame
        Vector foot_stance_J[NUM_LEGS]; // lift joint vectors
        Vector foot_lift_R[NUM_LEGS]; // lift (idle + z offset using linear interpolation)
        Vector foot_lift_J[NUM_LEGS]; // lift joint vectors
        Vector foot_v_R[NUM_LEGS]; // foot velocities in the body frame
        Vector foot_dp_R[NUM_LEGS]; // foot displacement vectors in the body frame
        float alpha_i[NUM_LEGS]; // alpha offset for each leg, calculated at body instantiation
        float alpha_ci[NUM_LEGS], alpha_si[NUM_LEGS]; // cos, sin x and y components computed at startup for each leg
        void computeAlphaI();
        void unpackTwist(Vector tw);
        void compute_vR();
        void compute_dP();
        void compute_SLS();
        void L_TF_B(Vector v);
        void bodyInitialSetup(Vector v);
};

#endif