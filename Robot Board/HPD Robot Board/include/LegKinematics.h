#ifndef LEG_KINEMATIC_H
#define LEG_KINEMATIC_H
//#define LEG_IK_DEBUG
//#define LEG_FK_DEBUG

#define CCW_CONFIG (-1.0) // invert atan2 angle to take CCW out of page, not into page (RHS)
#define LHS (1.0)
#define RHS (-1.0)
#define ELBOW_UP (false)
#define ELBOW_DOWN (true)

#include <math.h>
#include <HardwareSerial.h>
#include <Arduino.h>
#include "C_Position.h"
#include "J_Position.h"
#include "Vector.h"

// Kinematic class to separate leg functions cleanly

class LegKinematics {
    public:
        LegKinematics();
        LegKinematics(float, float, float, float); // configuration value (leg is CCW about Z = 1, CW about Z = -1)
        void setTargetPositionVector(Vector);
        void setTargetJointVector(Vector);
        Vector target_pV, calculated_jV; // cartesian space coordinate (IK) -> joint space
        Vector target_jV, calculated_pV; // joint space target (FK) -> cartesian space
        Vector fk(Vector); // compute FK given a leg, its configuration, target joint vector
        Vector ik(Vector, bool); // compute IK given a leg, its configuration, target position vector
        
    private:
        float configuration; // is leg coming off chassis in CCW or CW configuration (effects trig & signs)
        float beta1, beta2, gamma, r, t1, t2, t2_1, t2_2, t3, t3_1, t3_2, t3_s, p_x, p_y, p_z, L1, L2, L3, L4, L4_x, cb, sb, st3, ct3, r_c, r_c2; // IK Variables
        float d1, d2, d3, d4, a1, a2, a3, a4, ap1, ap2, ap3, ap4, tht1, tht2, tht3, tht4, c_x, c_y, c_z, R1_1, R1_2, R1_3, R2_1, R2_2, R2_3, R3_1, R3_2, R3_3; // FK Variables
        float threshold = 15.0; // 15.0 mm threshold for ik
        float normalizeAngles(float);
        Vector configurationHelper(bool);
        bool ikCheck();
        
        // /HardwareSerial* serial;

};


#endif