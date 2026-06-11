#ifndef LEG_KINEMATIC_H
#define LEG_KINEMATIC_H
//#define IK_DEBUG
//#define FK_DEBUG

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

// Kinematic class to separate leg functions cleanly

class LegKinematics {
    public:
        LegKinematics();
        LegKinematics(float, float, float);
        void setTargetPV(float, float, float);
        void setTargetJV(float, float, float);
        C_Position target_p = C_Position();
        C_Position calculated_p = C_Position();
        C_Position fk(float config);
        J_Position ik(bool config, float side);
        
    private:
        float beta1, beta2, gamma, r, t1, t2, t2_1, t2_2, t3, t3_1, t3_2, t3_s, p_x, p_y, p_z, L1, L2, L3, L4, L4_x, cb, sb, st3, ct3; // IK Variables
        float d1, d2, d3, d4, a1, a2, a3, a4, ap1, ap2, ap3, ap4, tht1, tht2, tht3, tht4, c_x, c_y, c_z, R1_1, R1_2, R1_3, R2_1, R2_2, R2_3, R3_1, R3_2, R3_3; // FK Variables
        float threshold = 15.0; // 15.0 mm threshold for ik
        float normalizeAngles(float);
        J_Position configCheck(bool config, float side);
        bool ikCheck(float);
        float distance(C_Position desired, C_Position calculated);
        // /HardwareSerial* serial;

};


#endif