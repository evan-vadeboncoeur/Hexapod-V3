#ifndef LEG_H
#define LEG_H

#define COXA (0)
#define FEMUR (1)
#define FOOT (2)
#define COXA_SERVO_OFFSET (90.0)
#define FEMUR_SERVO_OFFSET (90.0)
#define FOOT_SERVO_OFFSET (135.0)
#define FOOT_SERVO_MAX (270.0)
#define FOOT_SERVO_MAX_INT (270)
#define FOOT_SERVO_MIN (0.0)
#define LEG_DELAY (1000)

#include "Joint.h"
#include "C_Position.h"
#include "J_Position.h"
#include "Arduino.h"
#include "Kinematics.h"


// leg class. maintains leg position in joint and cartesian spaces. calculates forward and inverse kinematics for a singular leg, moves leg to desired JV/PV

class Leg{
    private:
        Joint joints[3]; // 3 joints per leg: J0, J1, J2
        int id; // leg ID
        float direction; // +/- 1.0
        float L1 = 70, L2 = 100, L3 = 150; // link lengths
        C_Position local_p = C_Position(); 
        C_Position global_p = C_Position();
        C_Position target_p = C_Position(); // keep track of local, global, and target (local) positions
        J_Position local_j = J_Position();
        J_Position target_j = J_Position(); // joint vector of 3 servos at present
        Kinematics kinematic = Kinematics(L1, L2, L3);
        void forwardKinematics(float);
        void inverseKinematics(bool, float);
        void moveToPV(); // move to position vector
        void moveTo(); // move the servos
        void adjustServos(); // adjust angles to servo values
    public:
        Leg();
        Leg(int j1, int j2, int j3, int id);
        void computeGlobal(); // conver local position to global
        void setTarget(C_Position goal);
        void moveToIK(C_Position tp, bool config, float side);
        void moveToJV(); // move to joint vector
        void moveToJV(J_Position jv); // overloaded
        void setDirection(float);
        float getDirection();
        int getID();
        

};

#endif