#ifndef LEG_H
#define LEG_H

#include "Joint.h"
#include "C_Position.h"
#include "J_Position.h"
#include <math.h>
#include "Arduino.h"


// leg class. maintains leg position in joint and cartesian spaces. calculates forward and inverse kinematics for a singular leg, moves leg to desired JV/PV

class Leg{
    private:
        char id; // leg ID
        Joint joints[3]; // 3 joints per leg: J0, J1, J2
        C_Position local, global, target; // keep track of local, global, and target (local) positions
        J_Position joint_space; // joint vector of 3 servos at present
        void forwardKinematics();
        void inverseKinematics();
        void moveToPV(); // move to position vector
        void moveToJV(); // move to joint vector
    public:
        Leg();
        Leg(int j1, int j2, int j3, int id);
        void computeGlobal(); // conver local position to global
        

};

#endif