#ifndef LEG_H
#define LEG_H
#define LEG_DEBUG

#define M_PI_3 (1.04719755)
#define M_PI_6 (0.523598776)

//#define CCW_CONFIG (-1)
#define CW_CONFIG (1)

#define COXA (0)
#define FEMUR (1)
#define FOOT (2)
#define COXA_SERVO_OFFSET (90.0)
#define FEMUR_SERVO_OFFSET (90.0)
#define FOOT_SERVO_OFFSET (135.0)
#define FOOT_SERVO_MAX (270.0)
#define FOOT_SERVO_MAX_INT (270)
#define FOOT_SERVO_MIN (0.0)
#define LEG_DELAY (10)

#include "Joint.h"
#include "C_Position.h"
#include "J_Position.h"
#include "Arduino.h"
#include "LegKinematics.h"


// leg class. maintains leg position in joint and cartesian spaces. calculates forward and inverse kinematics for a singular leg, moves leg to desired JV/PV

class Leg{
    private:
        Joint joints[3]; // 3 joints per leg: J0, J1, J2
        int id; // leg ID
        float configuration; // CCW facing or CW facing
        float L0 = 77.5, L1 = 70.0, L2 = 100.0, L3 = 150.0; // link lengths
        // cartesian space values
        Vector storage_j_L = Vector(0.0, -M_PI_3, -2.0); // initial storage position in the joint space
        
        Vector foot_p_L; // target foot position in the leg frame (J0 = base)
        Vector prevFoot_p_L; // prev foot position in the leg frame
        Vector foot_speed_L; // foot speed in the leg (J0) frame
        // joint space values
        Vector foot_j_L; // target foot position in the joint space
        Vector servo_j_L; // servo joint space vector for moving the leg
        Vector prevFoot_j_L; // prev foot position in the joint space
        LegKinematics lk = LegKinematics(); // calculation object
        // Helper functions
        void moveTo(); // move the servos
        void setServoJV(); // adjust angles to servo values
        void radToDeg(); // convert radiand to degrees
        void servoOffsets();
    public:
        // Constructors
        Leg();
        Leg(int id, int j1, int j2, int j3, float configuration);
        Vector storage_p_L = Vector(-29.33, 0.00, 1.78); // initial storage position in the leg frame
        // Getters
        Vector getPrevFootP(){return prevFoot_p_L;} // get position from previous move
        Vector getPrevFootJ(){return prevFoot_j_L;}
        Vector getTargetFootP(){return foot_p_L;}
        int getID(){return id;}
        // Setters
        void setTargetFootP(Vector new_pVL){prevFoot_p_L = foot_p_L, foot_p_L = new_pVL;} // set new, calculated goal position
        void setTargetFootJ(Vector new_jVL){foot_j_L = new_jVL;}
        void setPrevFootP(Vector prev_pVL){prevFoot_p_L = prev_pVL;}
        void setPrevFootJ(Vector prev_jVL){prevFoot_j_L = prev_jVL;}
        // Movement function macros
        void moveFootToPV(Vector new_pVL, bool config);
        void moveFootToJV(Vector new_jVL); // move to joint vector

        
 


};

#endif