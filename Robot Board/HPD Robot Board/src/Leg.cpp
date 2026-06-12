#include "Leg.h"

//default empty constructor
Leg::Leg(){

}

// leg constructor. takes 3 servo pin IDs and attaches them, sets leg ID number (physical), assigns CCW or CW configuration
Leg::Leg(int id, int j1, int j2, int j3, float configuration) : joints{ // initializer lists directly creates the class members before the constructor body
        Joint(j1, PWM_MIN, PWM_MAX), // bypasses creating temporary objects to assign into the joint[] array
        Joint(j2, PWM_MIN, PWM_MAX),
        Joint(j3, PWM_MIN, PWM_MAX)
    },
    id(id),
    configuration(configuration),
    lk(L1, L2, L3, configuration)
{
    #ifdef LEG_DEBUG
    Serial.println("Leg ID & Configuration:");
    Serial.print(id);
    Serial.print('\t');
    Serial.println(configuration);
    #endif
    // set current foot positions
    setTargetFootP(storage_p_L);
    setTargetFootJ(storage_j_L);
}

// takes a position vector in the leg's frame and calculates the joint vector needed to satisfy
void Leg::moveFootToPV(Vector new_pVL, bool config){
    // calculate workspace first..., change to bool for if we can move to this value
    // set previous foot values to store if needed in body class
    setPrevFootP(foot_p_L);
    setPrevFootJ(foot_j_L);
    // update current values to set values
    setTargetFootP(new_pVL); // moving to this point in cartesian space
    setTargetFootJ(lk.ik(new_pVL, config)); // calculate IK, update target movement joint vector
    // use JV target to compute servo vectors
    setServoJV();
    // move to target
    moveTo();
}

void Leg::moveFootToJV(Vector new_jVL){
    // set previous foot values to store if needed in body class
    setPrevFootP(foot_p_L);
    setPrevFootJ(foot_j_L);
    // set target values using FK
    setTargetFootJ(new_jVL);
    setTargetFootP(lk.fk(foot_j_L)); // calculate ik to obtain respective cartesian vector of foot we are moving to
    // Use JV target to compute servos
    setServoJV();
    // move to target
    moveTo();
}

void Leg::setServoJV(){
    servo_j_L = foot_j_L; // copy into servo position object
    radToDeg(); // convert joint control vector to degrees
    servoOffsets(); // servo offset helper
    
}

// convert joint vector to degrees
void Leg::radToDeg(){
    servo_j_L.setX1(servo_j_L.getX1()*RAD_TO_DEG);
    servo_j_L.setX2(servo_j_L.getX2()*RAD_TO_DEG);
    servo_j_L.setX3(servo_j_L.getX3()*RAD_TO_DEG);
    #ifdef LEG_DEBUG
    Serial.println("Servo angles [deg]: ");
    Serial.print("T1: ");
    Serial.print('\t');
    Serial.print(servo_j_L.getX1());
    Serial.print('\t');
    Serial.print("T2: ");
    Serial.print('\t');
    Serial.print(servo_j_L.getX2());
    Serial.print('\t');
    Serial.print("T3: ");
    Serial.print('\t');
    Serial.println(servo_j_L.getX3());
    #endif
}

void Leg::servoOffsets(){
    servo_j_L.setX1(servo_j_L.getX1() + COXA_SERVO_OFFSET);
    servo_j_L.setX2(servo_j_L.getX2() + FEMUR_SERVO_OFFSET);
    servo_j_L.setX3(map((servo_j_L.getX3() + FOOT_SERVO_OFFSET), FOOT_SERVO_MIN, FOOT_SERVO_MAX, PWM_MIN, PWM_MAX));
    #ifdef LEG_DEBUG
    Serial.println("Servo command [deg or us]: ");
    Serial.print("T1: ");
    Serial.print('\t');
    Serial.print(servo_j_L.getX1());
    Serial.print('\t');
    Serial.print("T2: ");
    Serial.print('\t');
    Serial.print(servo_j_L.getX2());
    Serial.print('\t');
    Serial.print("T3: ");
    Serial.print('\t');
    Serial.println(servo_j_L.getX3());
    #endif
}

// move to already calculated servo vector
void Leg::moveTo(){
    joints[FOOT].setAngle(servo_j_L.getX3());
    delay(JOINT_DELAY);
    joints[FEMUR].setAngle(servo_j_L.getX2());
    delay(JOINT_DELAY);
    joints[COXA].setAngle(servo_j_L.getX1());
    delay(JOINT_DELAY);
}




