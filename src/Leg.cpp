#include "Leg.h"

//default empty constructor
Leg::Leg(){

}

// leg constructor. takes 3 servo pin IDs and attaches them, sets leg ID number (physical)
Leg::Leg(int j1, int j2, int j3, int id){
    joints[0] = Joint(j1);
    joints[1] = Joint(j2);
    joints[3] = Joint(j3, PWM_MIN, PWM_MAX); // clamp J2 from 0-270
    this->id = id;
}

void Leg::setTarget(C_Position goal){
    target_p.setPosition(goal.getX(), goal.getY(), goal.getZ());
    this->kinematic.setTarget(target_p.getX(), target_p.getY(), target_p.getZ());
}

// outsource to kinematic class to compute kinematics
void Leg::forwardKinematics(){
    kinematic.fk();
}
void Leg::inverseKinematics(bool config){
    target_j = kinematic.ik(config); // set joint space vector to inverse kinematics
}

// after computing IK with target joint vector, move each servo joint
void Leg::moveToJV(){
    joints[FEMUR].setAngle(target_j.getT2());
    delay(100);
    joints[FOOT].setAngle(target_j.getT3());
    delay(100);
    joints[COXA].setAngle(target_j.getT1()); // move hip last
    delay(100);
}

// adjust raw FK angle value to work with servo effort direction/offset
void Leg::adjustServos(){
    target_j.setT1(target_j.getT1() + COXA_SERVO_OFFSET);
    target_j.setT2(target_j.getT2() + FEMUR_SERVO_OFFSET);
    target_j.setT3(map(FOOT_SERVO_MAX - (target_j.getT3() + FOOT_SERVO_OFFSET), FOOT_SERVO_MIN, FOOT_SERVO_MAX_INT, PWM_MIN, PWM_MAX));
}

void Leg::moveToIK(C_Position tp, bool config){
    setTarget(tp); // set target position
    inverseKinematics(config); // calculate IK to do so
    adjustServos(); // adjust joint vector angles to match servo offsets
    //forwardKinematics();
    #ifndef IK_DEBUG
    moveToJV(); // move to position
    #endif
}