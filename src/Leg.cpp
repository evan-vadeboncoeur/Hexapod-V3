#include "Leg.h"

//default empty constructor
Leg::Leg(){

}

// leg constructor. takes 3 servo pin IDs and attaches them, sets leg ID number (physical)
Leg::Leg(int j1, int j2, int j3, int id){
    joints[0] = Joint(j1);
    joints[1] = Joint(j2);
    joints[3] = Joint(j3);
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
    joints[1].setAngle(target_j.getT2());
    delay(100);
    joints[2].setAngle(target_j.getT3());
    delay(100);
    joints[0].setAngle(target_j.getT1());
    delay(100);

}

void Leg::moveToIK(C_Position tp, bool config){
    setTarget(tp); // set target position
    inverseKinematics(config); // calculate IK to do so
    moveToJV(); // move to position
    
}