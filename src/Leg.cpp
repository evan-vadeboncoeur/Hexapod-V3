#include "Leg.h"

//default empty constructor
Leg::Leg(){

}

// leg constructor. takes 3 servo pin IDs and attaches them, sets leg ID number (physical)
Leg::Leg(int j1, int j2, int j3, int id) : joints{
        Joint(COXA),
        Joint(FEMUR),
        Joint(FOOT, PWM_MIN, PWM_MAX)
    },
    id(id)
    // joints[COXA] = Joint(j1);
    // joints[FEMUR] = Joint(j2);
    // joints[FOOT] = Joint(j3, PWM_MIN, PWM_MAX); // clamp J2 from 0-270
    // this->id = id;
{
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
    #ifdef IK_DEBUG
    Serial.println("Moving to: ");
    Serial.print("T1: ");
    Serial.print('\t');
    Serial.print(target_j.getT1());
    Serial.print('\t');
    Serial.print("T2: ");
    Serial.print('\t');
    Serial.print(target_j.getT2());
    Serial.print('\t');
    Serial.print("T3: ");
    Serial.print('\t');
    Serial.println(target_j.getT3());
    #endif
    
    joints[FEMUR].setAngle((int)target_j.getT2());
    delay(1000);
    joints[FOOT].setAngle((int)target_j.getT3());
    delay(1000);
    
    joints[COXA].setAngle((int)target_j.getT1()); // move hip last
    delay(1000);
}

// adjust raw FK angle value to work with servo effort direction/offset
void Leg::adjustServos(){
    #ifdef IK_DEBUG
    Serial.println("Moving to: ");
    Serial.print("T1: ");
    Serial.print('\t');
    Serial.print(target_j.getT1());
    Serial.print('\t');
    Serial.print("T2: ");
    Serial.print('\t');
    Serial.print(target_j.getT2());
    Serial.print('\t');
    Serial.print("T3: ");
    Serial.print('\t');
    Serial.println(target_j.getT3());
    #endif

    target_j.setT1(target_j.getT1()*RAD_TO_DEG+ COXA_SERVO_OFFSET);
    target_j.setT2(target_j.getT2()*RAD_TO_DEG + FEMUR_SERVO_OFFSET);
    target_j.setT3(map(FOOT_SERVO_MAX - (target_j.getT3()*RAD_TO_DEG + FOOT_SERVO_OFFSET), FOOT_SERVO_MIN, FOOT_SERVO_MAX_INT, PWM_MIN, PWM_MAX));
}

void Leg::moveToIK(C_Position tp, bool config){
    setTarget(tp); // set target position
    inverseKinematics(config); // calculate IK to do so
    adjustServos(); // adjust joint vector angles to match servo offsets
    //forwardKinematics();
    //#ifndef IK_DEBUG
    moveToJV(); // move to position
    //#endif
}