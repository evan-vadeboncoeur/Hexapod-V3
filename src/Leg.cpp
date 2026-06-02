#include "Leg.h"

//default empty constructor
Leg::Leg(){

}

// leg constructor. takes 3 servo pin IDs and attaches them, sets leg ID number (physical)
Leg::Leg(int j1, int j2, int j3, int id) : joints{ // initializer lists directly creates the class members before the constructor body
        Joint(j1), // bypasses creating temporary objects to assign into the joint[] array
        Joint(j2),
        Joint(j3, PWM_MIN, PWM_MAX)
    },
    id(id)
    // joints[COXA] = Joint(j1);
    // joints[FEMUR] = Joint(j2);
    // joints[FOOT] = Joint(j3, PWM_MIN, PWM_MAX); // clamp J2 from 0-270
    // this->id = id;
{
    //Serial.println(id);
}

void Leg::setDirection(float d){
    direction = d;
}

void Leg::setTarget(C_Position goal){
    float y_adjust = goal.getY();
    if(direction < 0) y_adjust = -y_adjust;
    target_p.setPosition(goal.getX(), y_adjust, goal.getZ());
    this->kinematic.setTarget(target_p.getX(), target_p.getY(), target_p.getZ());
}

// outsource to kinematic class to compute kinematics
void Leg::forwardKinematics(float side){
    kinematic.fk(side);
}
void Leg::inverseKinematics(bool config, float side){
    target_j = kinematic.ik(config, side); // set joint space vector to inverse kinematics
}

// after computing IK with target joint vector, move each servo joint
void Leg::moveToJV(){
    adjustServos();
    moveTo();
}

void  Leg::moveToJV(J_Position jv){ // overloaded
    target_j = jv; // set target vector to desired
    adjustServos();
    moveTo();
}

// adjust raw FK angle value to work with servo effort direction/offset
void Leg::adjustServos(){
    #ifdef IK_DEBUG
    Serial.println("Moving to (rad): ");
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
    float _t1, _t2, _t3;
    _t1 = target_j.getT1()*RAD_TO_DEG + COXA_SERVO_OFFSET;
    _t2 = target_j.getT2()*RAD_TO_DEG + FEMUR_SERVO_OFFSET;
    _t3 = map(FOOT_SERVO_MAX - (target_j.getT3()*RAD_TO_DEG + FOOT_SERVO_OFFSET), FOOT_SERVO_MIN, FOOT_SERVO_MAX, PWM_MIN, PWM_MAX);
    target_j.setT1(_t1);
    target_j.setT2(_t2);
    target_j.setT3(_t3);
    #ifdef IK_DEBUG
    Serial.println("Moving to (servo angle): ");
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
}

void Leg::moveToIK(C_Position tp, bool config, float side){
    setTarget(tp); // set target position
    inverseKinematics(config, side); // calculate IK to do so
    moveToJV(); // adjust servos and move to joint vector
}

// move servos after all other commands
void Leg::moveTo(){
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
    delay(LEG_DELAY);
    joints[FOOT].setAngle((int)target_j.getT3());
    delay(LEG_DELAY);
    joints[COXA].setAngle((int)target_j.getT1()); // move hip last
    delay(LEG_DELAY);
}

float Leg::getDirection(){
    return direction;
}

int Leg::getID(){
    return id;
}