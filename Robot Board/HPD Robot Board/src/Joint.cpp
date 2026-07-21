#include "Joint.h"

Joint::Joint(){
    
}

Joint::Joint(int id){
    servo.attach(id);
    servo_id = id;
    #ifdef JOINT_DEBUG
    Serial.print("Attached servo to pin ");
    Serial.println(id);
    #endif
}

Joint::Joint(int id, int pwm_min, int pwm_max){
    servo.attach(id, pwm_min, pwm_max);
    #ifdef JOINT_DEBUG
    servo_id = id;
    Serial.print("Attached servo to pin ");
    Serial.println(id);
    #endif
}

void Joint::setAngle(int ang){
    #ifdef JOINT_DEBUG
    Serial.print("Servo pin ");
    Serial.print(servo_id);
    Serial.print(" -> ");
    Serial.println(ang);
    #endif
    servo.write(ang);
}

