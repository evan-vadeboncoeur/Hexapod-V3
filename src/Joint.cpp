#include "Joint.h"

Joint::Joint(){
    
}



Joint::Joint(int id){
    servo.attach(id);
    servo_id = id;
    Serial.print("Attached servo to pin ");
    Serial.println(id);
}

Joint::Joint(int id, int pwm_min, int pwm_max){
    servo.attach(id, pwm_min, pwm_max);
    servo_id = id;
    Serial.print("Attached servo to pin ");
    Serial.println(id);
}

void Joint::setAngle(int ang){
    Serial.print("Servo pin ");
    Serial.print(servo_id);
    Serial.print(" -> ");
    Serial.println(ang);
    servo.write(ang);
}

