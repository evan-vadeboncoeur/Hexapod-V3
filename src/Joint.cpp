#include "Joint.h"

Joint::Joint(){
    
}



Joint::Joint(int id){
    servo.attach(id);
    servo_id = id;
}

Joint::Joint(int id, int pwm_min, int pwm_max){
    servo.attach(id, pwm_min, pwm_max);
    servo_id = id;
}

void Joint::setAngle(float ang){
    servo.write(ang);
}

