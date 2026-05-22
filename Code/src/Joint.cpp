#include "Joint.h"

Joint::Joint(){
    
}

Joint::Joint(int id){
    servo.attach(id);
    servo_id = id;
}

void Joint::setAngle(float ang){
    servo.write(ang);
}

