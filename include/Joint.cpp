#include "Joint.h"

Joint::Joint(){
    
}

Joint::Joint(int id){
    servo.attach(id);
    servo_id = id;
}

