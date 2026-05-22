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
    target.setPosition(goal.getX(), goal.getY(), goal.getZ());
}

void Leg::forwardKinematics(){
    


}
