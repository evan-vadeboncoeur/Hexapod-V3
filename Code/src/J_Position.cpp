#include "J_Position.h"

J_Position::J_Position(){
    
}


J_Position::J_Position(float t1, float t2, float t3){
    this->t1 = t1;
    this->t2 = t2;
    this->t3 = t3;
}

float J_Position::getT1(){
    return t1;
}

float J_Position::getT2(){
    return t2;
}

float J_Position::getT3(){
    return t3;
}