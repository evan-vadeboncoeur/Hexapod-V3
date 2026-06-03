#include "J_Position.h"

J_Position::J_Position(){
    
}


J_Position::J_Position(float t_1, float t_2, float t_3){
    t1 = t_1;
    t2 = t_2;
    t3 = t_3;
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

void J_Position::setT1(float t){
    t1 = t;
}

void J_Position::setT2(float t){
    t2 = t;
}

void J_Position::setT3(float t){
    t3 = t;
}