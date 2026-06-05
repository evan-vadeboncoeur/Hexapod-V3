#include "J_Position.h"
#include "Arduino.h"
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
    #ifdef J_POS_DEBUG
    Serial.print("Set T1: ");
    Serial.println(t1);
    #endif
}

void J_Position::setT2(float t){
    t2 = t;
    #ifdef J_POS_DEBUG
    Serial.print("Set T2: ");
    Serial.println(t2);
    #endif
}

void J_Position::setT3(float t){
    t3 = t;
    #ifdef J_POS_DEBUG
    Serial.print("Set T3: ");
    Serial.println(t3);
    #endif
}