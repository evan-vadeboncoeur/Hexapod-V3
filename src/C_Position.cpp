#ifndef C_POS_H
#define C_POS_H

#include "C_Position.h"

C_Position::C_Position(float x_s, float y_s, float z_s){
    x = x_s;
    y = y_s;
    z = z_s;
}

void C_Position::setPosition(float x_s, float y_s, float z_s){
    x = x_s;
    y = y_s;
    z = z_s;
}

float C_Position::getX(){
    return x; 
}

float C_Position::getY(){
    return y; 
}

float C_Position::getZ(){
    return z; 
}

#endif