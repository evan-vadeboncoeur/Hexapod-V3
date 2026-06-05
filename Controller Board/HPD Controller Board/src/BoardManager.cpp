#include "BoardManager.h"

BoardManager::BoardManager(){

}

void readInputs(){ // reads all inputs, updates members

}

// readers
void BoardManager::readLeftJS(){
    lx = analogRead(LX_P);
    ly = analogRead(LY_P);
    lb = digitalRead(LB_P);
}
void BoardManager::readRightJS(){
    rx = analogRead(RX_P);
    ry = analogRead(RY_P);
    rb = digitalRead(RB_P);
}

void BoardManager::readBattery(){
    batt = analogRead(BATT_IN_P);
    // v_divider as needed
    // ...
    batt = batt/v_ref*full_scale;
}

void BoardManager::lowBattery(){
    if(batt <= low_batt) digitalWrite(LOW_BATT_P, HIGH);
}

void BoardManager::nrfConnected(){

}

// getters
int BoardManager::getLx(){
    return lx;
}
int BoardManager::getLy(){
    return ly;
}
bool BoardManager::getLb(){
    return lb;
}
int BoardManager::getRx(){
    return rx;
}
int BoardManager::getRy(){
    return ry;
}
bool BoardManager::getRb(){
    return rb;
}
char BoardManager::getGait(){
    return g;
}