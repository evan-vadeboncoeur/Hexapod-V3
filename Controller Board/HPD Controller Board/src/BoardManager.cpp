#include "BoardManager.h"

// setup pins for the board manager
BoardManager::BoardManager(){
    // pull-up pins for JS button
    pinMode(LB_P, INPUT_PULLUP);
    pinMode(RB_P, INPUT_PULLUP);
    // pulled-down on board for dipswitch array
    // pinMode(GP_1, INPUT);
    // pinMode(GP_1, INPUT);
    // pinMode(GP_3, INPUT);
    // pinMode(GP_4, INPUT);
    pinMode(GP_1, INPUT_PULLDOWN);
    pinMode(GP_1, INPUT_PULLDOWN);
    pinMode(GP_3, INPUT_PULLDOWN);
    pinMode(GP_4, INPUT_PULLDOWN);
    // analog pin setup
    analogReadResolution(12); // 12-bit is native to esp32
    analogSetAttenuation(ADC_11db); // set attenuation to 150mV - 3100 mV for all ADC pins
    pinMode(LX_P, INPUT);
    pinMode(LY_P, INPUT);
    pinMode(RX_P, INPUT);
    pinMode(RY_P, INPUT);
    pinMode(BATT_IN_P, INPUT);
    // output setup
    pinMode(LOW_BATT_P, OUTPUT);
    pinMode(NRFR_P, OUTPUT);
}

void BoardManager::readInputs(){ // reads all inputs, updates members
    readLeftJS();
    readRightJS();
    readBattery();
    readDS();
}

// readers
void BoardManager::readLeftJS(){
    lx = analogRead(LX_P); // all "coordinate" readings in regular XY frame of the controller
    ly = analogRead(LY_P);
    lb = digitalRead(LB_P);
    #ifdef BRD_R_DEBUG
    Serial.println("Left JS Debug: ");
    Serial.print("Lx - Horizontal: ");
    Serial.print('\t');
    Serial.print(lx);
    Serial.print('\t');
    Serial.print("Ly - Vertical: ");
    Serial.print('\t');
    Serial.print(ly);
    Serial.print('\t');
    Serial.print("Button: ");
    Serial.print('\t');
    Serial.print(lb);
    Serial.println();
    #endif
}
void BoardManager::readRightJS(){
    rx = analogRead(RX_P);
    ry = analogRead(RY_P);
    rb = digitalRead(RB_P);
    #ifdef BRD_R_DEBUG
    Serial.println("Right JS Debug: ");
    Serial.print("Rx - Horizontal: ");
    Serial.print('\t');
    Serial.print(rx);
    Serial.print('\t');
    Serial.print("Ry - Vertical: ");
    Serial.print('\t');
    Serial.print(ry);
    Serial.print('\t');
    Serial.print("Button: ");
    Serial.print('\t');
    Serial.print(rb);
    Serial.println();
    #endif
}

// battery management
void BoardManager::readBattery(){
    batt = analogRead(BATT_IN_P);
    // v_divider as needed
    // ...
    batt = batt/v_ref*full_scale;
    lowBattery();
}

void BoardManager::lowBattery(){
    if(batt <= low_batt) digitalWrite(LOW_BATT_P, HIGH);
}

void BoardManager::nrfConnected(){

}

// reads dipswitch values and stores in objects fields
void BoardManager::readDS(){
    char ga=0;
    // read input pins on DS array
    s_one = digitalRead(GP_1);
    s_two = digitalRead(GP_2);
    s_three = digitalRead(GP_3);
    s_four = digitalRead(GP_4);
    // xor bits together into char
    ga |= (s_one);
    ga |= (s_two << 1);
    ga |= (s_three << 2);
    ga |= (s_four << 3);
    // store in object member
    g = ga;
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