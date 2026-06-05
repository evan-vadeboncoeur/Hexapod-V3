#include "BoardManager.h"

BoardManager::BoardManager(int i_pin, int o_pin){
    in_pin = i_pin;
    out_pin = o_pin;
    pinMode(i_pin, INPUT);
    pinMode(o_pin, OUTPUT);
}

float BoardManager::readBattery(){
    float v2 = (analogRead(in_pin) / resolution) * ref; // voltage from 0-5V
    v_out = v2 *((r2 + r2)/r1); // convert back to 8.4V
    battery = (v_out / max_battery)*100;
    return v_out;
}

void BoardManager::processBattery(){
    readBattery();
    if(v_out >= clamp){
        analogWrite(out_pin, LED_ON);
        low_battery = true;
    } else{
        analogWrite(out_pin, LED_OFF);
        low_battery = false;
    }
}