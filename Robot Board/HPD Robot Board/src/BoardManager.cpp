#include "BoardManager.h"

BoardManager::BoardManager(){
    // input setup
    // analog pin setup
    pinMode(BATT_IN_P, INPUT);
    // output setup
    pinMode(LOW_BATT_P, OUTPUT);
    pinMode(NRFR_P, OUTPUT);
}

// battery management
void BoardManager::readBattery(){
    adc_battery = analogRead(BATT_IN_P);
    // R1 = 5100 ohms, R2 = 10000 ohms, V_full_charge = 4.2V, V_nom = 3.7V
    // V_full_charge computes to V_fc_in = 3.2V (padding for analog input channels)
    v_in = (float)adc_battery/full_scale * v_ref;
    v_battery = v_in * (float)((R1 + R2) / R1);
    lowBattery();
}

void BoardManager::lowBattery(){
    if(v_battery <= low_battery) digitalWrite(LOW_BATT_P, HIGH);
    else digitalWrite(LOW_BATT_P, LOW);
}