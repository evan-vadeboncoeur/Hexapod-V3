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
    // R1 = 7.8k ohms, R2 = 10k ohms, V_full_charge = 8.4V, V_nom = 7.6V
    // V_full_charge computes to V_fc_in = 4.7V (padding for analog input channels)
    v_in = ((float) adc_battery)/full_scale * v_ref;
    v_battery = v_in * (float)(float(R1 + R2) / R2);
    #ifdef BRD_DEBUG
    Serial.println("-----Battery Readout-----");
    Serial.print("adc");
    Serial.print('\t');
    Serial.print("v_adc");
    Serial.print('\t');
    Serial.println("v_battery");
    Serial.print(adc_battery);
    Serial.print('\t');
    Serial.print(v_in);
    Serial.print('\t');
    Serial.println(v_battery);
    #endif
    lowBattery();
}

void BoardManager::lowBattery(){
    if(v_battery <= low_battery) digitalWrite(LOW_BATT_P, HIGH);
    else digitalWrite(LOW_BATT_P, LOW);
}

void BoardManager::checkBatteries(){
    readBattery();
}

void BoardManager::receiveBlink(){
    digitalWrite(NRFR_P, HIGH);
    delay(200);
    digitalWrite(NRFR_P, LOW);
}