#ifndef BRD_H_H
#define BRD_H_H

#define LED_ON (500)
#define LED_OFF (0)

#define BATT_IN_P (A5) // does indeed work, is not "boot-strapped"
#define LOW_BATT_P (A7)
#define NRFR_P (A8)

#define R1 (7800)
#define R2 (10000)

#include "Arduino.h"


// class to read battery of robot and send information to LEDs
// add LED to board for NRF connection
// measures from 0-4.7V (padding to not damage ADC)

class BoardManager{
    private:
        int adc_battery;   
        float v_battery, v_in;
        float full_scale = 1023.0, v_ref = 5.0, clamp = 4.7, max_battery = 8.4, low_battery = 6.2;
        float v_out;
        void readBattery();
        void lowBattery();
    public:
        BoardManager();
        void checkBatteries();
        


};

#endif