#ifndef PWR_MNG_H
#define PWR_MNG_H

#define LED_ON (500)
#define LED_OFF (0)

#include "Arduino.h"


// class to read battery of robot and send information to LEDs
// add LED to board for NRF connection
// measures from 0-4.7V (padding to not damage ADC)

class PowerManager{
    private:
        int in_pin, out_pin;    
        float battery;
        float resolution = 1023.0, ref = 5.0, clamp = 4.7, max_battery = 8.4;
        float r1 = 7800.0, r2 = 10000.0;
        float v_out;
        bool low_battery=false;
        float readBattery();
        void processBattery();
    public:
        PowerManager();
        PowerManager(int, int);


};

#endif