#ifndef BRD_R_H
#define BRD_R_H

#define LX_P (1) // fill in place holders after reading the schematic
#define LY_P (1)
#define LB_P (1)
#define RX_P (1)
#define RY_P (1)
#define RB_P (1)
#define GP_1 (1)
#define GP_2 (1)
#define GP_3 (1)
#define GP_4 (1)
#define BATT_IN_P (1)
#define LOW_BATT_P (1)
#define NRFR_P (1)

#include <Arduino.h>

// pin definitions here...

// class for managing the power of the board (low power LED, board on, analog read voltage), inputs, etc.
// copy for Hexapod reciever and adjust as neccesary. add conditional compiles

class BoardManager{
    public:
        BoardManager();
        int getLx();
        int getLy();
        bool getLb();
        int getRx();
        int getRy();
        bool getRb();
        char getGait();
        void readInputs();
    private:
        int lx, ly, rx, ry, lb, rb;
        char g;
        float batt, v_ref = 3.3, full_scale = 1024, low_batt = 2.5;
        void readLeftJS();
        void readRightJS();
        void readBattery();
        void lowBattery();
        void nrfConnected();
        
};

#endif