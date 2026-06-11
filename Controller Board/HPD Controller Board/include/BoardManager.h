#ifndef BRD_R_H
#define BRD_R_H
#define BRD_R_DEBUG

#define LX_P (34) // fill in place holders after reading the schematic
#define LY_P (39)
#define LB_P (2)
#define RX_P (14)
#define RY_P (12)
#define RB_P (13)
#define GP_1 (33)
#define GP_2 (25)
#define GP_3 (26)
#define GP_4 (27)
#define BATT_IN_P (15)
#define LOW_BATT_P (35)
#define NRFR_P (32)

#include <Arduino.h>
#include <HardwareSerial.h>
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
        bool s_one, s_two, s_three, s_four;
        float batt, v_ref = 3.3, full_scale = 1024, low_batt = 2.5;
        void readLeftJS();
        void readRightJS();
        void readBattery();
        void readDS();
        void lowBattery();
        void nrfConnected();
        
};

#endif