#ifndef REMOTE_H
#define REMOTE_H
//#define REMOTE_DEBUG
#define TWIST_DEBUG
#define TRANSMIT_DEBUG
#define NEW_DEBUG

#define L0 (0)
#define L1 (1)
#define L2 (2)
#define L3 (3)
#define L4 (4)
#define L5 (5)
#define M_PI_3 (1.04719755)
#define M_PI_6 (0.523598776) 
#define ADC_MIN (0)
#define ADC_MAX (4095) // 12-bit
#define ADC_MID_18650_X (1782)
#define ADC_MID_18650_Y (1775)
#define CENTER_SHIFT_18650_X (266)
#define CENTER_SHIFT_18650_Y (273)
#define ADC_MID (1958) // should be 2048
#define CENTER_SHIFT (90) // |MID_Theoretical - MID_Actual|
#define X_BUFF (21)
#define Y_BUFF (21)
#define R_MIN (0)
#define R_MAX (2047) //(root2 * ADC_MID) (true ADC MID = 2894)
#define V_MIN (0) // mm/s
#define V_MAX (140)
#define V_MIN_CLAMP (15.0)
#define CMD_DELAY (300)
#define R_DELTA (5.0)
#define T_DELTA (1.0)
#define R_ZERO (10.0)

#include "CommunicationManager.h"
#include "BoardManager.h"
#include "Vector.h"
// class for the entire remote controller (analog to hexapod class)

class RemoteControl{
    private:
        CommunicationManager cmr = CommunicationManager(CE_T, CSN_T);
        BoardManager bmr = BoardManager();
        Packet cmd;
        //Vector t;
        void readSensors();
        void transmitMessage();
        void handlePrevInputs();
        bool newCommand();
        float v_ref = 3.3; // reference voltage on the ESP32
        int lx, ly, rx, ry;
        uint8_t lb, rb, lb_p, rb_p;
        uint8_t g, g_p;
        bool s_one, s_two, s_three, s_four;
        float theta_tw, theta_tw_p, vx, vx_p, vy, vy_p, v, v_p, wz, wz_p, r, r_p;
        
    public:
        RemoteControl();
        void stateManager();
        void initComm();
        void transmitMessage(char msg[]);
        void buildTwist();
};

#endif