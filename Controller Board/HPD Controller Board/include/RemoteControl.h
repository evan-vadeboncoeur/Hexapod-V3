#ifndef REMOTE_H
#define REMOTE_H
#define REMOTE_DEBUG

#define L0 (0)
#define L1 (1)
#define L2 (2)
#define L3 (3)
#define L4 (4)
#define L5 (5)
#define M_PI_3 (1.04791)
#define M_PI_6 (0.52359)
#define ADC_MIN (0)
#define ADC_MAX (4095) // 12-bit
#define ADC_MID (ADC_MAX/2)
#define R_MAX (724) //(root2 * ADC_MID)
#define V_MIN (10) // mm/s, maginute, not maximum
#define V_MAX (140)
#define CMD_DELAY (1000)

#include "CommunicationManager.h"
#include "BoardManager.h"
#include "Vector.h"
// class for the entire remote controller (analog to hexapod class)

class RemoteControl{
    private:
        CommunicationManager cmr = CommunicationManager(CE_T, CSN_T);
        BoardManager bmr = BoardManager();
        Packet cmd;
        Vector t;
        void stateManager();
        void readSensors();
        void transmitMessage();
        float v_ref = 3.3; // reference voltage on the ESP32
        
    public:
        RemoteControl();
        void commandUpdate();
        void initComm();
        void transmitMessage(char msg[]);
        Vector buildTwist();
};

#endif