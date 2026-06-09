#ifndef REMOTE_H
#define REMOTE_H

#include "CommunicationManager.h"
#include "BoardManager.h"
// class for the entire remote controller (analog to hexapod class)

class RemoteControl{
    private:
        CommunicationManager cmr = CommunicationManager(CE_T, CSN_T);
        BoardManager bmr = BoardManager();
        Packet cmd;
        
        void commandUpdate();
        void readSensors();
        void transmitMessage();
        
    public:
        RemoteControl();
        void transmitMessage(char msg[]);
};

#endif