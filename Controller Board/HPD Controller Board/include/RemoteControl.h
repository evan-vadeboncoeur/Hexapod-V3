#ifndef REMOTE_H
#define REMOTE_H

#include "CommunicationsManager.h"
#include "BoardManager.h"
// class for the entire remote controller (analog to hexapod class)

class RemoteControl{
    private:
        CommunicationsManager cmr = CommunicationsManager(CE_T, CSN_T);
        BoardManager bmr = BoardManager();
        Packet cmd;
        RemoteControl();
        void commandUpdate();
        void readSensors();
        void transmitMessage();
    public:
};

#endif