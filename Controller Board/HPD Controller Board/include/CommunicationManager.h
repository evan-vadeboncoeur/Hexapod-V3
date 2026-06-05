#ifndef COMM_R_H
#define COMM_R_H

#define CE_T (4)
#define CSN_T (5)

// MOSI: D23
// MISO: D19
// SCK: D18
// CE: D4
// CSN: D5
// VCC: 3V3


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "Packet.h"

class CommunicationManager{
    public:
        CommunicationManager(int ce, int cs);
        void sendMessage(Packet *p);
        Packet buildPacket(char g, int lx, int ly, bool lb, int rx, int ry, bool rb);
    private:
        RF24 radio; // filled out in initializer list of constructor
        const uint64_t address = 0xDEADBEEF01; 
        const uint8_t pipe = 0;
        


};

#endif