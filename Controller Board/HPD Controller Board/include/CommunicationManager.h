#ifndef COMM_R_H
#define COMM_R_H
//#define COMM_R_DEBUG

#define CE_T (4)
#define CSN_T (5)

// MOSI: GPIO23
// MISO: GPIOD19
// SCK: GPIO18
// CE:  GPIO4
// CSN: GPIO5
// VCC: 3V3


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "Packet.h"

class CommunicationManager{
    public:
        CommunicationManager(int ce, int cs);
        void sendMessage(Packet *p);
        Packet buildPacket(Twist t, char g, bool lb, bool rb);
        void sendMessage(char msg[]);
    private:
        RF24 radio; // filled out in initializer list of constructor
        const uint64_t address = 0xDEADBEEF01; 
        const uint8_t pipe = 0;
        


};

#endif