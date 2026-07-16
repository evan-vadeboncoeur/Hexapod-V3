#ifndef COMM_H_H
#define COMM_H_H
#define COMM_DEBUG

#define CE_H (48)
#define CSN_H (49)
#define MOSI_H (51)
#define MISO_H (50)
#define SCK_H (52)
#define IRQ_NRF (2)

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "Packet.h"
#include <Arduino.h>
#include "HardwareSerial.h"

// receiver class for NRF24L01

class CommunicationManager{
    public:
        CommunicationManager();
        CommunicationManager(uint8_t ce, uint8_t cs);
        bool receivePacket();
        void receiveMessage();
        void commBegin();
        Packet p;
        char msg[10];
        int comm_ind = 0;
        Packet getPacket(){return p;}
    private:
        RF24 radio; // filled out in initializer list of constructor
        const uint64_t address = 0xDEADBEEF01; 
        const uint8_t pipe = 0;
        static void radioISR();
        volatile bool radio_interrupt = false;
        void handleInterrupt();
        static CommunicationManager* instance;
        
};


#endif