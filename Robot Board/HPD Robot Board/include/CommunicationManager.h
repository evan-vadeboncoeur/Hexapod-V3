#ifndef COMM_H_H
#define COMM_H_H
//#define COMM_H_DEBUG


#define CE_H (48)
#define CSN_H (49)
#define MOSI_H (51)
#define MISO_H (50)
#define SCK_H (52)

// nrf24l01 communication manager: receive commands from board
// packet consists of:
// first bit: control mode (teleop/robot control)
// next 2 bits: gait mode (TRIPOD, RIPPLE, WAVE, QUADRUPED) (2 dipswitch)
// next 4 bits: direction (FORWARD, BACK, LEFT_UP, LEFT_DOWN, RIGHT_UP, RIGHT_DOWN, ROTATE_CW, ROTATE_CCW) (analog joystick L)
// next 4 bits: steps (dependent on analog read of how far stick is moved)


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
        Packet getPacket(){return p;}
    private:
        RF24 radio; // filled out in initializer list of constructor
        const uint64_t address = 0xDEADBEEF01; 
        const uint8_t pipe = 0;
        


};


#endif