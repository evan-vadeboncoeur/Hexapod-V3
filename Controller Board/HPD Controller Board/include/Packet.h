#ifndef PACKET_H
#define PACKET_H

#include "Twist.h"
// struct for un/packing data packets for the NRF24L01

struct Packet{
    Twist t; // twist command
    char g; // gait
    bool lb; // left button
    bool rb; // right button
        
};

#endif