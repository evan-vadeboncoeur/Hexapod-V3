#ifndef PACKET_H
#define PACKET_H

#include "Vector.h"
// struct for un/packing data packets for the NRF24L01

struct Packet{
    Vector t; // twist command
    uint8_t g; // gait
    bool lb; // left button
    bool rb; // right button
        
};

#endif