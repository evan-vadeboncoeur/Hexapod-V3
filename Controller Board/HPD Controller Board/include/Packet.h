#ifndef PACKET_H
#define PACKET_H

#include "Vector.h"
// struct for un/packing data packets for the NRF24L01

struct __attribute__((packed)) Packet{
    float v_x;
    float v_y;
    float w_z;
    uint8_t g; // gait
    uint8_t lb; // left button
    uint8_t rb; // right button
        
};

#endif