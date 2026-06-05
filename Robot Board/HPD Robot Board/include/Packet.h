#ifndef PACKET_H
#define PACKET_H

// struct for un/packing data packets for the NRF24L01

struct Packet{
    char g; // gait
    int lx;
    int ly;
    bool lb; // left button
    int rx;
    int ry;
    bool rb;
        
};

#endif