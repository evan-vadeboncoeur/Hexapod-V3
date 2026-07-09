#ifndef PACKET_H
#define PACKET_H

// struct for un/packing data packets for the NRF24L01

struct Packet{
    Vector t; // twist command
    char g; // gait
    bool lb; // left button
    bool rb; // right button
        
};

#endif