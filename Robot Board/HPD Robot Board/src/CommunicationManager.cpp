#include "CommunicationManager.h"

CommunicationManager::CommunicationManager(uint8_t ce, uint8_t cs) : radio(ce, cs){

}

CommunicationManager::CommunicationManager() : radio(CE_H, CSN_H) {
    // initialize the RF24 radio object with the given CE and CSN pins
    // this constructor uses an initializer list to directly initialize the radio member
    // without needing to create a temporary RF24 object and assign it to radio
    
}

void CommunicationManager::commBegin(){
    radio.begin();
    radio.openReadingPipe(pipe, address); // TX/RX must agree on address
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening(); 
}

// transmit the packet to the reciever (packet already constructed)
bool CommunicationManager::receivePacket(){
    if(radio.available()){
        radio.read(&p, sizeof(p));
        #ifdef COMM_H_DEBUG
        Serial.print("Gait: ");
        Serial.print('\t');
        Serial.print(p.g);
        Serial.print('\t');
        Serial.print("LX: ");
        Serial.print('\t');
        Serial.print(p.lx);
        #endif
        return true;
    }
    return false;
}

// use for debugging
void CommunicationManager::receiveMessage(){
    if(radio.available()){
        Serial.println("Message is available. ");
        radio.read(&msg, sizeof(msg));
        #ifdef COMM_H_DEBUG
        Serial.println(msg);
        #endif
    } 
    #ifdef COMM_H_DEBUG
    else Serial.println("Message not available");
    #endif
}

// construct a packet struct (data values already read - probably isolate to power class)
Packet CommunicationManager::buildPacket(char g, int lx, int ly, bool lb, int rx, int ry, bool rb){
    Packet p;
    p.g = g;
    p.lx = lx;
    p.ly = ly;
    p.lb = lb;
    p.rx = rx;
    p.ry = ry;
    p.rb = rb;
    return p;
}