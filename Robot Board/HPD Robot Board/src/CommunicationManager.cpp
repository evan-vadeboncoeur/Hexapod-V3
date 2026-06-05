#include "CommunicationManager.h"

CommunicationManager::CommunicationManager(){

}

CommunicationManager::CommunicationManager(char ce, char cs) : radio(ce, cs) {
    // initialize the RF24 radio object with the given CE and CSN pins
    // this constructor uses an initializer list to directly initialize the radio member
    // without needing to create a temporary RF24 object and assign it to radio
    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening(); 
}

// transmit the packet to the reciever (packet already constructed)
void CommunicationManager::receiveMessage(){
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
        //p = radio.read
    }
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