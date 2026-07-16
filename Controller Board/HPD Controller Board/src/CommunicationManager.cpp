#include "CommunicationManager.h"

CommunicationManager::CommunicationManager(int ce, int cs) : radio(ce, cs) {
    // initialize the RF24 radio object with the given CE and CSN pins
    // this constructor uses an initializer list to directly initialize the radio member
    // without needing to create a temporary RF24 object and assign it to radio
    
}

// transmit the packet to the reciever (packet already constructed)
void CommunicationManager::sendMessage(Packet *p){
    radio.write(p, sizeof(*p));
    digitalWrite(32, HIGH);
    delay(200);
    digitalWrite(32, LOW);
}

void CommunicationManager::sendMessage(char msg[]){
    int i = 0;
    while(msg[i] != '\0') i++;
    
    Serial.println(i);
    radio.write(msg, i);
}

void CommunicationManager::initCM(){
    #ifdef COMM_DEBUG
    Serial.println("--------------------COMM INITIALIZATION--------------------");
    #endif
    radio.begin();
    //if(!radio.begin()){
        #ifdef COMM_DEBUG
        Serial.println("FATAL: No NRF (ESP32) Connected. Check wiring. Program Terminating.");
        #endif
        delay(3000);
        //exit(1);
    //}
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_MIN);
    radio.stopListening(); // stop listening for incoming messages, switch to transmit mode
}

// construct a packet struct (data values already read - probably isolate to power class)
Packet CommunicationManager::buildPacket(Vector t, uint8_t g, bool lb, bool rb){
    Packet p;
    p.t = t;
    p.g = g;
    p.lb = lb;
    p.rb = rb;
    return p;
}
