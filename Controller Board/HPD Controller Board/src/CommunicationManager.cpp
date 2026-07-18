#include "CommunicationManager.h"

CommunicationManager::CommunicationManager(int ce, int cs) : radio(ce, cs) {
    // initialize the RF24 radio object with the given CE and CSN pins
    // this constructor uses an initializer list to directly initialize the radio member
    // without needing to create a temporary RF24 object and assign it to radio
    
}

// transmit the packet to the reciever (packet already constructed)
void CommunicationManager::sendMessage(){
    //p.v_x = 1.0;
    //p.v_y = 2.0;
    #ifdef COMM_DEBUG
    Serial.println("**********Sending Command**********");
    Serial.print("P size");
    Serial.print('\t');
    Serial.println("xv");
    Serial.print(sizeof(p));
    Serial.print('\t');
    Serial.println(p.v_x);
    #endif
    radio.write(&p, sizeof(p));
}

void CommunicationManager::sendMessage(char msg[]){
    int i = 0;
    while(msg[i] != '\0') i++;
    
    Serial.println(i);
    radio.write(msg, i);
}

void CommunicationManager::commBegin(){
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
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(RADIO_CHANNEL);
    radio.stopListening(); // stop listening for incoming messages, switch to transmit mode
}

// construct a packet struct (data values already read - probably isolate to power class)
void CommunicationManager::buildPacket(float x, float y, float w, uint8_t g, uint8_t lb, uint8_t rb){
    p.v_x = x;
    p.v_y = y;
    p.w_z = w;
    p.g = g;
    p.lb = lb;
    p.rb = rb;

}
