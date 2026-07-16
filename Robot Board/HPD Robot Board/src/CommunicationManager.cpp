#include "CommunicationManager.h"

CommunicationManager* CommunicationManager::instance = nullptr; // cant set this in .h, needs to be constant in .h, nullptr is not const

CommunicationManager::CommunicationManager(uint8_t ce, uint8_t cs) : radio(ce, cs){

}

CommunicationManager::CommunicationManager() : radio(CE_H, CSN_H) {
    // initialize the RF24 radio object with the given CE and CSN pins
    // this constructor uses an initializer list to directly initialize the radio member
    // without needing to create a temporary RF24 object and assign it to radio
    //instance = this;
    //radio.maskIRQ(true, true, false); // configure which events trigger interrupt request pin (IRQ). active low. tx_success, tx_failure, rx_ready (data received)
    //pinMode(IRQ_NRF, INPUT_PULLUP); 
    //attachInterrupt(digitalPinToInterrupt(IRQ_NRF), radioISR, FALLING);
}

void CommunicationManager::radioISR(){ // static, so does not receive a hidden object* this pointer -> void (*)() rather than void(*)(CommunicationManager*)
    if(instance){
        instance->handleInterrupt();
    }
}

void CommunicationManager::handleInterrupt(){
    radio_interrupt = true;
    #ifdef COMM_DEBUG
    Serial.println("Radio handler!");
    #endif
}

void CommunicationManager::commBegin(){
    #ifdef COMM_DEBUG
    Serial.println("--------------------COMM INITIALIZATION--------------------");
    #endif
    if(!radio.begin()){
        #ifdef COMM_DEBUG
        Serial.println("FATAL: No NRF (MEGA2560) Connected. Check wiring. Program Terminating.");
        #endif
        delay(3000);
        exit(1); 
    }
    radio.openReadingPipe(pipe, address); // TX/RX must agree on address
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening(); 
}

// recieve new packet -> let robot know
bool CommunicationManager::receivePacket(){
    #ifdef COMM_DEBUG
    Serial.println("####################Checking for Packet####################");
    #endif
    //if(radio_interrupt){ // comment this out for testing purposes
        while(radio.available()) // read all packets in the queue to get newest command
        //radio_interrupt = false;
        radio.read(&p, sizeof(p));
        //add debug prints here
        #ifdef COMM_DEBUG
        Serial.println("-----Test Packet Recieved-----");
        Serial.print("Gait: ");
        Serial.print('\t');
        Serial.print("P Off: ");
        Serial.print('\t');
        Serial.print("PowOn: ");
        Serial.print('\t');
        Serial.print("Tw Vx: ");
        Serial.print('\t');
        Serial.print("Tw Vy: ");
        Serial.print('\t');
        Serial.println("Tw Wz: ");
        Serial.print(p.g);
        Serial.print('\t');
        Serial.print(p.lb);
        Serial.print('\t');
        Serial.print(p.rb);
        Serial.print('\t');
        Serial.print(p.t.getX1());
        Serial.print('\t');
        Serial.print(p.t.getX2());
        Serial.print('\t');
        Serial.println(p.t.getX3());
        #endif
        return true;
    //}
    return false;
}

// use for debugging packet setup (rec. strings)
void CommunicationManager::receiveMessage(){
    if(radio.available()){
        Serial.println("Message is available. ");
        radio.read(&msg, sizeof(msg));
        #ifdef COMM_DEBUG
        Serial.println(msg);
        #endif
    } 
    #ifdef COMM_DEBUG
    else Serial.println("Message not available.");
    #endif
}
