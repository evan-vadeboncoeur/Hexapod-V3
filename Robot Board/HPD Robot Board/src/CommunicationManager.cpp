#include "CommunicationManager.h"
// pointer needed for "this" object for calling interrupt handler
CommunicationManager* CommunicationManager::instance = nullptr; // cant set this in .h, needs to be constant in .h, nullptr is not const

// overload for designated pins
CommunicationManager::CommunicationManager(uint8_t ce, uint8_t cs) : radio(ce, cs){}

CommunicationManager::CommunicationManager() : radio(CE_H, CSN_H) {
    // initialize the RF24 radio object with the given CE and CSN pins
    // this constructor uses an initializer list to directly initialize the radio member
    // without needing to create a temporary RF24 object and assign it to radio
    #ifdef COMM_DEBUG
    Serial.println("********************Initializing Radio**********************");
    #endif
    instance = this;
    #ifdef COMM_DEBUG
    Serial.println("----------Instance Created-----------");
    #endif
    //radio.setStatusFlags(RF24_RX_DR); // try setStatusFlags instead of deprecated maskIRQ (still hangs w/o NRF)
    //radio.maskIRQ(true, true, false); // configure which events trigger interrupt request pin (IRQ). active low. tx_success, tx_failure, rx_ready (data received)
    #ifdef COMM_DEBUG
    Serial.println("----------IRQ Bits Masked-----------");
    #endif
    //pinMode(IRQ_NRF, INPUT_PULLUP); 
    #ifdef COMM_DEBUG
    Serial.println("----------IRQ Pin Configured-----------");
    #endif
    #ifndef IQR2_DEBUG
    attachInterrupt(digitalPinToInterrupt(IRQ_NRF), radioISR, FALLING);
    #endif
    #ifdef COMM_DEBUG
    Serial.println("----------Interrupt Attached-----------");
    #endif
}

void CommunicationManager::radioISR(){ // static, so does not receive a hidden object* this pointer -> void (*)() rather than void(*)(CommunicationManager*)
    //instance->radio_interrupt = true; alternatively use the class object to set the flag...
    if(instance){ // ? should be true since it isnt null? assuming that is what this checks for
        instance->handleInterrupt();
    }
}

void CommunicationManager::handleInterrupt(){
    #ifndef IQR2_DEBUG
    bool tx_ok, tx_fail, rx_ready;
    radio.whatHappened(tx_ok, tx_fail, rx_ready); // check reason for IQR and reset pin to high
    if(rx_ready) radio_interrupt = true; // rx was the reason for the flag
    digitalWrite(NRF_LED, HIGH);
    #endif
    //radio_interrupt = true;
    //Serial.println("Radio handler!");
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
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(RADIO_CHANNEL); // select channel frequency (2400MHz + Channel)
    radio.flush_rx();
    radio.startListening(); 
}

// recieve new packet -> let robot know
bool CommunicationManager::receivePacket(){
    #ifdef COMM_DEBUG
    Serial.println("####################Checking for Packet####################");
    #endif
    #ifndef IQR2_DEBUG
    if(radio_interrupt){ // comment this out for testing purposes
    #endif
        bool packet_received = false;
        while(radio.available()) { 
            radio.read(&p, sizeof(p)); // read all packets in the queue to get newest command
            packet_received = true;
        }

        #ifndef IQR2_DEBUG // outside of packet received block in case false-positive
        radio_interrupt = false;
        digitalWrite(NRF_LED, LOW);
        #endif

        if(packet_received){
            //Serial.println(p.v_x);

            // uint8_t* ptr = (uint8_t*)&p;

            // for(int i=0;i<sizeof(Packet);i++){
            //     Serial.print(ptr[i], HEX);
            //     Serial.print(' ');
            // }
            // Serial.println();
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
            Serial.print(p.v_x);
            Serial.print('\t');
            Serial.print(p.v_y);
            Serial.print('\t');
            Serial.println(p.w_z);
            #endif
            return true;
        }    
    #ifndef IQR2_DEBUG
    }
    #endif
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
