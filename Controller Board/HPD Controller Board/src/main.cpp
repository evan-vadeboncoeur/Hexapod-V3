#include "RemoteControl.h"
#include "HardwareSerial.h"
#define GLOBAL_DEBUG
RemoteControl rc;
#ifdef COMM_R_DEBUG
char msg[] = "test 2!!!";
#endif

void setup() {
    #ifdef GLOBAL_DEBUG
    Serial.begin(9600); // open before creating legs (at least in testing phases)
    delay(1000);
    Serial.println("#####In setup#####");
    #endif
    rc.initComm();
    delay(50);
}

void loop() {
    #ifdef GLOBAL_DEBUG
    Serial.println("#####In loop#####");
    #endif
    rc.stateManager();
    delay(CMD_DELAY);
}