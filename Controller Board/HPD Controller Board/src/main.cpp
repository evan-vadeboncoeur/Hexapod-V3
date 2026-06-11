#include "RemoteControl.h"

RemoteControl rc;
#ifdef COMM_R_DEBUG
char msg[] = "test 2!!!";
#endif

void setup() {
    Serial.begin(9600);
    Serial.println("In Setup");
    delay(50);
}

void loop() {
    #ifdef COMM_R_DEBUG
    rc.transmitMessage(msg);
    Serial.println(msg);
    #endif
    rc.commandUpdate(); // add code that tracks previous twist values and compares -> if 0, keep doing same thing, or maybe not idk
    delay(CMD_DELAY);
}