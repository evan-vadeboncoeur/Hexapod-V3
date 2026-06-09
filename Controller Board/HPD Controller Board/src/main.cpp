#include "RemoteControl.h"

RemoteControl rc;
char msg[] = "test 2!!!";

void setup() {
    Serial.begin(9600);
    Serial.println("In Setup");
    delay(50);
}

void loop() {
    rc.transmitMessage(msg);
    Serial.println(msg);
    delay(50);
}