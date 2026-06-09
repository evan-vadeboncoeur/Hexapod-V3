#include "RemoteControl.h"

RemoteControl::RemoteControl(){

}

void RemoteControl::commandUpdate(){
    readSensors(); // updates sensor values in BM class
    transmitMessage();
}

void RemoteControl::readSensors(){
    bmr.readInputs();
}

void RemoteControl::transmitMessage(){
    cmd = cmr.buildPacket(bmr.getGait(), bmr.getLx(), bmr.getLy(), bmr.getLb(), bmr.getRx(), bmr.getRy(), bmr.getRb());
    cmr.sendMessage(&cmd);
}
void RemoteControl::transmitMessage(char msg[]){
    cmr.sendMessage(msg);
}