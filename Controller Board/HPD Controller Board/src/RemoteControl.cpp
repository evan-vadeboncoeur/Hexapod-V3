#include "RemoteControl.h"

RemoteControl::RemoteControl(){

}

void RemoteControl::initComm(){
    cmr.initCM();
}

void RemoteControl::commandUpdate(){
    readSensors(); // updates sensor values in BM class
    buildTwist(); // calculate twist command
    //transmitMessage(); // send to designated address on designated pipe using NRF
}

void RemoteControl::readSensors(){
    bmr.readInputs();
}

void RemoteControl::transmitMessage(){
    cmd = cmr.buildPacket(t, bmr.getGait(), bmr.getLb(), bmr.getRb());
    Serial.println("cmd built");
    cmr.sendMessage(&cmd);
}
void RemoteControl::transmitMessage(char msg[]){
    cmr.sendMessage(msg);
}

Twist RemoteControl::buildTwist(){
    // X_JS > 0 is left, Y_JS > 0 is up therefore we will treat the vertical pot as x and the horizontal as y to mimic the robot setup
    int x = bmr.getLx(), y = bmr.getLy();
    x -= ADC_MID, y -= ADC_MID; // shift to midpoint of ranges to allow for signed values
    float r = sqrt(x*x + y*y); // magnitude of the command, will designate speed, (or position if desired. if T_Stance and Duty Factor are constant, this is the same thing)
    r = map (r, -ADC_MID, ADC_MID, V_MIN, V_MAX); // map adc value to velocity min/max range
    float theta = atan2(y,x); // will return the same as typical RHR XY coordinate system, works for this viewpoint of frame
    int sign = signbit(theta); // figure out this part!!!
    theta = abs(theta);
    theta = floor(theta / M_PI_3)*M_PI_3; // make increment of PI/3
    theta *= sign; // shift back to negative, if necessary
    
    Twist t_r;
    float vx, vy;
    vx = r*sin(theta); // flip x and y components since X is up, y is left... or not, despends on sign stuff
    vy = r*cos(theta); // figure out if we need a negative... yeah we need some sort of angle handling here
    t_r.v_x = vx;
    t.v_x = vx;
    t_r.v_y = vy;
    t.v_y = vy;
    return t_r;
}