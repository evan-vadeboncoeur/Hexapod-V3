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
    cmd = cmr.buildPacket(t, bmr.getGait(), bmr.getLb(), bmr.getRb());
    cmr.sendMessage(&cmd);
}
void RemoteControl::transmitMessage(char msg[]){
    cmr.sendMessage(msg);
}

Twist RemoteControl::buildTwist(){
    // X > 0 is left, Y > 0 is up
    // therefore, to match X_R, Y_R, we map the vertical pot to X and the horizontal to Y to not have to adjust the frame
    int x = bmr.getLy(), y = bmr.getLx();
    float r = sqrt(x*x - y*y); // magnitude of the command, will designate speed, (or position if desired. if T_Stance and Duty Factor are constant, this is the same thing)
    float theta = atan2(y,x);
    while(theta < M_PI) theta += 2*M_PI; // normalize angle, remember theta starts at 0, straight up
    while(theta > M_PI) theta -= 2*M_PI;
    // this version will be only for walking in a straight line, no ICC (6 discrete directions)
    // in advanced version, w will be given using the right stick, and the velocity vector on the left will be 360 degrees
    // that said, I need to figure out rotate in place, ICC calculations for v2 / v3
    if(theta >= -M_PI_6 && theta <= M_PI_6)theta = 0; // between L1 and L2
    else if(theta >= M_PI_6 && theta <= M_PI_2) theta = M_PI_3;
    else if(theta >= M_PI_2 && theta <= M_PI*(0.8333)) theta = 0.667*M_PI;
    else if(theta <= -M_PI_6 && theta >= -M_PI_2) theta = -M_PI_3; // use negative angles starting here
    else if(theta <= -M_PI_2 && theta >= -(0.667)*M_PI) theta = -(0.8333)*M_PI;
    else if(theta <= -(0.833)*M_PI || theta >= (0.833)*M_PI) theta = M_PI; // 180 degrees, should be a catch all from every other range
    else theta = 0.00; // default to D1 

    Twist t_r;
    r = map (r, ADC_MIN, ADC_MAX, V_MIN, V_MAX);
    float vx, vy;
    vx = r*sin(theta); // flip x and y components since X is up, y is left... or not, despends on sign stuff
    vy = r*cos(theta); // figure out if we need a negative... yeah we need some sort of angle handling here
    t_r.v_x = vx;
    t.v_x = vx;
    t_r.v_y = vy;
    t.v_y = vy;
    return t_r;
}