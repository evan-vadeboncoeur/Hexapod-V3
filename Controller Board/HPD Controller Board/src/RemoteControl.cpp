#include "RemoteControl.h"

RemoteControl::RemoteControl(){

}

void RemoteControl::initComm(){
    cmr.initCM();
}

void RemoteControl::stateManager(){
    readSensors(); // update sensor values
    // if gait_n != gait_o 
    // or if velocity command delta_direction > threshold
    // or if speed command delta_magnitude > threshold
    // build packet 
    // transmit packet
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

Vector RemoteControl::buildTwist(){
    // X_JS > 0 is left, Y_JS > 0 is up therefore we will treat the vertical pot as x and the horizontal as y to mimic the robot setup
    int x = bmr.getLy(), y = bmr.getLx();
    x -= ADC_MID, y -= ADC_MID; // shift to midpoint of ranges to allow for signed values
    float r = sqrt(x*x + y*y); // magnitude of the command, will designate speed
    r = map (r, 0, R_MAX, V_MIN, V_MAX); // map adc value to velocity min/max range (abs value of velocity, theta determines component signs)
    float theta = atan2(y,x); // will return the same as typical RHR XY coordinate system, works for this viewpoint of frame
    int sign = signbit(theta); // figure out this part!!!
    theta = abs(theta);
    theta = floor(theta / M_PI_3)*M_PI_3; // make increment of PI/3
    theta *= sign; // shift back to negative, if necessary
    
    // convert back to velocity components
    float vx, vy;
    vx = r*cos(theta); 
    vy = r*sin(theta); 
    // handle angular another time...
    // TODO
    float wz = 0.0;
    // in-class object
    t.setX1(vx);
    t.setX2(vy);
    t.setX3(wz);

    return Vector(vx, vy, wz);
}