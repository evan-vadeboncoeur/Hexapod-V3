#include "RemoteControl.h"

RemoteControl::RemoteControl(){
    
}

void RemoteControl::initComm() // figure out if initializer list is needed for this 
{
    cmr.initCM();
    // initalize prev variables here?
}

void RemoteControl::stateManager(){
    readSensors(); // update sensor values
    buildTwist();
    // if gait_n != gait_o 
    // or if velocity command delta_direction > threshold
    // or if speed command delta_magnitude > threshold
    // build and transmit packet
    if(newCommand()) transmitMessage();
    handlePrevInputs(); // set previous values for comparison
}

bool RemoteControl::newCommand(){
    // gait change
    // left macro
    // right macro
    // angle change
    // significant magnitude change
    if((g_p != g) || (rb && (!rb_p)) || (lb && (!lb_p)) || (abs(theta_tw - theta_tw_p) > 0.0) || (abs(r - r_p) > R_DELTA)) return true; // theta: any amount change is grounds for new command since in this version we are discretely using multiples of PI/3
    else return false;
}

void RemoteControl::readSensors(){
    bmr.readInputs(); // read all inputs
    lb = bmr.getLb(); 
    lx = bmr.getLx();
    ly = bmr.getLy();
    rb = bmr.getRb();
    rx = bmr.getRx();
    ry = bmr.getRy();
    g = bmr.getGait();
}

void RemoteControl::transmitMessage(){
    cmd = cmr.buildPacket(t, bmr.getGait(), bmr.getLb(), bmr.getRb());
    Serial.println("cmd built");
    cmr.sendMessage(&cmd);
}
void RemoteControl::transmitMessage(char msg[]){
    cmr.sendMessage(msg);
}

void RemoteControl::handlePrevInputs(){
    theta_tw_p = theta_tw;
    vx_p = vx;
    vy_p = vy;
    wz_p = wz;
    rb_p = rb;
    lb_p = lb;
    g_p = g;
    r_p = r;
}

Vector RemoteControl::buildTwist(){
    // X_JS > 0 is left, Y_JS > 0 is up therefore we will treat the vertical pot as x and the horizontal as y to mimic the robot setup
    int x = ly, y = lx;
    x -= ADC_MID, y -= ADC_MID; // shift to midpoint of ranges to allow for signed values
    r = sqrt(x*x + y*y); // magnitude of the command, will designate speed
    r = map (r, 0, R_MAX, V_MIN, V_MAX); // map adc value to velocity min/max range (abs value of velocity, theta determines component signs)
    theta_tw = atan2(y,x); // will return the same as typical RHR XY coordinate system, works for this viewpoint of frame
    int sign = signbit(theta_tw); // figure out this part!!!
    theta_tw = abs(theta_tw);
    theta_tw = floor(theta_tw / M_PI_3)*M_PI_3; // make increment of PI/3
    theta_tw *= sign; // shift back to negative, if necessary
    
    // convert back to velocity components
    vx = r*cos(theta_tw); 
    vy = r*sin(theta_tw); 
    // handle angular another time...
    // TODO
    wz = 0.0;
    // in-class object
    t.setX1(vx);
    t.setX2(vy);
    t.setX3(wz);
    return Vector(vx, vy, wz);
}