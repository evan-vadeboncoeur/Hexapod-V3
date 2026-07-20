#include "RemoteControl.h"

RemoteControl::RemoteControl(){
    
}

void RemoteControl::initRC(){
    #ifdef REMOTE_DEBUG
    Serial.println("####################JOYSTICK CENTER SETUP####################");
    #endif
    cmr.commBegin();
    int adc_read_x[ADC_READINGS];
    int adc_read_y[ADC_READINGS];
    int adc_x_sum=0, adc_y_sum=0;
    for(int i=0; i<ADC_READINGS; i++){
        adc_x_sum += bmr.readLeftX();
        adc_y_sum += bmr.readLeftY();
        delay(40);
    }
    adc_mid_x = adc_x_sum / ADC_READINGS;
    adc_mid_y = adc_y_sum / ADC_READINGS;
    center_shift_x = ADC_MID - adc_mid_x;
    center_shift_y = ADC_MID - adc_mid_y;

    #ifdef REMOTE_DEBUG
    Serial.print("MidX");
    Serial.print('\t');
    Serial.print("MidY");
    Serial.print('\t');
    Serial.print("ShiftX");
    Serial.print('\t');
    Serial.println("ShiftY");
    Serial.print(adc_mid_x);
    Serial.print('\t');
    Serial.print(adc_mid_y);
    Serial.print('\t');
    Serial.print(center_shift_x);
    Serial.print('\t');
    Serial.println(center_shift_y);
    #endif
}

void RemoteControl::stateManager(){
    readSensors(); // update sensor values
    buildTwist(); // build twist value based on current joystick positioning
    // if there is a new command, build and transmit new message
    if(newCommand()) transmitMessage();
    handlePrevInputs(); // set previous values for comparison
}

bool RemoteControl::newCommand(){
    // gait change, left macro, right macro, angle change, significant magnitude change, 0 velocity (& not prev 0 velocity)
    
    #ifdef NEW_DEBUG
    uint8_t a = (abs(r - r_p) > R_DELTA);
    uint8_t b = (r < R_ZERO);
    Serial.println("--------------------New Command Debug--------------------");
    Serial.print("Gait");
    Serial.print('\t');
    Serial.print("Power On");
    Serial.print('\t');
    Serial.print("Power Off");
    Serial.print('\t');
    Serial.print("Angle");
    Serial.print('\t');
    Serial.print("V Mag");
    Serial.print('\t');
    Serial.println("V 0");
    Serial.print((g_p != g));
    Serial.print('\t');
    Serial.print((lb != lb_p));
    Serial.print('\t');
    Serial.print((rb != rb_p));
    Serial.print('\t');
    Serial.print((abs(theta_tw - theta_tw_p) > T_DELTA));
    Serial.print('\t');
    Serial.print(a);
    Serial.print('\t');
    Serial.println(b);
    #endif
    if((g_p != g) || (rb != rb_p) || (lb != lb_p) || (abs(theta_tw - theta_tw_p) > T_DELTA) || (abs(r - r_p) > R_DELTA) || (r <= R_ZERO && (r_p > R_ZERO))) return true; // theta: any amount change is grounds for new command since in this version we are discretely using multiples of PI/3
    else return false;
}

void RemoteControl::readSensors(){
    bmr.readInputs(); // read all inputs
    lb = !(bmr.getLb()); // flip to opposite (active low -> active high)
    lx = bmr.getLx();
    ly = bmr.getLy();
    rb = !(bmr.getRb());
    rx = bmr.getRx();
    ry = bmr.getRy();
    g = bmr.getGait();
    g = (g == 0) ? g : (g - 1);
    #ifdef REMOTE_DEBUG
    Serial.println("--------------------Sensor Output--------------------");
    Serial.print("Lb");
    Serial.print('\t');
    Serial.print("Lx");
    Serial.print('\t');
    Serial.print("Ly");
    Serial.print('\t');
    Serial.print("Rb");
    Serial.print('\t');
    Serial.print("Rx");
    Serial.print('\t');
    Serial.print("Ry");
    Serial.print('\t');
    Serial.println("Gait");
    Serial.print(lb);
    Serial.print('\t');
    Serial.print(lx);
    Serial.print('\t');
    Serial.print(ly);
    Serial.print('\t');
    Serial.print(rb);
    Serial.print('\t');
    Serial.print(rx);
    Serial.print('\t');
    Serial.print(ry);
    Serial.print('\t');
    Serial.println(g);
    #endif
}

void RemoteControl::transmitMessage(){
    cmr.buildPacket(vx, vy, wz, bmr.getGait(), bmr.getLb(), bmr.getRb());
    cmr.sendMessage();
    bmr.transmitBlink();
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

void RemoteControl::buildTwist(){
    // X_JS > 0 is left, Y_JS > 0 is up therefore we will treat the vertical pot as x and the horizontal as y to mimic the robot setup
    int x = lx, y = ly; // get adc values
    x -= adc_mid_x, y -= adc_mid_y; // shift to midpoint of ranges to allow for signed values and process out 0 velocity
    x = (abs(x) < X_BUFF || x == 0) ? 0 : ((x > 0) ? (x + center_shift_x) : (x - center_shift_x)); // if not 0 or in 0 range, shift back to full-scale +/- ranges (could make back to nested ternary with +/- 1 to make symmetrical about 0)
    y = (abs(y) < Y_BUFF || y == 0) ? 0 : ((y > 0) ? (y + center_shift_y) : (y - center_shift_y));
    // compute magnitude of velocity vector
    r = sqrt(x*x + y*y); // magnitude of the command, will designate speed
    r = map(r, R_MIN, R_MAX, V_MIN, V_MAX); // map adc value to velocity min/max range (abs value of velocity, theta determines component signs)
    r = (r > (float)V_MAX) ? (float)V_MAX : r; // clamp to V_MAX
    // compute angle of velocity vector, centered along the perpendicular bisector of 1 of 6 edges between J0's
    theta_tw = (float)(atan2(y,x)); // will return the same as typical RHR XY coordinate system, works for this viewpoint of frame
    theta_tw = M_PI_3*roundf(theta_tw/M_PI_3); // Quantization to pi/3 buckets centered at theta = 0
    // convert back to velocity components
    vx = r*cos(theta_tw); 
    vy = r*sin(theta_tw); 
    // clamp just in case map didnt filter
    vx = (abs(vx) < X_BUFF) ? 0.0 : vx;
    vy = (abs(vy) < Y_BUFF) ? 0.0 : vy; // ? why is vy negative sometimes? fp rounding?
    // TODO
    wz = 0.0;
    #ifdef TWIST_DEBUG
    Serial.println("-----TWIST DEBUG-----");
    Serial.print("r");
    Serial.print('\t');
    Serial.print("theta");
    Serial.print('\t');
    Serial.print("vx");
    Serial.print('\t');
    Serial.print("vy");
    Serial.print('\t');
    Serial.println("wz");
    Serial.print(r);
    Serial.print('\t');
    Serial.print(theta_tw);
    Serial.print('\t');
    Serial.print(vx);
    Serial.print('\t');
    Serial.print(vy);
    Serial.print('\t');
    Serial.println(wz);
    #endif
    
}