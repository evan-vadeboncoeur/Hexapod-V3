#include "Kinematics.h"

Kinematics::Kinematics(){

}

Kinematics::Kinematics(float L1, float L2, float L3){
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
}

void Kinematics::setTarget(float x, float y, float z){
    p_x = x;
    p_y = y;
    p_z = z;
    target_p.setPosition(p_x, p_y, p_z);
}

C_Position Kinematics::fk(){



    //calculated_p.setPosition();
}

J_Position Kinematics::ik(bool config){
    // Calculate theta 1 using XY projection of leg
    t1 = atan2(p_y, p_x);
    // Calculate theta 2 using beta, gamma, "L4"
    L4_x = p_x - L1; // x component of L4 (J1 to P)
    L4 = sqrt(L4_x*L4_x + p_z*p_z);
    cb = (L2*L2 + L4*L4 - L3*L3) / (2*L2*L4); // x component of beta
    sb = sqrt(cb); // y component of beta (2 solutions for 2 configurations)
    beta1 = atan2(sb, cb); // angle between L2 and L4 (check for angle sense)
    beta2 = atan2(-sb, cb);
    gamma = atan2(p_z, p_x); // angle between X axis and L4
    t2_1 = beta1 - gamma;
    t2_2 = beta2 - gamma;
    // Calculate t3
    ct3 = (L2*L2 + L3*L3 - L4*L4) / (2*L2*L3);
    st3 = sqrt(ct3);
    t3_1 = M_PI + (st3, ct3); // check angle sense
    t3_2 = M_PI + atan2(-st3, ct3);
    // Solve desired configuration and return
    return configCheck(true);
}

float Kinematics::normalizeAngles(float ang){
    while (ang < M_PI) ang += 2.00*M_PI; // turn negative angle < PI into positive [0, PI]
    while (ang > M_PI) ang -= 2.00*M_PI; // turn positive angle > PI into negative [-PI, 0]
    return ang;
}

// returns the desired configuration of the leg (true = down, false = up)
J_Position Kinematics::configCheck(bool config){
    // normalize all angles to ensure proper sign conventions 
    t2_1 = normalizeAngles(t2_1);
    t2_2 = normalizeAngles(t2_2);
    t3_1 = normalizeAngles(t3_1);
    t3_2 = normalizeAngles(t3_2);
    // check angle signs and associate with configurations
    if(config){
        t2 = (t2_1 > 0) ? t2_1 : t2_2; // elbow down
        t3 = (t3_1 < 0) ? t3_1 : t3_2;
    } else {
        t2 = (t2_1 < 0) ? t2_1 : t2_2; // elbow up
        t3 = (t3_1 > 0) ? t3_1 : t3_2;
    }

    return J_Position(t1, t2, t3); // return pointer (heap allocated memory using new keyword??)
}

float Kinematics::distance(C_Position desired, C_Position calculated){
    float dx = desired.getX() - calculated.getX();
    float dy = desired.getY() - calculated.getY();
    float dz = desired.getZ() - calculated.getZ();
    float total_d = sqrt(dx*dx + dy*dy + dz*dz);
    // DEBUG condition compile -> figure out how to pass serial monitor references?
    #ifdef DEBUG_IK
    



    #endif
    
    return total_d;
}

bool Kinematics::ikCheck(){    
    fk(); // calculate FK to determine magnitude of error
    if(fabs(distance(target_p, calculated_p)) < threshold) return true; // check computed distance difference vs threshold
    return false;
}