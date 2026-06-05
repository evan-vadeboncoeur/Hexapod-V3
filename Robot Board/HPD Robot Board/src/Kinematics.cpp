#include "Kinematics.h"

Kinematics::Kinematics(){

}

Kinematics::Kinematics(float L1, float L2, float L3){
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    //serial = &Serial;
}

void Kinematics::setTarget(float x, float y, float z){
    p_x = x;
    p_y = y;
    p_z = z;
    target_p.setPosition(p_x, p_y, p_z);
}

C_Position Kinematics::fk(float config){
   
    // 3 x 3 rotation matrix
    R1_1 = cos(t2 + t3)*cos(t1);
    R1_2 = - 1.0*sin(t2 + t3)*cos(t1);
    R1_3 = -1.0*sin(t1);
    R2_1 = cos(t2 + t3)*sin(t1);
    R2_2 = - 1.0*sin(t2 + t3)*sin(t1);
    R2_3 = cos(t1);
    R3_1 = -1.0*sin(t2 + t3);
    R3_2 = -1.0*cos(t2 + t3);
    R3_3 = 0.0;
    // 3 x 1 positon vector of EE frame represented in global frame
    c_x = L1*cos(t1) + cos(t1)*(L3*cos(t2 + t3) + L2*cos(t2));
    c_y = L1*sin(t1) + sin(t1)*(L3*cos(t2 + t3) + L2*cos(t2));
    c_z = config*CCW_CONFIG*L3*sin(t2 + t3) + config*CCW_CONFIG*L2*sin(t2);

    calculated_p.setPosition(c_x, c_y, c_z);

    #ifdef FK_DEBUG
    Serial.println("FK OUTPUT: ");
    Serial.print("X_c: ");
    Serial.print('\t');
    Serial.print(c_x);
    Serial.print('\t');
    Serial.print("Y_c: ");
    Serial.print('\t');
    Serial.print(c_y);
    Serial.print('\t');
    Serial.print("Z_c: ");
    Serial.print('\t');
    Serial.println(c_z);
    #endif
    return C_Position(c_x, c_y, c_z);
}

// calculates the inverse kinematics based on the FK diagram. 
// +X is left, +Z is up, +Y is out of the page
// use side modifier to determine if leg is acting as LHS or RHS 
J_Position Kinematics::ik(bool config, float side){
    // Calculate theta 1 using XY projection of leg
    t1 = atan2(p_y, p_x);
    // Calculate theta 2 using beta, gamma, L4_x (x-component of L4 in the yaw plane), L4
    r = sqrt(p_x*p_x + p_y*p_y);
    L4_x = r - L1; // x component of L4 (J1 to P)
    L4 = sqrt(L4_x*L4_x + p_z*p_z);
    cb = (L2*L2 + L4*L4 - L3*L3) / (2*L2*L4); // x component of beta
    sb = sqrt(1 - cb*cb); // y component of beta (2 solutions for 2 configurations)
    beta1 = atan2(sb, cb); // angle between L2 and L4 (check for angle sense)
    beta2 = -beta1; // + beta = elbow down, - beta = elbow up
    gamma = side*CCW_CONFIG*atan2(p_z, L4_x); // angle between X axis and L4, + gamma = position below x-axis, - gamma = position above x-axis
    #ifdef IK_DEBUG
    Serial.println("Gamma/Beta Angles [rads]");
    Serial.print("L4_x");
    Serial.print('\t');
    Serial.print(L4_x);
    Serial.print('\t');
    Serial.print("Gamma: ");
    Serial.print('\t');
    Serial.print(gamma, 6);
    Serial.print('\t');
    Serial.print("Beta1: ");
    Serial.print('\t');
    Serial.print(beta1, 6);
    Serial.print('\t');
    Serial.print("Beta2: ");
    Serial.print('\t');
    Serial.println(beta2, 6);
    #endif
    t2_1 = CCW_CONFIG*(side*beta1 - gamma); // CCW to flip angle sense (LHS mounted servos)
    t2_2 = CCW_CONFIG*(side*beta2 - gamma);
    // Calculate t3
    ct3 = (L2*L2 + L3*L3 - L4*L4) / (2*L2*L3);
    st3 = sqrt(1 - ct3*ct3);
    t3_1 = M_PI + side*CCW_CONFIG*atan2(st3, ct3); // check angle sense???
    t3_2 = M_PI + side*CCW_CONFIG*atan2(-st3, ct3);
    // debug section
    #ifdef IK_DEBUG
    Serial.println("Mult. solutions output [rads]");
    Serial.print("T1: ");
    Serial.print('\t');
    Serial.print(t1, 6);
    Serial.print('\t');
    Serial.print("T2_1: ");
    Serial.print('\t');
    Serial.print(t2_1, 6);
    Serial.print('\t');
    Serial.print("T2_2: ");
    Serial.print('\t');
    Serial.print(t2_2, 6);
    Serial.print('\t');
    Serial.print("T3_1: ");
    Serial.print('\t');
    Serial.print(t3_1, 6);
    Serial.print('\t');
    Serial.print("T3_2: ");
    Serial.print('\t');
    Serial.println(t3_2, 6);
    #endif

    // Solve desired configuration and return
    return configCheck(config, side);
}

float Kinematics::normalizeAngles(float ang){
    while (ang < -M_PI) ang += 2.00*M_PI; // turn negative angle < PI into positive [0, PI]
    while (ang > M_PI) ang -= 2.00*M_PI; // turn positive angle > PI into negative [-PI, 0]
    return ang;
}

// returns the desired configuration of the leg (true = down, false = up)
J_Position Kinematics::configCheck(bool config, float side){
    // normalize all angles to ensure proper sign conventions 
    t2_1 = normalizeAngles(t2_1);
    t2_2 = normalizeAngles(t2_2);
    t3_1 = normalizeAngles(t3_1);
    t3_2 = normalizeAngles(t3_2);
    // check angle signs and associate with configurations
    if(side > 0){ // LHS Branch Logic
        if(config){
            t2 = (t2_1 < 0) ? t2_1 : t2_2; // elbow down
            t3 = (t3_1 > 0) ? t3_1 : t3_2;
        } else {
            t2 = (t2_1 > 0) ? t2_1 : t2_2; // elbow up
            t3 = (t3_1 < 0) ? t3_1 : t3_2;
        }
    } else { // RHS Branch Logic
        if(config){
            t2 = (t2_1 > 0) ? t2_1 : t2_2; // elbow down
            t3 = (t3_1 < 0) ? t3_1 : t3_2;
        } else {
            t2 = (t2_1 < 0) ? t2_1 : t2_2; // elbow up
            t3 = (t3_1 > 0) ? t3_1 : t3_2;
        }
    }
    
    #ifdef IK_DEBUG
    Serial.println("Branch Selection Angles [rads]");
    if(config){
        Serial.println("Leg down:");
    } else Serial.println("Leg up: ");
    Serial.print("T1: ");
    Serial.print('\t');
    Serial.print(t1);
    Serial.print('\t');
    Serial.print("T2: ");
    Serial.print('\t');
    Serial.print(t2);
    Serial.print('\t');
    Serial.print("T3: ");
    Serial.print('\t');
    Serial.println(t3);
    #endif

    #ifdef FK_DEBUG
    ikCheck(side);
    #endif
    return J_Position(t1, t2, t3); // return pointer (heap allocated memory using new keyword??)
}

float Kinematics::distance(C_Position desired, C_Position calculated){
    float dx = desired.getX() - calculated.getX();
    float dy = desired.getY() - calculated.getY();
    float dz = desired.getZ() - calculated.getZ();
    float total_d = sqrt(dx*dx + dy*dy + dz*dz);
    // DEBUG condition compile -> figure out how to pass serial monitor references?
    #ifdef IK_DEBUG
    Serial.println("IK-FK Comparison");
    // X error printout
    Serial.print("x_d: ");
    Serial.print('\t');
    Serial.print(desired.getX());
    Serial.print('\t');
    Serial.print("x_c: ");
    Serial.print('\t');
    Serial.print(calculated.getX());
    Serial.print('\t');
    Serial.print("dx: ");
    Serial.print('\t');
    Serial.print(dx);
    Serial.print('\t');
    // Y error printout
    Serial.print("y_d: ");
    Serial.print('\t');
    Serial.print(desired.getY());
    Serial.print('\t');
    Serial.print("y_c: ");
    Serial.print('\t');
    Serial.print(calculated.getY());
    Serial.print('\t');
    Serial.print("dy: ");
    Serial.print('\t');
    Serial.print(dy);
    Serial.print('\t');
    // Z error printout
    Serial.print("z_d: ");
    Serial.print('\t');
    Serial.print(desired.getZ());
    Serial.print('\t');
    Serial.print("z_c: ");
    Serial.print('\t');
    Serial.print(calculated.getZ());
    Serial.print('\t');
    Serial.print("dz: ");
    Serial.print('\t');
    Serial.print(dz);
    Serial.print('\t');
    // Total Distance Printout
    Serial.print("Distance error: ");
    Serial.print('\t');
    Serial.println(total_d);
    #endif
    
    return total_d;
}

bool Kinematics::ikCheck(float config){    
    fk(config); // calculate FK to determine magnitude of error
    if(fabs(distance(target_p, calculated_p)) < threshold) return true; // check computed distance difference vs threshold
    return false;
}