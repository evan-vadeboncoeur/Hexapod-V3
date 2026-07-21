#include "LegKinematics.h"

LegKinematics::LegKinematics(){

}

LegKinematics::LegKinematics(float L1, float L2, float L3, float configuration){
    this->L1 = L1;
    this->L2 = L2;
    this->L3 = L3;
    this->configuration = configuration;
}

void LegKinematics::setTargetPositionVector(Vector p){
    p_x = p.getX1();
    p_y = p.getX2();
    p_z = p.getX3();
    target_pV = p;
}

void LegKinematics::setTargetJointVector(Vector j){
    tht1 = j.getX1();
    tht2 = j.getX2();
    tht3 = j.getX3();
    target_jV = j;
}

// computes that FK on the leg of the chosen joint vector
Vector LegKinematics::fk(Vector t_jV){
    setTargetJointVector(t_jV); // setup tht variables
    // 3 x 3 rotation matrix
    R1_1 = cos(tht2 + tht3)*cos(tht1);
    R1_2 = - 1.0*sin(tht2 + tht3)*cos(tht1);
    R1_3 = configuration*sin(tht1);
    R2_1 = cos(tht2 + tht3)*sin(tht1);
    R2_2 = - 1.0*sin(tht2 + tht3)*sin(tht1);
    R2_3 = -configuration*cos(tht1);
    R3_1 = configuration*(tht2 + tht3);
    R3_2 = configuration*cos(tht2 + tht3);
    R3_3 = 0.0;
    // 3 x 1 positon vector of EE frame represented in global frame
    c_x = L1*cos(tht1) + cos(tht1)*(L3*cos(tht2 + tht3) + L2*cos(tht2));
    c_y = L1*sin(tht1) + sin(tht1)*(L3*cos(tht2 + tht3) + L2*cos(tht2));
    c_z = configuration*L3*sin(tht2 + tht3) + configuration*L2*sin(tht2); // ccw to get opposite angle sense
    
    calculated_pV = Vector(c_x, c_y, c_z);

    #ifdef LEG_FK_DEBUG
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
    return calculated_pV;
}

// calculates the inverse kinematics based on the FK diagram. 
// +X is left, +Z is up, +Y is out of the page
// use side modifier to determine if leg is acting as LHS or RHS 
Vector LegKinematics::ik(Vector t_pV, bool elbow){
    setTargetPositionVector(t_pV);
    #ifdef LEG_IK_DEBUG
    Serial.println("Position components [mm]:");
    Serial.print("p_x");
    Serial.print('\t');
    Serial.print(p_x);
    Serial.print('\t');
    Serial.print("p_y: ");
    Serial.print('\t');
    Serial.print(p_y, 6);
    Serial.print('\t');
    Serial.print("p_z: ");
    Serial.print('\t');
    Serial.print(p_z, 6);
    Serial.println();
    #endif
    // Calculate theta 1 using XY projection of leg
    t1 = atan2(p_y, fabs(p_x)); // assume t1 is < PI / X > 0
    // Calculate theta 2 using beta, gamma, L4_x (x-component of L4 in the yaw plane), L4
    r = sqrt(p_x*p_x + p_y*p_y);
    r_c = (r < L1) ? -1.0 : 1.0; // check if x < 0 or r is < L1 (L4_x component modifier)
    r_c2 = (r < L1) ? 1.0 : 0.0; // fpr t2 calculation (M_PI)
    L4_x = r - r_c*L1; // x component of L4 (J1 to P)
    L4 = sqrt(L4_x*L4_x + p_z*p_z);
    cb = (L2*L2 + L4*L4 - L3*L3) / (2*L2*L4); // x component of beta
    sb = sqrt(1 - cb*cb); // y component of beta (2 solutions for 2 configurations)
    beta1 = atan2(sb, cb); // angle between L2 and L4 (check for angle sense)
    beta2 = -beta1; // + beta = elbow down, - beta = elbow up
    gamma = atan2(r_c*configuration*p_z, L4_x); // angle between X axis and L4, + gamma = position below x-axis, - gamma = position above x-axis
    #ifdef LEG_IK_DEBUG
    Serial.println("Gamma/Beta Angles [rads]");
    Serial.print("RC: ");
    Serial.print('\t');
    Serial.print(r_c);
    Serial.print('\t');
    Serial.print("L4: ");
    Serial.print('\t');
    Serial.print(L4);
    Serial.print('\t');
    Serial.print("L4_x: ");
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
    t2_1 = M_PI*r_c2 + configuration*(beta1 - gamma); // conditional orientation (config.) & r/L1 (r_c2)
    t2_2 = M_PI*r_c2 + configuration*(beta2 - gamma);
    // Calculate t3
    ct3 = (L2*L2 + L3*L3 - L4*L4) / (2*L2*L3);
    st3 = sqrt(1 - ct3*ct3);
    t3_1 = M_PI + configuration*atan2(st3, ct3); 
    t3_2 = M_PI + configuration*atan2(-st3, ct3);
    // normalize all angles to ensure proper sign conventions 
    t2_1 = normalizeAngles(t2_1);
    t2_2 = normalizeAngles(t2_2);
    t3_1 = normalizeAngles(t3_1);
    t3_2 = normalizeAngles(t3_2);
    // debug section
    #ifdef LEG_IK_DEBUG
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
    calculated_jV = configurationHelper(elbow);  // Solve for desired configuration and return
    #ifdef MEMORY_DEBUG
    Serial.println(freeMemory());
    #endif
    //Vector output = calculated_jV;
    if(!ikCheck()) return Vector(0,0,0);
    return calculated_jV;
}

float LegKinematics::normalizeAngles(float ang){
    while (ang < -M_PI) ang += 2.00*M_PI; // turn negative angle < PI into positive [0, PI]
    while (ang > M_PI) ang -= 2.00*M_PI; // turn positive angle > PI into negative [-PI, 0]
    return ang;
}

// returns the desired configuration of the leg (true = down, false = up)
Vector LegKinematics::configurationHelper(bool elbow){
    // check angle signs and associate with configurations
    if(configuration < 1.0){ // LHS Branch Logic
        if(elbow && (r_c > 0.0)){
            t2 = (t2_1 < 0) ? t2_1 : t2_2; // elbow down, not approaching base sing
            t3 = (t3_1 > 0) ? t3_1 : t3_2;
        } else if(!elbow && (r_c > 0.0)) {
            t2 = (t2_1 > 0) ? t2_1 : t2_2; // elbow up, not approaching base sing
            t3 = (t3_1 < 0) ? t3_1 : t3_2;
        } else if(elbow && (r_c < 0.0)){ // ed, approaching base sing
            t2 = (t2_1 > 0) ? t2_1 : t2_2; 
            t3 = (t3_1 > 0) ? t3_1 : t3_2;
        } else { // eu, approaching base sing
            t2 = (t2_1 < 0) ? t2_1 : t2_2;
            t3 = (t3_1 < 0) ? t3_1 : t3_2;
        }
    } else { // RHS Branch Logic
        if(elbow && (r_c > 0.0)){
            t2 = (t2_1 > 0) ? t2_1 : t2_2; // elbow down
            t3 = (t3_1 < 0) ? t3_1 : t3_2;
        } else if(!elbow && (r_c > 0.0)) {
            t2 = (t2_1 < 0) ? t2_1 : t2_2; // elbow up
            t3 = (t3_1 > 0) ? t3_1 : t3_2;
        } else if(elbow && (r_c < 0.0)){
            t2 = (t2_1 < 0) ? t2_1 : t2_2; 
            t3 = (t3_1 < 0) ? t3_1 : t3_2;
        } else { // eu, approaching base sing
            t2 = (t2_1 > 0) ? t2_1 : t2_2;
            t3 = (t3_1 > 0) ? t3_1 : t3_2;
        }
    }
    #ifdef LEG_IK_DEBUG
    Serial.println("Branch Selection Angles [rads]");
    if(elbow){
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

    return Vector(t1, t2, t3); 
}

// make sure we're within the required distance
bool LegKinematics::ikCheck(){    
    fk(calculated_jV);
    float dx = (target_pV.getX1() - calculated_pV.getX1());
    float dy = target_pV.getX2() - calculated_pV.getX2();
    float dz = target_pV.getX3() - calculated_pV.getX3();
    float d_t = sqrt(dx*dx + dy*dy + dz*dz);
    #ifdef LEG_IK_DEBUG
    Serial.println("IK-FK Comparison");
    // X error printout
    Serial.print("x_d: ");
    Serial.print('\t');
    Serial.print(target_pV.getX1());
    Serial.print('\t');
    Serial.print("x_c: ");
    Serial.print('\t');
    Serial.print(calculated_pV.getX1());
    Serial.print('\t');
    Serial.print("dx: ");
    Serial.print('\t');
    Serial.print(target_pV.getX1() - calculated_pV.getX1());
    Serial.print('\t');
    // Y error printout
    Serial.print("y_d: ");
    Serial.print('\t');
    Serial.print(target_pV.getX2());
    Serial.print('\t');
    Serial.print("y_c: ");
    Serial.print('\t');
    Serial.print(calculated_pV.getX2());
    Serial.print('\t');
    Serial.print("dy: ");
    Serial.print('\t');
    Serial.print(target_pV.getX2() - calculated_pV.getX2());
    Serial.print('\t');
    // Z error printout
    Serial.print("z_d: ");
    Serial.print('\t');
    Serial.print(target_pV.getX3());
    Serial.print('\t');
    Serial.print("z_c: ");
    Serial.print('\t');
    Serial.print(calculated_pV.getX3());
    Serial.print('\t');
    Serial.print("dz: ");
    Serial.print('\t');
    Serial.print(target_pV.getX3() - calculated_pV.getX3());
    Serial.print('\t');
    // Total Distance Printout
    Serial.print("Distance error: ");
    Serial.print('\t');
    Serial.println(d_t);
    #endif
    if(fabs(d_t < threshold)) return true; // check computed distance difference vs threshold
    return false;
}


