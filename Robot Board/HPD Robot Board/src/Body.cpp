#include "Body.h"

Body::Body(){

}

Body::Body(float df, float t_c, float sh) // initialize each leg before contructing the rest of the object
    : legs{
        Leg(LEG_0, LEG_0_J0, LEG_0_J1, LEG_0_J2, CCW_CONFIG),
        Leg(LEG_1, LEG_1_J0, LEG_1_J1, LEG_1_J2, CCW_CONFIG),
        Leg(LEG_2, LEG_2_J0, LEG_2_J1, LEG_2_J2, CCW_CONFIG),
        Leg(LEG_3, LEG_3_J0, LEG_3_J1, LEG_3_J2, CCW_CONFIG),
        Leg(LEG_4, LEG_4_J0, LEG_4_J1, LEG_4_J2, CCW_CONFIG),
        Leg(LEG_5, LEG_5_J0, LEG_5_J1, LEG_5_J2, CCW_CONFIG)
    } 
{
    step_height = sh;
    duty_factor = df;
    t_cycle = t_c;
    t_stance = t_cycle*duty_factor;
    computeAlphaI(); // offset angles for each leg - calculate based on chassis circle and angle mounting
    L_TF_B(legs[LEG_0].idle_p_L); // use idle position of Leg to compute initial leg positions (legs will go from storage -> home -> idle but pos. not tracked til idle)
    z_g = foot_idle_R[LEG_0].getX3(); // load in ground height
}

void Body::velocityCommand(Vector tw){
    unpackTwist(tw); // unpack twist if not handled elsewhere
    compute_vR(); // compute individual foot velocities to satisfy command
    compute_dP(); // compute displacement vectors for each foot
    compute_SLS(); // compute stance and swing vectors for each foot
    //compute_pN_L(); // probably dont need this - handle in motion planner class !!!
    // now move on to motion planner to order legs??
    // compute_pN_L = computeStance for each leg
    // move on to computeLift
    // computeSwing (stance magnitude)
    // computePlant ...
}

void Body::unpackTwist(Vector tw){
    body_velocity = Vector(tw.getX1(), tw.getX2());
    bv = body_velocity.getMagnitude();
    theta = body_velocity.getTheta();
    vx = bv*cos(theta);
    vy = bv*sin(theta);
    wz = tw.getX3();
}

// compute rotation angles for RBTF between body and leg frame
// compute x and y tf components
void Body::computeAlphaI(){
    for(int i=0; i<(NUM_LEGS); i++){
        alpha_i[i] = M_PI_3*i + M_PI_6;
        alpha_ci[i] = cos(alpha_i[i]); // reduce sin/cos computations @ runtime
        alpha_si[i] = sin(alpha_i[i]);
        #ifdef SETUP_BK_DEBUG
        Serial.println("AlphaI Setup: ");
        Serial.print("Leg: ");
        Serial.print('\t');
        Serial.println("Angle: ");
        Serial.print(i);
        Serial.print('\t');
        Serial.println(alpha_i[i]);
        #endif
    }
}

// computes velocity of each foot in the body frame (need to establish bases first at time of instantiation - run sim backward to calc.)
void Body::compute_vR(){
    Vector t_w;
    float v_x, v_y;
    #ifdef VR_BK_DEBUG
    Serial.println("VR computed: ");
    Serial.print("Leg: ");
    Serial.print('\t');
    Serial.print("v_x: ");
    Serial.print('\t');
    Serial.println("v_y: ");
    #endif
    for(int i=0; i<(NUM_LEGS); i++){
        // v_x = -(vx + (-wz*foot_p_R[i].getX2()));
        // v_y = -(vy + (wz*foot_p_R[i].getX1()));
        v_x = -(vx + (-wz*foot_idle_R[i].getX2()));
        v_y = -(vy + (wz*foot_idle_R[i].getX1()));
        t_w = Vector(v_x, v_y);
        foot_v_R[i] = t_w;
        Serial.print(i);
        Serial.print('\t');
        Serial.print(foot_v_R[i].getX1());
        Serial.print('\t');
        Serial.println(foot_v_R[i].getX2());
    }     
}

// compute position displacement of each foot in the body frame based on the timestep/duty factor and previous position
void Body::compute_dP(){
    float dP_x, dP_y;
    for(int i=0; i<(NUM_LEGS); i++){
        dP_x = foot_v_R[i].getX1() * t_stance;
        dP_y = foot_v_R[i].getX2() * t_stance;
        foot_dp_R[i] = Vector(dP_x, dP_y);
    }
}

// calculate stance, lift, swing values in the body frame, store in vector arrays
void Body::compute_SLS(){
    float linear_interpolation = 0.5; // halfway along trajectory
    float l_x, l_y, l_z;
    float st_x, st_y, st_z;
    float sw_x, sw_y, sw_z;
    //p_nom = p_lift, p_swing = p_nom + dP_i/2, p_stance = p_nom - dP_i/2
    //p_lift = (1 - s)*p_stance + s*p_swing, s = 0.5 (halfway through cycle) [x y]^T ?? dont need this since we have idle i think
    //p_lift_z = z_ground + h*sin(pi*s), z_ground = p_idle_z (see idle value), h = clearance height = 20mm, s = 0.5 (halfway pt of cycle)
    // compute lift of leg, apply in for loop
    l_z = z_g + step_height*sin(M_PI*linear_interpolation); // only need to compute once
    st_z = sw_z = z_g; // only set once, added for clarity

    #ifdef SLS_DEBUG
    Serial.println("SLS computed: ");
    Serial.print("Leg");
    Serial.print('\t');
    Serial.print("St_x");
    Serial.print('\t');
    Serial.print("St_y");
    Serial.print('\t');
    Serial.print("St_z");
    Serial.print('\t');
    Serial.print("Sw_x");
    Serial.print('\t');
    Serial.print("Sw_y");
    Serial.print('\t');
    Serial.print("Sw_z");
    Serial.print('\t');
    Serial.print("Lf_x");
    Serial.print('\t');
    Serial.print("Lf_y");
    Serial.print('\t');
    Serial.println("Lf_z");
    #endif
    // compute stance and swing of each leg
    for(int i=0; i<(NUM_LEGS); i++){
        // stance
        st_x = foot_idle_R[i].getX1() - foot_dp_R[i].getX1()/2.0;
        st_y = foot_idle_R[i].getX2() - foot_dp_R[i].getX2()/2.0;
        foot_stance_R[i] = Vector(st_x, st_y, st_z);
        // swing
        sw_x = foot_idle_R[i].getX1() + foot_dp_R[i].getX1()/2.0;
        sw_y = foot_idle_R[i].getX2() + foot_dp_R[i].getX2()/2.0;
        foot_swing_R[i] = Vector(sw_x, sw_y, sw_z);
        // lift
        l_x = foot_idle_R[i].getX1(); // xy differ for each leg, z is constant
        l_y = foot_idle_R[i].getX2();
        foot_lift_R[i] = Vector(l_x, l_y, l_z);
        
        #ifdef SLS_DEBUG
        Serial.print(i);
        Serial.print('\t');
        Serial.print(foot_stance_R[i].getX1());
        Serial.print('\t');
        Serial.print(foot_stance_R[i].getX2());
        Serial.print('\t');
        Serial.print(foot_stance_R[i].getX3());
        Serial.print('\t');
        Serial.print(foot_swing_R[i].getX1());
        Serial.print('\t');
        Serial.print(foot_swing_R[i].getX2());
        Serial.print('\t');
        Serial.print(foot_swing_R[i].getX3());
        Serial.print('\t');
        Serial.print(foot_lift_R[i].getX1());
        Serial.print('\t');
        Serial.print(foot_lift_R[i].getX2());
        Serial.print('\t');
        Serial.println(foot_lift_R[i].getX3());
        #endif
    }
}

// transform vector from body coordinates to leg coordinates given a vector and leg id
Vector Body::B_TF_L(Vector bc, int id){
    float bx, by, bz, lx, ly, lz; // temporary xyz components
    bx = bc.getX1();
    by = bc.getX2();
    bz = bc.getX3();
    lx = bx*alpha_ci[id]+ by*alpha_si[id] - chassis_radius; 
    ly = by*alpha_ci[id] - bx*alpha_si[id];
    lz = bz;
    return Vector(lx, ly, lz);
}

// compute new position in the leg frame (LEGACY)
void Body::compute_pN_L(){
    Vector t_v;
    float t_x_b, t_y_b, t_x_f, t_y_f, t_z_f;
    #ifdef STANCE_BK_DEBUG
    Serial.println("STANCE POSITION for Twist CMD: ");
    Serial.print("Leg: ");
    Serial.print('\t');
    Serial.print("X_R: ");
    Serial.print('\t');
    Serial.print("Y_R: ");
    Serial.print('\t');
    Serial.println("Z_R: ");
    #endif
    for(int i=0; i<(NUM_LEGS); i++){
        t_v = foot_p_R[i];
        t_x_b = t_v.getX1();
        t_y_b = t_v.getX2();
        t_z_f = t_v.getX3();
        t_x_f = t_x_b*alpha_ci[i]+ t_y_b*alpha_si[i] - chassis_radius; 
        t_y_f = t_y_b*alpha_ci[i] - t_x_b*alpha_si[i];
        t_v = Vector(t_x_f, t_y_f, t_z_f);
        foot_p_L[i] = t_v;
        legs[i].setTargetFootP(t_v);
        #ifdef STANCE_BK_DEBUG
        Serial.print(i);
        Serial.print('\t');
        Serial.print(foot_p_L[i].getX1());
        Serial.print('\t');
        Serial.print(foot_p_L[i].getX2());
        Serial.print('\t');
        Serial.println(foot_p_L[i].getX3());
        #endif
    }
}

// TF function to help setup idle pose for robot -> store in foot_idle_R[] array
Vector Body::L_TF_B(Vector v){
    Vector t_v;
    float t_x_b, t_y_b, t_z_b, t_x_f, t_y_f, t_z_f;  // foot frame, robot/body frame
    t_x_f = v.getX1();
    t_y_f = v.getX2();
    t_z_f = v.getX3();
    #ifdef SETUP_BK_DEBUG
    Serial.println("Body Kinematic Setup: ");
    Serial.print("Leg: ");
    Serial.print('\t');
    Serial.print("X_R: ");
    Serial.print('\t');
    Serial.print("Y_R: ");
    Serial.print('\t');
    Serial.println("Z_R: ");
    #endif
    for(int i=0; i<(NUM_LEGS); i++){
        t_x_b = t_x_f*alpha_ci[i] - t_y_f*alpha_si[i] + chassis_radius*alpha_ci[i]; 
        t_y_b = t_y_f*alpha_ci[i] + t_x_f*alpha_si[i] + chassis_radius*alpha_si[i] ;
        t_z_b = t_z_f;
        t_v = Vector(t_x_b, t_y_b, t_z_b);
        foot_p_R[i] = t_v;
        foot_idle_R[i] = t_v;
        legs[i].setTargetFootP(t_v);
        #ifdef SETUP_BK_DEBUG
        Serial.print(i);
        Serial.print('\t');
        Serial.print(foot_p_R[i].getX1());
        Serial.print('\t');
        Serial.print(foot_p_R[i].getX2());
        Serial.print('\t');
        Serial.println(foot_p_R[i].getX3());
        #endif
    }
}