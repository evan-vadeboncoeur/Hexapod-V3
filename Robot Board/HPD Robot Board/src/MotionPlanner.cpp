
#include "MotionPlanner.h"
#ifndef PLAN_T_DEBUG
// CONSTRUCTORS

MotionPlanner::MotionPlanner(){

}

MotionPlanner::MotionPlanner(Leg** l, int g){
    legs = l; // pointer to array of legs
    setGait(g);
    setupTripod(0, 1, 20);
    delay(5);
}

// SETTERS 

// enum gait type
void MotionPlanner::setGait(int g){
    switch(g){
        case 0: gait = TRIPOD;
        case 1: gait = RIPPLE;
        case 2: gait = WAVE;
        case 3: gait = QUADRUPED;
    }
}

void MotionPlanner::setDistanceIncrement(int s_inc){
    s = s_inc;
}

void MotionPlanner::setDirection(int d){
    direction = d;
    //Serial.println("In Direction setter");
    
}
void MotionPlanner::setTargetSteps(int s){
    target_steps = s;
}

// GETTERS


// TRIPOD V1 CODE

bool MotionPlanner::tripodGait(int steps, int dir, int s_inc){
    int s_c=0;
    setupTripod(steps, dir, s_inc);
    prev_move = millis();
    while(s_c != steps){
        #ifdef PLAN_DEBUG
        Serial.println();
        Serial.println();
        Serial.print("Tripod Cycle: ");
        Serial.print('\t');
        Serial.println(s_c);
        #endif
        halfTripod(tp_L, tp_R);
        halfTripod(tp_R, tp_L);
        // leg cycle complete, increment steps
        s_c++;
        total_steps++;
        move_time = millis() - prev_move;
        prev_move = millis();
        #ifdef PLAN_DEBUG
        Serial.print("Gait cycle time: ");
        Serial.print('\t');
        Serial.print(move_time/1000.0);
        Serial.println('s');
        #endif
    }
    return true;

}

void MotionPlanner::setupTripod(int steps, int dir, int s_inc){
    setDirection(dir);
    setTargetSteps(steps);
    setDistanceIncrement(s_inc);
    sortTripod();
}

// sorts tripod based on chosen direction into left and right "halves"
// divides legs into two groups split along the edge of the chosen direction. finds pivot foot first (foot opposite other two in tripod), then assigns leg directions
// if direction == -1, then legs are going -Y (local to the leg, not the robot, as each leg is constructed in the same orientation [CCW])
void MotionPlanner::sortTripod(){
    for(int i=0,j=2,k=5; i<(NUM_LEGS / 2); i++,j+=2, k+=2){
        *(tp_L + i) = legs[(direction + j) % NUM_LEGS];
        *(tp_R + i) = legs[(direction + k) % NUM_LEGS];
    }
    // negative is always 3 left CW of direction, positive is always 3 right CCW of direction
    for(int i=0; i<(NUM_LEGS); i++){
        if(i<3)legs[(direction + i + 1) % NUM_LEGS]->setDirection(RVRS); // 3 CCW of leg are negative
        else legs[(direction + i + 1) % NUM_LEGS]->setDirection(FWD); // 3 CW of leg are positive
    }
    #ifdef PLAN_DEBUG
        Serial.println("Planner tripod printout:");
        Serial.println("Tripod 1:");
        Serial.print("Leg #: ");
        Serial.print('\t');
        Serial.println("Direction: ");
        for(int b=0; b<3; b++){
            Serial.print(tp_L[b]->getID());
            Serial.print('\t');
            Serial.println(tp_L[b]->getDirection());
        }
        Serial.println("Tripod 2:");
        Serial.print("Leg #: ");
        Serial.print('\t');
        Serial.println("Direction: ");
        for(int b=0; b<3; b++){
            Serial.print(tp_R[b]->getID());
            Serial.print('\t');
            Serial.println(tp_R[b]->getDirection());
        }
    #endif
}

bool MotionPlanner::halfTripod(Leg** l, Leg** p){
    liftLeg(l, liftC);
    swingLeg(l, swingC);
    pushLeg(p, pushC);
    plantLeg(l, plantC);
    return true;
}

bool MotionPlanner::liftLeg(Leg** trip, C_Position pos){
    Serial.println("Lift Leg 0: ");
    trip[0]->moveToIK(pos, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Lift Leg 1: ");
    trip[1]->moveToIK(pos, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Lift Leg 2: ");
    trip[2]->moveToIK(pos, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    return true;
}

bool MotionPlanner::swingLeg(Leg** trip, C_Position s){
    Serial.println("Swing Leg 0: ");
    trip[0]->moveToIK(s, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Swing Leg 1: ");
    trip[1]->moveToIK(s, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Swing Leg 2: ");
    trip[2]->moveToIK(s, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    return true;
}

bool MotionPlanner::plantLeg(Leg** trip, C_Position p){
    Serial.println("Plant Leg 0: ");
    trip[0]->moveToIK(p, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Plant Leg 1: ");
    trip[1]->moveToIK(p, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Plant Leg 2: ");
    trip[2]->moveToIK(p, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    return true;
}
bool MotionPlanner::pushLeg(Leg** trip, C_Position p){
    Serial.println("Push Leg 0: ");
    trip[0]->moveToIK(p, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Push Leg 1: ");
    trip[1]->moveToIK(p, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    Serial.println("Push Leg 2: ");
    trip[2]->moveToIK(p, ELBOW_DOWN, LHS);
    delay(MOTION_PLANNER_DELAY);
    return true;
}

// RBTF kinematics
// 1) generate movement direction, steps, and linear advance
// 2) generate global coordinates for each leg based on above (ie. start at L0 then extrapolate for L1-L5, lift, swing, plant, push)
// 3) convert to local coordinates for each leg
// 4) sequence motions for each leg
// 5) figure out how to handle CCW, config etc. in Leg class
// 6) maintain steps and direction until motion complete 

// express robot body coordinates in leg coordinate frame by transforming from G to B (global to body)
C_Position MotionPlanner::Body_TF_Leg(Leg* l, C_Position target){
    float x_L, y_L, z_L; // coordinates in the leg's base frame
    float x_B = target.getX(), y_B = target.getY(); // coordinates in the robot Body frame
    float a = l->getAlpha(), Link0_L = l->getLink0();
    x_L = x_B*cos(a) - Link0_L + y_B*sin(a);
    y_L = y_B*cos(a) - x_B*sin(a);
    z_L = target.getZ();
    //                         1                  
    C_Position output = C_Position(x_L, y_L, z_L); // body coordinates in leg frame
    return output;
} 

// edit to reverse values / negatives/ transpose whatever
Vector MotionPlanner::Foot_TF_Body(Leg* l, C_Position target){
    float x_L, y_L, z_L; // coordinates in the leg's base frame
    float x_B = target.getX(), y_B = target.getY(); // coordinates in the robot Body frame
    float a = l->getAlpha(), Link0_L = l->getLink0();
    x_L = x_B*cos(a) - Link0_L + y_B*sin(a);
    y_L = y_B*cos(a) - x_B*sin(a);
    z_L = target.getZ();
    //                         1                  
    Vector output = Vector(x_L, y_L, z_L); // body coordinates in leg frame
    return output;
}

// takes a target position in the walking frame (dictated by direction) and a given leg (which stores mount angle and attachment orientation) and computes TF from walking to leg
// frame is rotating alpha, and translated r along x_B
C_Position MotionPlanner::Walking_TF_Leg(Leg* l, C_Position target){
    float x_L, y_L, z_L; // coordinates in the leg's base frame
    float x_B = target.getX(), y_B = target.getY(); // coordinates in the robot Body frame
    float a = l->getAlpha(), Link0_L = l->getLink0();
    x_L = x_B*cos(a) - Link0_L + y_B*sin(a);
    y_L = y_B*cos(a) - x_B*sin(a);
    z_L = target.getZ();
    //                              1                  
    C_Position output = C_Position(x_L, y_L, z_L); // body coordinates in leg frame
    return output;
}

//new version, call at object instantiation and save alpha values
float MotionPlanner::computeWalkingAlpha(int id){
    float ret = M_PI_6*(2*(id-direction)-1); // simplified expression to calculate angular distance from walking direction to given leg
    if(ret > M_PI) ret-= (2*M_PI); // make shortest angle to X_W axis (measured about Z_W)
    if(ret < M_PI) ret+= (2*M_PI);
    return ret;
}

// MOVEMENT MACROS
bool MotionPlanner::powerOnSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("Powering On: ");
    #endif
    moveHome();
    delay(1000);
    moveIdle();
    // set Pn in body frame for the first time
    // legs[0]->forwardKinematics(legs[0]->getTargetJ());
    // C_Position target_idle = legs[0]->getTargetC();
    // for(int i =0; i<(NUM_LEGS-1); i++){
    //    R_Pn_foot[i] = Foot_TF_Body(legs[i], legs[i]->getTargetC()); // tf each leg's most recent target position to the body frame
    // }
}

bool MotionPlanner::powerOffSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("Powering Off: ");
    #endif
    moveIdle();
    delay(1000);
    moveHome();
    delay(1000);
    moveStorage();
}

bool MotionPlanner::moveHome(){
    #ifdef PLAN_DEBUG
    Serial.println("Moving to Home: ");
    #endif
    for(int l=0; l<(NUM_LEGS/2-1); l++){
        tp_L[l]->moveToJV(&home);
    }
    for(int l=0; l<(NUM_LEGS/2-1); l++){
        tp_R[l]->moveToJV(&home);
    }
    
    return true;
}

bool MotionPlanner::moveStorage(){
    #ifdef PLAN_DEBUG
    Serial.println("Moving to Storage: ");
    #endif
    for(int l=0; l<(NUM_LEGS/2-1); l++){
        tp_L[l]->moveToJV(&storage);
    }
    for(int l=0; l<(NUM_LEGS/2-1); l++){
        tp_R[l]->moveToJV(&storage);
    }
    return true;
}

bool MotionPlanner::moveIdle(){
    #ifdef PLAN_DEBUG
    Serial.println("Moving to Idle: ");
    #endif
    for(int l=0; l<(NUM_LEGS/2-1); l++){
        tp_L[l]->moveToJV(&idle);
    }
    for(int l=0; l<(NUM_LEGS/2-1); l++){
        tp_R[l]->moveToJV(&idle);
    }

    return true;
}

// end movement macros

// v2.1 kinematics 
void MotionPlanner::setTwist(Twist tw){
    t.v_x = tw.v_x;
    t.v_y = tw.v_y;
    t.w_z = tw.w_z;
}
// load twist values into object members
void MotionPlanner::unpackTwist(){
    vx = t.v_x;
    vy = t.v_y;
    wz = t.w_z;
    v_R = Vector(vx, vy);
}

// commandSetup()

// calculate all the position vectors of the feet in the body frame
void MotionPlanner::Body_r_Foot(){

}

// compute each foot velocity in the body frame of the robot needed to obtain twist vector
// B_VFoot_F = -
void MotionPlanner::Body_V_Foot(){
    for(int i=0; i<(NUM_LEGS-1); i++){

    }
}

// end v2.1 kinematics
#endif