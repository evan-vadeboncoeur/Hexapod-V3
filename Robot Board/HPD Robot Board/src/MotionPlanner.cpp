#include "MotionPlanner.h"
// CONSTRUCTORS
MotionPlanner::MotionPlanner(){
}

MotionPlanner::MotionPlanner(int g, float df, float tc, float sh)
    : b{Body(df, tc, sh)} // initialize body first (set alpha, idle, gait cycle values), set tripods too in body constructor
{
    delay(5);
    setGait(g, NWALK); // off rip
    // powerOnSequence()
    // waitForInput...?
}

// GAIT SETUP
void MotionPlanner::movementSetup(int g, Vector tw){
    setBodyVelocity(tw); // compute trajectories
    setGait(g, WALK); // calls desired gait, start walking
}

void MotionPlanner::setBodyVelocity(Vector tw){
    b.velocityCommand(tw); // set target twist value, foot velocities, position changes, etc.
}

void MotionPlanner::setGait(int g, bool walk){
    switch(g){
        case TRIPOD: 
            gait = TRIPOD;
            if(walk) tripodGait(TEST_STEPS); // test version
            break;
        case RIPPLE: 
            gait = RIPPLE;
        case WAVE: 
            gait = WAVE;
            break;
        case QUADRUPED: 
            gait = QUADRUPED;
            break;
        default:
            gait = TRIPOD;
            break;
    }
}

void MotionPlanner::walk(){
    switch(gait){
    case TRIPOD: 
        tripodGait();

        break;
    case RIPPLE: 

    case WAVE: 

        break;
    case QUADRUPED: 

        break;
    default:
        
        break;
    }
}

void MotionPlanner::turn(){
    setBodyVelocity(omega);
    tripodGait();
}

// GAITS
// teleop, indefinite version
bool MotionPlanner::tripodGait(){ // some way to check messages here... for now hardcode but future would be interrupts
    //while(walk_flag){ // finish cycle until walk_flag shuts off
    #ifdef PLAN_DEBUG
    Serial.println("**********************TRIPOD GAIT***********************");
    #endif
    delay(HALF_TRIPOD_DELAY);
    #ifdef PLAN_DEBUG
    Serial.println("--------------------HALF GAIT ODD--------------------");
    #endif
    halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
    delay(HALF_TRIPOD_DELAY);
    #ifdef PLAN_DEBUG
    Serial.println("--------------------HALF GAIT EVEN--------------------");
    #endif
    halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
    //} 
    return true; // tripod gait finished
}

bool MotionPlanner::tripodGait(int cc){
    #ifdef PLAN_DEBUG
    Serial.println("**********************TRIPOD GAIT***********************");
    #endif
    int num_cycle = 0;
    while(num_cycle < cc){
        #ifdef PLAN_DEBUG
        Serial.println("--------------------HALF GAIT ODD--------------------");
        #endif

        delay(HALF_TRIPOD_DELAY);
        half_c_prev = full_c_prev = millis();
        //halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
        halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
        half_c = (millis() - half_c_prev);
        delay(HALF_TRIPOD_DELAY);

        #ifdef PLAN_DEBUG
        Serial.print("1/2 gait even elapsed time: ");
        Serial.print('\t');
        Serial.println(((float)(half_c)/1000.0));
        Serial.println("--------------------HALF GAIT EVEN--------------------");
        #endif

        half_c_prev = millis();
       // halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
        halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
        half_c = (millis() - half_c_prev);
        full_c = (millis() - full_c_prev);

        #ifdef PLAN_DEBUG
        Serial.print("1/2 gait odd elapsed time: ");
        Serial.print('\t');
        Serial.print(((float)(half_c)/1000.0));
        Serial.print('\t');
        Serial.print("Full gait time: ");
        Serial.print('\t');
        Serial.println(((float)(full_c)/1000.0));
        #endif
        
        num_cycle++;
    } return true;
}

bool MotionPlanner::halfTripod(Leg** sw, Leg** st){
    delay(HALF_TRIPOD_DELAY);
    lift(sw);
    delay(HALF_TRIPOD_DELAY);
    push(st, sw);
    return true;
}

// lift non-moving legs before push/swing phase
bool MotionPlanner::lift(Leg** l_l){
    Leg* t_l; // temp leg for ids
    int t_id; // temp id
    Vector F_lift[NUM_LEGS/2]; // lifts in foot frames
    Vector J_lift[NUM_LEGS/2]; //lifts in joint space
    
    #ifdef PLAN_DEBUG
    Serial.println("---------LIFT----------");
    Serial.print("Leg: ");
    Serial.print('\t');
    Serial.print("X_B: ");
    Serial.print('\t');
    Serial.print("Y_B: ");
    Serial.print('\t');
    Serial.print("Z_B: ");
    Serial.print('\t');
    Serial.print("X_F: ");
    Serial.print('\t');
    Serial.print("Y_F: ");
    Serial.print('\t');
    Serial.print("Z_F: ");
    Serial.print('\t');
    Serial.print("J0: ");
    Serial.print('\t');
    Serial.print("J1: ");
    Serial.print('\t');
    Serial.println("J2: ");
    #endif
    
    // compute ids, foot coordinates, joint vectors for tripod
    for(int i=0; i<(NUM_LEGS/2); i++){
        t_l = *(l_l+i);
        t_id = t_l->getID();
        F_lift[i] = b.B_TF_L(b.getLift(t_id), t_id);
        J_lift[i] = b.computeIK(t_l, F_lift[i], ELBOW_DOWN);
        
        #ifdef PLAN_DEBUG
        Serial.print(t_id); // leg
        Serial.print('\t');
        Serial.print((b.getLift(t_id)).getX1()); // body coord
        Serial.print('\t');
        Serial.print((b.getLift(t_id)).getX2());
        Serial.print('\t');
        Serial.print((b.getLift(t_id)).getX3());
        Serial.print('\t');
        Serial.print(F_lift[i].getX1()); // foot coord
        Serial.print('\t');
        Serial.print(F_lift[i].getX2());
        Serial.print('\t');
        Serial.print(F_lift[i].getX3());
        Serial.print('\t');
        Serial.print(J_lift[i].getX1()); // joint coord
        Serial.print('\t');
        Serial.print(J_lift[i].getX2());
        Serial.print('\t');
        Serial.println(J_lift[i].getX3());
        #endif
    }
    // move all legs simulataneously
    b.moveTripod(l_l, J_lift);
    
    return true;
}

bool MotionPlanner::push(Leg** l_st, Leg** l_sw){
    Leg* t_st_l; // temp leg for ids
    Leg* t_sw_l;
    int t_st_id, t_sw_id; // temp id
    Vector F_stance[NUM_LEGS/2]; // lifts in foot frames
    Vector J_stance[NUM_LEGS/2]; //lifts in joint space
    Vector F_swing[NUM_LEGS/2]; // lifts in foot frames
    Vector J_swing[NUM_LEGS/2]; //lifts in joint space
    #ifdef PLAN_DEBUG
    Serial.println("---------PUSH----------");
    Serial.print("L_ST: ");
    Serial.print('\t');
    Serial.print("X_STB: ");
    Serial.print('\t');
    Serial.print("Y_STB: ");
    Serial.print('\t');
    Serial.print("Z_STB: ");
    Serial.print('\t');
    Serial.print("X_STF: ");
    Serial.print('\t');
    Serial.print("Y_STF: ");
    Serial.print('\t');
    Serial.print("Z_STF: ");
    Serial.print('\t');
    Serial.print("J0ST: ");
    Serial.print('\t');
    Serial.print("J1ST: ");
    Serial.print('\t');
    Serial.print("J2ST: ");
    Serial.print('\t');
    Serial.print("L_SW: ");
    Serial.print('\t');
    Serial.print("X_SWB: ");
    Serial.print('\t');
    Serial.print("Y_SWB: ");
    Serial.print('\t');
    Serial.print("Z_SWB: ");
    Serial.print('\t');
    Serial.print("X_SWF: ");
    Serial.print('\t');
    Serial.print("Y_SWF: ");
    Serial.print('\t');
    Serial.print("Z_SWF: ");
    Serial.print('\t');
    Serial.print("J0SW: ");
    Serial.print('\t');
    Serial.print("J1SW: ");
    Serial.print('\t');
    Serial.println("J2SW: ");
    #endif
    // compute all stances and swings
    for(int i=0; i<(NUM_LEGS/2); i++){
        t_st_l = *(l_st+i);
        t_sw_l = *(l_sw+i);
        t_st_id = t_st_l->getID();
        t_sw_id = t_sw_l->getID();
        F_stance[i] = b.B_TF_L(b.getStance(t_st_id), t_st_id);
        F_swing[i] = b.B_TF_L(b.getSwing(t_sw_id), t_sw_id);
        J_stance[i] = b.computeIK(t_st_l, F_stance[i], ELBOW_DOWN);
        J_swing[i] = b.computeIK(t_sw_l, F_swing[i], ELBOW_DOWN);

        #ifdef PLAN_DEBUG
        Serial.print(t_st_id); // leg
        Serial.print('\t');
        Serial.print((b.getStance(t_st_id)).getX1()); // body coord
        Serial.print('\t');
        Serial.print((b.getStance(t_st_id)).getX2());
        Serial.print('\t');
        Serial.print((b.getStance(t_st_id)).getX3());
        Serial.print('\t');
        Serial.print(F_stance[i].getX1()); // foot coord
        Serial.print('\t');
        Serial.print(F_stance[i].getX2());
        Serial.print('\t');
        Serial.print(F_stance[i].getX3());
        Serial.print('\t');
        Serial.print(J_stance[i].getX1()); // joint coord
        Serial.print('\t');
        Serial.print(J_stance[i].getX2());
        Serial.print('\t');
        Serial.print(J_stance[i].getX3());
        Serial.print('\t');
        Serial.print(t_sw_id); // leg
        Serial.print('\t');
        Serial.print((b.getSwing(t_sw_id)).getX1()); // body coord
        Serial.print('\t');
        Serial.print((b.getSwing(t_sw_id)).getX2());
        Serial.print('\t');
        Serial.print((b.getSwing(t_sw_id)).getX3());
        Serial.print('\t');
        Serial.print(F_swing[i].getX1()); // foot coord
        Serial.print('\t');
        Serial.print(F_swing[i].getX2());
        Serial.print('\t');
        Serial.print(F_swing[i].getX3());
        Serial.print('\t');
        Serial.print(J_swing[i].getX1()); // joint coord
        Serial.print('\t');
        Serial.print(J_swing[i].getX2());
        Serial.print('\t');
        Serial.println(J_swing[i].getX3());
        #endif
    }
    
   
    // move at same time
    
    //delay(HALF_TRIPOD_DELAY); // consider adding delay here
    
    b.moveTripod(l_sw, J_swing);
    b.moveTripod(l_st, J_stance);
    

    delay(50);
    return true;
}

// MOVEMENT MACROS
bool MotionPlanner::powerOnSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("**********************Powering On***********************");
    #endif
    moveHome();
    delay(50);
    moveIdle();
    return true;
}

bool MotionPlanner::moveHome(){
    #ifdef PLAN_DEBUG
    Serial.println("----------------------Moving To Home---------------------");
    Serial.println("----------Even Tripod----------");
    #endif
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_EVEN);
        #ifdef PLAN_DEBUG
        Serial.print("-----Moving Leg: ");
        Serial.print(cl->getID());
        Serial.println("-----");
        #endif
        b.moveLeg(cl, cl->home_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    delay(MACRO_DELAY);
    #ifdef PLAN_DEBUG
    Serial.println("----------Odd Tripod----------");
    #endif
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_ODD);
        #ifdef PLAN_DEBUG
        Serial.print("-----Moving Leg: ");
        Serial.print(cl->getID());
        Serial.println("-----");
        #endif
        b.moveLeg(cl, cl->home_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    return true;
}

bool MotionPlanner::moveIdle(){
    #ifdef PLAN_DEBUG
    Serial.println("----------------------Moving To Idle---------------------");
    Serial.println("----------Even Tripod----------");
    #endif
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_EVEN);
        #ifdef PLAN_DEBUG
        Serial.print("-----Moving Leg: ");
        Serial.print(cl->getID());
        Serial.println("-----");
        #endif
        b.moveLeg(cl, cl->idle_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    delay(MACRO_DELAY);
    #ifdef PLAN_DEBUG
    Serial.println("----------Odd Tripod----------");
    #endif
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_ODD);
        #ifdef PLAN_DEBUG
        Serial.print("-----Moving Leg: ");
        Serial.print(cl->getID());
        Serial.println("-----");
        #endif
        b.moveLeg(cl, cl->idle_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    return true;
}

bool MotionPlanner::moveStorage(){
    #ifdef PLAN_DEBUG
    Serial.println("----------------------Moving To Storage---------------------");
    #endif
    Serial.println("----------Even Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_EVEN);
        #ifdef PLAN_DEBUG
        Serial.print("-----Moving Leg: ");
        Serial.print(cl->getID());
        Serial.println("-----");
        #endif
        b.moveLeg(cl, cl->storage_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    delay(MACRO_DELAY);
    Serial.println("----------Odd Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_ODD);
        #ifdef PLAN_DEBUG
        Serial.print("-----Moving Leg: ");
        Serial.print(cl->getID());
        Serial.println("-----");
        #endif
        b.moveLeg(cl, cl->storage_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    return true;
}

bool MotionPlanner::powerOffSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("**********************Powering Off***********************");
    #endif
    moveIdle();
    delay(100);
    moveHome();
    delay(100);
    moveStorage();
    delay(1000);
    return true;
}