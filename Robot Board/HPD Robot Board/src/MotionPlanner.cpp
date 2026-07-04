
#include "MotionPlanner.h"
// CONSTRUCTORS

MotionPlanner::MotionPlanner(){
}

MotionPlanner::MotionPlanner(int g, int df, int tc, int sh)
    : b{Body(df, tc, sh)} // initialize body first (set alpha, idle, gait cycle values), set tripods too in body constructor
{
    delay(5);
    setGait(g, NWALK); // off rip
    // powerOnSequence()
    // waitForInput...?
}

// GAIT SETUP
void MotionPlanner::movement(int g, Vector tw){
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
            if(walk) tripodGait(5); // test version
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

// GAITS
// teleop, indefinite version
bool MotionPlanner::tripodGait(){
    while(walk_flag){ // finish cycle until walk_flag shuts off

    } return true; // tripod gait finished
}

bool MotionPlanner::tripodGait(int cc){
    #ifdef PLAN_DEBUG
    Serial.println("**********************TRIPOD GAIT***********************");
    #endif
    int num_cycle = 0;
    while(num_cycle < cc){
        #ifdef PLAN_DEBUG
        Serial.println("--------------------HALF GAIT EVEN--------------------");
        #endif
        half_c_prev = full_c_prev = millis();
        halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
        half_c = (millis() - half_c_prev);
        #ifdef PLAN_DEBUG
        Serial.print("Half gait even time: ");
        Serial.print('\t');
        Serial.println(((float)(half_c)/1000.0));
        Serial.println("--------------------HALF GAIT ODD--------------------");
        #endif
        half_c_prev = millis();
        halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
        half_c = (millis() - half_c_prev);
        full_c = (millis() - full_c_prev);
        #ifdef PLAN_DEBUG
        Serial.print("Half gait odd time: ");
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
    lift(sw);
    delay(10);
    push(st, sw);
    return true;
}

// lift non-moving legs before push/swing phase
bool MotionPlanner::lift(Leg** l_l){
    Leg* cl;
    int id;
    #ifdef PLAN_DEBUG
    Serial.println("---------LIFT----------");
    #endif
    for(int i=0; i<(NUM_LEGS/2); i++){    
        cl = *(l_l+i); // leg in tripod
        id = cl->getID(); // id for position index in body trajectory 
        #ifdef PLAN_DEBUG
        Serial.print("Lift Leg: ");
        Serial.print('\t');
        Serial.println(id);
        #endif
        b.moveLeg(cl, b.getLift(id), CART_MOVE, ELBOW_DOWN); // cmd move
    }
    return true;
}

bool MotionPlanner::push(Leg** l_st, Leg** l_sw){
    Leg* st;
    Leg* sw;
    int st_id, sw_id;
    #ifdef PLAN_DEBUG
    Serial.println("---------STANCE----------");
    #endif
    for(int i=0; i<(NUM_LEGS/2); i++){    
        st = *(l_st+i), sw = *(l_sw + i); // swing and stance leg
        st_id = st->getID(), sw_id = sw->getID(); // ids
        #ifdef PLAN_DEBUG
        Serial.print("Stance Leg: ");
        Serial.print('\t');
        Serial.print(st_id);
        Serial.print('\t');
        Serial.print("Swing Leg: ");
        Serial.print('\t');
        Serial.println(sw_id);
        #endif
        b.moveLeg(sw, b.getSwing(sw_id), CART_MOVE, ELBOW_DOWN); // cmd move
        delay(10);
        b.moveLeg(st, b.getStance(st_id), CART_MOVE, ELBOW_DOWN);
    }
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
    delay(100);
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
    delay(100);
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
    delay(100);
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
    delay(50);
    moveHome();
    delay(50);
    moveStorage();
    return true;
}

/*** Old tripod gait sequence
 * 
 * bool MotionPlanner::liftLeg(Leg** trip, C_Position pos){
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
 * 
 * 
 */