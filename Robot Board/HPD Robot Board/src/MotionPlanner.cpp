
#include "MotionPlanner.h"
// CONSTRUCTORS

MotionPlanner::MotionPlanner(){
}

MotionPlanner::MotionPlanner(int g, int df, int tc, int sh)
    : b{Body(df, tc, sh)} // initialize body first (set alpha, idle, gait cycle values), set tripods too in body constructor
{
    delay(5);
    setGait(g); // off rip
    // powerOnSequence()
    // moveHome()
        // moveIdle()
    // waitForInput...?
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

// MOVEMENT MACROS
bool MotionPlanner::powerOnSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("**********************Powering On***********************");
    #endif
    moveHome();
    delay(50);
    moveIdle();
}

bool MotionPlanner::moveHome(){
    #ifdef PLAN_DEBUG
    Serial.println("----------------------Moving To Home---------------------");
    #endif
    Serial.println("----------Even Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_EVEN);
        b.moveLeg(cl, cl->home_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    delay(100);
    Serial.println("----------Odd Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_ODD);
        b.moveLeg(cl, cl->home_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    
    return true;
}

bool MotionPlanner::moveIdle(){
    #ifdef PLAN_DEBUG
    Serial.println("----------------------Moving To Idle---------------------");
    #endif
    Serial.println("----------Even Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_EVEN);
        b.moveLeg(cl, cl->idle_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    delay(100);
    Serial.println("----------Odd Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_ODD);
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
        b.moveLeg(cl, cl->storage_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
    delay(100);
    Serial.println("----------Odd Tripod----------");
    for(int l=0; l<(NUM_LEGS/2); l++){
        Leg* cl = b.getLegTripod(l, TP_ODD);
        b.moveLeg(cl, cl->storage_j_L, JOINT_MOVE, ELBOW_DOWN);
    }
}

bool MotionPlanner::powerOffSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("**********************Powering Off***********************");
    #endif
    moveIdle();
    delay(1000);
    moveHome();
    delay(1000);
    moveStorage();
}
/** 








// end movement macros



// end v2.1 kinematics
**/

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