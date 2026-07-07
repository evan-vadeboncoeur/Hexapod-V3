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
            if(walk) tripodGait(3); // test version
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
        halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
        delay(100);
        halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
    } return true; // tripod gait finished
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
        delay(200);
        half_c_prev = full_c_prev = millis();
        halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
        half_c = (millis() - half_c_prev);
        delay(200);
        #ifdef PLAN_DEBUG
        Serial.print("0.5 even elapsed time: ");
        Serial.print('\t');
        Serial.println(((float)(half_c)/1000.0));
        Serial.println("--------------------HALF GAIT EVEN--------------------");
        #endif
        half_c_prev = millis();
        halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
        half_c = (millis() - half_c_prev);
        full_c = (millis() - full_c_prev);
        #ifdef PLAN_DEBUG
        Serial.print("0.5 odd elapsed time: ");
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
    delay(HALF_TRIPOD_DELAY);
    return true;
}

// lift non-moving legs before push/swing phase
bool MotionPlanner::lift(Leg** l_l){
    Leg* cl;
    int id;
    Vector vm;
    #ifdef PLAN_DEBUG
    Serial.println("---------LIFT----------");
    Serial.print("Leg: ");
    Serial.print('\t');
    Serial.print("X: ");
    Serial.print('\t');
    Serial.print("Y: ");
    Serial.print('\t');
    Serial.println("Z: ");
    #endif
    for(int i=0; i<(NUM_LEGS/2); i++){    
        cl = *(l_l+i); // leg in tripod
        id = cl->getID(); // id for position index in body trajectory 
        vm = b.B_TF_L((b.getLift(id)), id);
        #ifdef PLAN_DEBUG
        Serial.print(id);
        Serial.print('\t');
        Serial.print(vm.getX1());
        Serial.print('\t');
        Serial.print(vm.getX2());
        Serial.print('\t');
        Serial.println(vm.getX3());
        #endif
        b.moveLeg(cl, vm, CART_MOVE, ELBOW_DOWN); // cmd move
    }
    return true;
}

bool MotionPlanner::push(Leg** l_st, Leg** l_sw){
    Vector st_array[NUM_LEGS/2];
    Vector sw_array[NUM_LEGS/2];
    int st_id_t, sw_id_t;
    Leg* st_t;
    Leg* sw_t;
    Leg* st_l_array[NUM_LEGS/2];
    Leg* sw_l_array[NUM_LEGS/2];
    // compute all stances and swings
    for(int i=0; i<(NUM_LEGS/2); i++){
        st_t = (*(l_st + i));
        sw_t = (*(l_sw + i));
        st_id_t = st_t->getID();
        sw_id_t = sw_t->getID();
        st_array[i] = b.B_TF_L(b.getStance(st_id_t), st_id_t); // get all temp swing and stances
        st_l_array[i] = st_t;
        sw_array[i] = b.B_TF_L(b.getSwing(sw_id_t), sw_id_t);
        sw_l_array[i] = sw_t;
    }
    for(int i=0; i<(NUM_LEGS/2); i++){
        b.moveLeg(st_l_array[i], st_array[i], CART_MOVE, ELBOW_DOWN);
        b.moveLeg(sw_l_array[i], sw_array[i], CART_MOVE, ELBOW_DOWN); // cmd move
    }
    #ifdef PLAN_DEBUG
    Serial.println("---------STANCE----------");
    Serial.print("StID: ");
    Serial.print('\t');
    Serial.print("St x :");
    Serial.print('\t');
    Serial.print("St y :");
    Serial.print('\t');
    Serial.print("St z :");
    Serial.print('\t');
    Serial.print("SwID: ");
    Serial.print('\t');
    Serial.print("Sw x :");
    Serial.print('\t');
    Serial.print("Sw y :");
    Serial.print('\t');
    Serial.println("Sw z :");
    #endif

    //b.moveLeg(st_array[0], v_st, CART_MOVE, ELBOW_DOWN);
    //b.moveLeg(sw, v_sw, CART_MOVE, ELBOW_DOWN); // cmd move
    for(int i=0; i<(NUM_LEGS/2); i++){    
        // st = *(l_st+i); // swing and stance leg
        // sw = *(l_sw + i); 
        // st_id = st->getID(); // ids
        // sw_id = sw->getID(); 
        // v_st = b.B_TF_L((b.getStance(st_id)), st_id);
        // v_sw = b.B_TF_L((b.getSwing(sw_id)), sw_id);
        #ifdef PLAN_DEBUG
        Serial.print(st_id);
        Serial.print('\t');
        Serial.print(v_st.getX1());
        Serial.print('\t');
        Serial.print(v_st.getX2());
        Serial.print('\t');
        Serial.print(v_st.getX3());
        Serial.print('\t');
        Serial.print(sw_id);
        Serial.print('\t');
        Serial.print(v_sw.getX1());
        Serial.print('\t');
        Serial.print(v_sw.getX2());
        Serial.print('\t');
        Serial.println(v_sw.getX3());
        #endif
        // delay(10);
        // b.moveLeg(st, v_st, CART_MOVE, ELBOW_DOWN);
        // b.moveLeg(sw, v_sw, CART_MOVE, ELBOW_DOWN); // cmd move
    }
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
    return true;
}