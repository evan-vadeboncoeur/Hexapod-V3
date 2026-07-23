#include "MotionPlanner.h"
// CONSTRUCTORS
MotionPlanner::MotionPlanner(){
}

MotionPlanner::MotionPlanner(uint8_t g, float df, float tc, float sh)
    : b{Body(df, tc, sh)} // initialize body first (set alpha, idle, gait cycle values), set tripods too in body constructor
{
    delay(5);
    tripods[0] = b.getTripod(TP_EVEN);
    tripods[1] = b.getTripod(TP_ODD);
    setGait(g, NWALK); 
    t_c_m = tc * 1000; // make an int for millis comparison
}

// GAIT SETUP
void MotionPlanner::movementSetup(int g, Vector tw){
    setBodyVelocity(tw); // compute trajectories
    // set state times
    setGait(g, NWALK); // calls desired gait, start walking
    stateTime();
}

// returns movement time between two angles of a single servo
uint32_t MotionPlanner::moveTime(float t1, float t2){
    uint32_t calc = (uint32_t)(((abs(t2-t1) / OMEGA) * 1000 + MVMT_BFR)*OMEGA_C);
    #ifdef STATE_TIME_DEBUG
    Serial.print("t2");
    Serial.print('\t');
    Serial.print("t1");
    Serial.print('\t');
    Serial.println("time");
    Serial.print(t2);
    Serial.print('\t');
    Serial.print(t1);
    Serial.print('\t');
    Serial.println(calc);
    #endif
    return calc;
}

// parses through joint vectors and determines limiting motions for state, sets the time value for that state
// TODO: additional functionality for other gaits (body class may need to be limited as well?)
void MotionPlanner::stateTime(){
    stance_lift=0, lift_swing=0, swing_stance=0;
    for(int i=0; i<NUM_LEGS; i++){
        uint32_t s_l, l_s, s_s;
        // Stance -> Lift
        s_l = moveTime(b.getStanceJ(i).getX1(), b.getLiftJ(i).getX1());
        if (s_l > stance_lift) stance_lift = s_l;
        s_l = moveTime(b.getStanceJ(i).getX2(), b.getLiftJ(i).getX2());
        if (s_l > stance_lift) stance_lift = s_l;
        s_l = moveTime(b.getStanceJ(i).getX3(), b.getLiftJ(i).getX3());
        if (s_l > stance_lift) stance_lift = s_l;
        // Lift -> Swing
        l_s = moveTime(b.getSwingJ(i).getX1(), b.getLiftJ(i).getX1());
        if (l_s > lift_swing) lift_swing = l_s;
        l_s = moveTime(b.getSwingJ(i).getX2(), b.getLiftJ(i).getX2());
        if (l_s > lift_swing) lift_swing = l_s;
        l_s = moveTime(b.getSwingJ(i).getX3(), b.getLiftJ(i).getX3());
        if (l_s > lift_swing) lift_swing = l_s;
        // Swing -> Stance
        s_s = moveTime(b.getSwingJ(i).getX1(), b.getStanceJ(i).getX1());
        if (s_s > swing_stance) swing_stance = s_s;
        s_s = moveTime(b.getSwingJ(i).getX2(), b.getStanceJ(i).getX3());
        if (s_s > swing_stance) swing_stance = s_s;
        s_s = moveTime(b.getSwingJ(i).getX2(), b.getStanceJ(i).getX3());
        if (s_s > swing_stance) swing_stance = s_s;
    }
    #ifdef PLAN_DEBUG
    Serial.println("---------Tripod Gait State Time----------");
    Serial.print("St-L");
    Serial.print('\t');
    Serial.print("L-Sw");
    Serial.print('\t');
    Serial.println("Sw-St");
    Serial.print(stance_lift);
    Serial.print('\t');
    Serial.print(lift_swing);
    Serial.print('\t');
    Serial.println(swing_stance);
    #endif
    gait_update_time = millis() + stance_lift;
}

void MotionPlanner::setBodyVelocity(Vector tw){
    b.velocityCommand(tw); // set target twist value, foot velocities, position changes, etc.
}

void MotionPlanner::setGait(int g, bool walk){
    switch(g){
        case TRIPOD: 
            gait = TRIPOD;
            if(walk) tripodGait(2); // test version
            break;
        case RIPPLE: 
            gait = RIPPLE;
            //#ifdef PLAN_DEBUG
            if(walk) rippleGait(2);
            //#endif
            break;
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
        //tripodGait();
        updateTripod();
        break;
    case RIPPLE: 
        rippleGait(1);
        break;
    case WAVE: 
        waveGait();
        break;
    case QUADRUPED: 

        break;
    default:
        tripodGait();
        break;
    }
}

void MotionPlanner::turn(){
    setBodyVelocity(omega);
    switch(gait){
    case TRIPOD: 
        tripodGait();
        break;
    case RIPPLE: 
        rippleGait(1);
        break;
    case WAVE: 
        waveGait();
        break;
    case QUADRUPED: 
        break;
    default:
        tripodGait();
        break;
    }
}

// GAITS ----------------------------------------------------------------------------------
void MotionPlanner::updateTripod(){ // can work with ripple and wave as well?. may need code to verify if at end of half-cycle (flag)
    if(millis() < gait_update_time) return;
  
    switch(tp_stance){
        case LIFT:
            lift(tripods[!tp_index]);
            tp_stance = PUSH;
            gait_update_time = millis() + stance_lift;
        break;
        case PUSH:
            push(tripods[tp_index], tripods[!tp_index]);
            tp_stance = LIFT;
            gait_update_time = millis() + swing_stance;
            tp_index = !tp_index;
        break;
    }
}


// teleop, indefinite version
bool MotionPlanner::tripodGait(){
    #ifdef PLAN_DEBUG
    Serial.println("**********************TRIPOD GAIT***********************");
    #endif
    delay(HALF_TRIPOD_DELAY);

    #ifdef PLAN_DEBUG
    Serial.println("--------------------STANCE EVEN--------------------");
    #endif
    halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
    delay(HALF_TRIPOD_DELAY);

    #ifdef PLAN_DEBUG
    Serial.println("--------------------STANCE ODD--------------------");
    #endif
    halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
    
    
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
        half_c_prev = full_c_prev = millis();
        #endif
        delay(HALF_TRIPOD_DELAY);
        halfTripod(b.getTripod(TP_ODD), b.getTripod(TP_EVEN));
        delay(HALF_TRIPOD_DELAY);

        #ifdef PLAN_DEBUG
        half_c = (millis() - half_c_prev);
        Serial.print("1/2 gait even elapsed time: ");
        Serial.print('\t');
        Serial.println(((float)(half_c)/1000.0));
        Serial.println("--------------------HALF GAIT EVEN--------------------");
        half_c_prev = millis();
        #endif
        halfTripod(b.getTripod(TP_EVEN), b.getTripod(TP_ODD));
        #ifdef PLAN_DEBUG
        half_c = (millis() - half_c_prev);
        full_c = (millis() - full_c_prev);
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
    even_forward = ((*(sw))->getID() == 0) ? true : false;
    return true;
}

// lift non-moving legs before push/swing phase
bool MotionPlanner::lift(Leg** l_l){
    Leg* t_l; // temp leg for ids
    int t_id; // temp id
    Vector F_lift;
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

        F_lift = b.B_TF_L(b.getLift(t_id), t_id); // could remove this and one beneath it from call stack by computing this at init time in body class, then could have all joint vectors stored in body/mp for timing calculations
        J_lift[i] = b.computeIK(t_l, F_lift, ELBOW_DOWN);
        
        #ifdef PLAN_DEBUG
        Serial.print(t_id); // leg
        Serial.print('\t');
        Serial.print((b.getLift(t_id)).getX1()); // body coord
        Serial.print('\t');
        Serial.print((b.getLift(t_id)).getX2());
        Serial.print('\t');
        Serial.print((b.getLift(t_id)).getX3());
        Serial.print('\t');
        Serial.print(F_lift.getX1()); // foot coord
        Serial.print('\t');
        Serial.print(F_lift.getX2());
        Serial.print('\t');
        Serial.print(F_lift.getX3());
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
   
    Vector F_stance; // lifts in foot frames
    Vector F_swing; // lifts in foot frames

 
    Vector J_stance[NUM_LEGS/2]; //lifts in joint space
    Vector J_swing[NUM_LEGS/2]; //lifts in joint space
    #ifdef MEMORY_DEBUG
    Serial.println(freeMemory());
    #endif
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
        F_stance = b.B_TF_L(b.getStance(t_st_id), t_st_id);
        F_swing = b.B_TF_L(b.getSwing(t_sw_id), t_sw_id);
        J_stance[i] = b.computeIK(t_st_l, F_stance, ELBOW_DOWN);
        J_swing[i] = b.computeIK(t_sw_l, F_swing, ELBOW_DOWN);

        #ifdef PLAN_DEBUG
        Serial.print(t_st_id); // leg
        Serial.print('\t');
        Serial.print((b.getStance(t_st_id)).getX1()); // body coord
        Serial.print('\t');
        Serial.print((b.getStance(t_st_id)).getX2());
        Serial.print('\t');
        Serial.print((b.getStance(t_st_id)).getX3());
        Serial.print('\t');
        Serial.print(F_stance.getX1()); // foot coord
        Serial.print('\t');
        Serial.print(F_stance.getX2());
        Serial.print('\t');
        Serial.print(F_stance.getX3());
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
        Serial.print(F_swing.getX1()); // foot coord
        Serial.print('\t');
        Serial.print(F_swing.getX2());
        Serial.print('\t');
        Serial.print(F_swing.getX3());
        Serial.print('\t');
        Serial.print(J_swing[i].getX1()); // joint coord
        Serial.print('\t');
        Serial.print(J_swing[i].getX2());
        Serial.print('\t');
        Serial.println(J_swing[i].getX3());
        #endif
        
    }
    // move at same time    
    b.moveTripod(l_sw, J_swing);
    b.moveTripod(l_st, J_stance);
    delay(MOTION_PLANNER_DELAY);
    return true;
}

bool MotionPlanner::waveGait(){
    
    
    
    return true;
}

bool MotionPlanner::rippleGait(int cc){
    // tripods front to back, alternate tripods:
    // Legs CCW about hexagon circle, X-axis aligned between legs 0 and 5. Legs: 0, 1, 2, 3, 4, 5
    // 0, 4, 2 -> 5, 1, 3
    // at each foot: 1 leg in swing, rest in stance
    // ||swing|| == ||stance||, all stance legs move 1/6 the amount of swing leg during that particular swing leg's motion
    // sort tripods in ripple gait manner ... ? change tripod pre-existing?
    // pretty much all setBodyVelocity commands with calculated vectors should be identical for the setup.. will need some wizardry for indexes though
    static int ripple_ct=0; // life of program
    int num_cycle=0;
    while(num_cycle < cc){
        float s = (float)(millis() % t_c_m) / t_c_m;
        // gait scheduler code
        if((s >= (float (ripple_ct*((float)1/6)))) && (s < (float ((ripple_ct+1)*((float)1/6))))){
            ripplePush(ripple_ct);
            ripple_ct++;
            ripple_ct %= NUM_LEGS; // wraparound 6 to 0
            if(ripple_ct == 0) num_cycle++; // full cycle?
        }
    }
    
    return true;
}

bool MotionPlanner::ripplePush(int rc){
    // for each push:
        // calculate body frame coordinates along trajectory (stance + swing*s, s = 1/6*i) BUT also need to assign stance at different times along the cycle...
        // calculate foot coordinates in leg frame
        // calculate joint vectors of push
    // for swing:
        // calculate body frame coordinates of swing position
        // calculate foot coordinate in leg frame
        // calculate joint vector
    // move function that simultaneously moves all joints to calculated positions
    // Challenges: gait startup, gait pause, memory usage
    // setup variables
    float sf;
    Leg* t_l; // temp leg for ids
    int t_id;
    Vector B_st, B_sw, B_t;
    Vector F_st_sw; // no debug, optimize
    Vector J_st_sw[NUM_LEGS]; //stance OR swing in joint space
    // sort into swing (1) and push (5) -> dont need to if we just use if() in for loop for calculations
    Leg** all_legs = b.getLegList();
    #ifdef PLAN_DEBUG
    Serial.println("---------Ripple ST/SW Coordinates----------");
    Serial.print("RC: ");
    Serial.print('\t');
    Serial.print("Leg: ");
    Serial.print('\t');
    Serial.print("S");
    Serial.print('\t');
    Serial.print("SW/ST: ");
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
    for(int i=0; i<NUM_LEGS; i++){
        float sw_z;
        //int drc = rc - i;
        float phase = 5.0;
        t_l = *(all_legs+i);
        t_id = t_l->getID();
        // ripple v2
        int d = (i - rc + NUM_LEGS) % NUM_LEGS; // THIS IS THE EXPRESSION I WAS LOOKING FOR, didnt need a +/- 1, uses whole num legs to get "abs" value
        if(d == 0){
            sf = 0.0;
        } else{
            sf = ((phase + 1.0) - (float(d))) / phase; // find phase / 5, while 0 = 0, inside of 1-6 cycle
        }
        // find magnitude away from rc leg (but make note of direction)
        //sf = (abs(rc - i)*((float)1/6)); // change to FIFTHS
        //sf = ((i - rc) <=  0) ? sf : 1.0 - sf;
        //sf = (drc == 0) ? 0.0 : ((drc < 0) ? (1.0 - ((abs(drc)-1.0)/5.0)) : (abs(drc))/5.0); // has to be an easier way than this?
        //sf = 1.0 - (rc - i)*((float)1/5);
        // assuming linear translation, it *should* be ok to chunk everything into linear lines in 1/6 increments of the gait- ...
        // get bounds of leg in body frame
        B_st = b.getStance(t_id);
        B_sw = b.getSwing(t_id);
        // modify linear coordinate from SW->stance (may need actual linear interpolation helper function)
        // need to "dot product" in direction of motion
        // x = x_swing - sf*(x_swing - x_stance)*cos(theta);
        B_t.setX1(B_sw.getX1() - ((B_sw.getX1() - B_st.getX1())*(sf)));
        B_t.setX2(B_sw.getX2() - ((B_sw.getX2() - B_st.getX2())*(sf)));
        sw_z = (sf < 0.12) ? b.getLiftHeight() : B_sw.getX3();
        B_t.setX3(sw_z); // Z is constant
        F_st_sw = b.B_TF_L(B_t, t_id);
        J_st_sw[i] = b.computeIK(t_l, F_st_sw, ELBOW_DOWN);    
        // printout
        #ifdef PLAN_DEBUG
        Serial.print(rc);
        Serial.print('\t');
        Serial.print(t_id); // leg
        Serial.print('\t');
        Serial.print(sf);
        Serial.print('\t');
        if(sf < 0.15) Serial.print("SW");
        else Serial.print("ST");
        Serial.print('\t');
        Serial.print(B_t.getX1()); // body coordinate
        Serial.print('\t');
        Serial.print(B_t.getX2());
        Serial.print('\t');
        Serial.print(B_t.getX3());
        Serial.print('\t');
        Serial.print(F_st_sw.getX1()); // foot cood
        Serial.print('\t');
        Serial.print(F_st_sw.getX2());
        Serial.print('\t');
        Serial.print(F_st_sw.getX3());
        Serial.print('\t');
        Serial.print(J_st_sw[i].getX1()); // joint coord
        Serial.print('\t');
        Serial.print(J_st_sw[i].getX2());
        Serial.print('\t');
        Serial.println(J_st_sw[i].getX3());
        #endif
    }
    // call push all legs function
    // need to add a lift intermediate for swing leg?
    b.moveLegs(J_st_sw);
    return true;
}

// MOVEMENT MACROS -------------------------------------------------------------------------------------
bool MotionPlanner::powerOnSequence(){
    #ifdef PLAN_DEBUG
    Serial.println("**********************Powering On***********************");
    #endif
    moveHome();
    delay(MACRO_DELAY);
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
    //moveHome();
    //delay(100);
    moveStorage();
    delay(1000);
    return true;
}