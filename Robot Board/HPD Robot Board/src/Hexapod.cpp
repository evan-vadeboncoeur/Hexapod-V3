#include "Hexapod.h"

Hexapod::Hexapod(int g, float df, float tc, float sh)
    : plan{MotionPlanner(g, df, tc, sh)},
    radio{CommunicationManager()},
    board{BoardManager()}
{

}

void Hexapod::startupHexapod(){
    #ifdef HEXAPOD_DEBUG
    Serial.println("**********************Startup Hexapod***********************");
    #endif
    radio.commBegin();
    plan.powerOnSequence();
    power_on = false;
    powered_on = true;
    state = WAITING;
}

void Hexapod::shutdownHexapod(){
    #ifdef HEXAPOD_DEBUG
    Serial.println("**********************Startup Hexapod***********************");
    #endif
    plan.powerOffSequence();
    state = WAITING;
}

void Hexapod::gaitSetup(){
    plan.movementSetup(gait, twist);
}

void Hexapod::gaitShutdown(){
    plan.moveIdle();
    state = WAITING;
}

void Hexapod::walk(){
    plan.walk();
}

void Hexapod::turn(){
    plan.turn();
}

// NEED:
// 1) way to ensure the new twist vector is different than old (done in RC class)
// 2) way to recieve change in walk commands inside the motion planner class...?
//      new command
//      setupWalk
//      walk for cycle return true
// check new command
// if new command recompute gait, else walk one cycle??
// 3) way to go to/from walking-> 0 speed -> walking

void Hexapod::processPacket(){
    #ifdef HEXAPOD_DEBUG
    Serial.println("--------------------NEW COMMAND--------------------");
    #endif
    // get new packet
    
    command = radio.getPacket();
    // fill out new packet
    gait_old = gait;
    gait = command.g;
    power_off = command.rb;
    power_on = command.lb; 
    twist = command.t;
    #ifdef HEXAPOD_DEBUG
    Serial.print("Gait: ");
    Serial.print('\t');
    Serial.print("PowOff: ");
    Serial.print('\t');
    Serial.print("PowOn: ");
    Serial.print('\t');
    Serial.print("Tw Vx: ");
    Serial.print('\t');
    Serial.print("Tw Vy: ");
    Serial.print('\t');
    Serial.println("Tw Wz: ");
    Serial.print(gait);
    Serial.print('\t');
    //int p_off = ((power_off) ? 1 : 0);
    //int p_on = ((power_on) ? 1 : 0);
    Serial.print(power_off);
    Serial.print('\t');
    Serial.print(power_on);
    Serial.print('\t');
    Serial.print(twist.getX1());
    Serial.print('\t');
    Serial.print(twist.getX2());
    Serial.print('\t');
    Serial.println(twist.getX3());
    #endif
    // check all types of non-gait update commands
    if((!power_off) && (!power_on) && (powered_on)){ // no macro, no turning
        if(twist.getMagnitude() > 0.0){ // non-zero twist command update
            state = WALKING;
            gaitSetup();
        } else{ // new command is 0 velocity, bring the robot to idle, but dont shutdown
            gaitShutdown(); 
        }
    }
    else if((power_off) && (!power_on)) state = POWER_OFF; // only p off macro
    else if((!power_off) && (power_on)) state = POWER_ON; // only p on macro
    else if((power_off) && (power_on) && (powered_on)) state = TURNING; // both = turning mode (CCW) (MAY BE DIFFICULT GETTING BOTH AT SAME TIME)
    
}

void Hexapod::stateManager(){
    #ifdef HEXAPOD_DEBUG
    Serial.println("**********************State Manager***********************");
    #endif
    while(!power_off){
        if(radio.receivePacket()) processPacket();// this would be on some type of interrupt as well
        // Hexapod state machine 
        switch(state){
            case WAITING: // do nothing... maybe add in a blink for "NRF LED"
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------WAITING--------------------");
                #endif
            break;
            
            case WALKING: // 1 walk cycle at current speed
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------WALKING--------------------");
                #endif
                walk(); 
            break;
            
            case TURNING: // 1 turn cycle at current speed
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------TURNING--------------------");
                #endif
                turn(); // one turn cycle at current speed
            break;

            case POWER_ON: // power on macro
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------POWERING ON--------------------");
                #endif    
                startupHexapod();
            break;

            case POWER_OFF: // power off macro
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------POWERING OFF--------------------");
                #endif    
                shutdownHexapod();
            break;
        }
        checkBattery(); // this would  be on a timer interrupt delay
        delay(HEXAPOD_LOOP_DELAY); // make smaller
    }
}

void Hexapod::checkBattery(){
    board.checkBatteries();
}

