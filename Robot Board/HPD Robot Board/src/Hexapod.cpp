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
    if((!power_off) && (!power_on)){ // fix this... needs to be a new twist (should be from RC class, but need to verify)
        if(twist.getMagnitude() > 0.0){ // non-zero twist command
            state = WALKING;
            gaitSetup();
        } else{ // new command is 0 velocity, bring the robot to idle, but dont shutdown
            gaitShutdown(); 
        }
    }
    checkBattery(); // this would  be on a timer interrupt delay
}

// while not given command to shutdown
// check for new packet
//  if new packet, process packet
//      if there is a new gait, and we arent turning off or shuttong on, check magnitude of twist
//          if twist > 0, init walking and walk
//          if twist is 0, initializing waiting and move to idle

void Hexapod::stateManager(){
    #ifdef HEXAPOD_DEBUG
    Serial.println("**********************State Manager***********************");
    #endif
    while(!power_off){
        if(radio.receivePacket()) processPacket();// this would be on some type of interrupt as well
            
        switch(state){
            case WAITING:
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------WAITING--------------------");
                #endif
                if(power_on) startupHexapod();
            break;
            
            case WALKING: // may need {} for setting values in switch statement
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------WALKING--------------------");
                #endif
                walk();
            break;
            
            case TURNING:
                #ifdef HEXAPOD_DEBUG
                Serial.println("--------------------TURNING--------------------");
                #endif
                state = WAITING;
            break;
        }
    
        delay(HEXAPOD_LOOP_DELAY);
    }
    shutdownHexapod();
}

void Hexapod::checkBattery(){
    board.checkBatteries();
}

