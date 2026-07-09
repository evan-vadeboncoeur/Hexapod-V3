#include "Hexapod.h"

Hexapod::Hexapod(int g, float df, float tc, float sh)
    : plan{MotionPlanner(g, df, tc, sh)},
    radio{CommunicationManager()},
    board{BoardManager()}
{

}

void Hexapod::startupHexapod(){
    radio.commBegin();
    plan.powerOnSequence();
    power_on = false;
    state = WAITING;
}

void Hexapod::shutdownHexapod(){
    // ---?
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
    // get new packet
    command = radio.getPacket();
    // fill out new packet
    gait_old = gait;
    gait = command.g;
    power_off = command.rb;
    power_on = command.lb; 
    twist = command.t;
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
    while(!power_off){
        if(radio.receivePacket()) processPacket();// this would be on some type of interrupt as well
            
        switch(state){
            case WAITING:
                if(power_on) startupHexapod();
            break;
            
            case WALKING: // may need {} for setting values in switch statement
                walk();
            break;
            
            case TURNING:
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

