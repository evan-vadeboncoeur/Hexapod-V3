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
}

void Hexapod::shutdownHexapod(){
    // ---?
    plan.powerOffSequence();
}

void Hexapod::gaitSetup(){
    plan.movement(gait, twist);
}

void Hexapod::processPacket(){
    command = radio.getPacket();
    gait_old = gait;
    gait = command.g;
    power_off = command.rb;
    power_on = command.lb; 
    twist = command.t;
    if(gait != gait_old || (!power_off) || (!power_on)) state = WALKING;
}

void Hexapod::stateManager(){
    while(!power_off){
        if(radio.receivePacket()) processPacket();
            
        switch(state){
            case WAITING:
                if(power_on) startupHexapod();
                power_on != power_on;
            break;
            
            case WALKING: // may need {} for setting values in switch statement
                gaitSetup(); // NEED TO ADD SOME WAY FOR COMMAND CHANGE IN MOTION PLANNER CLASS....
                state = WAITING;
            break;
            
            case TURNING:
                state = WAITING;
            break;
        }
    
        delay(HEXAPOD_LOOP_DELAY);
    }
    shutdownHexapod();
}

void Hexapod::gaitSet(){
    ;
}


