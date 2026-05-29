#include "MotionPlanner.h"

MotionPlanner::MotionPlanner(){

}

MotionPlanner::MotionPlanner(Leg** l, int g){
    legs = l; // pointer to array of legs
    setGait(g);
    // tripod setup code...
    // for(int i =0, j=0; i<2; i++, j++){
    //     tp_1[i];
    // }
}

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
}
void MotionPlanner::setTargetSteps(int s){
    steps = s;
}

void MotionPlanner::setupTripod(int steps, int dir, int s_inc){
    setDirection(dir);
    setTargetSteps(steps);
    setDistanceIncrement(s_inc);
}

// sorts tripod based on chosen direction into left and right "halves"
// divides legs into two groups split along the edge of the chosen direction
void MotionPlanner::sortTripod(){
    for(int i=0,j=2,k=5; i<3; i++,j+=2, k+=2){
        tp_1[i] = legs[(direction + j) % NUM_LEGS];
        tp_2[i] = legs[(direction + k) % NUM_LEGS];
    }
    // negative is always 3 left CW of direction, positive is always 3 right CCW of direction
    for(int i=0; i<5; i++){
        if(i<3)legs[(direction + i) % NUM_LEGS]->setDirection(-1.0); // 3 CCW of leg are negative
        else legs[(direction + i) % NUM_LEGS]->setDirection(1.0); // 3 CW of leg are positive
    }
}

bool MotionPlanner::tripodGait(int steps, int dir, int s_inc){
    setupTripod(steps, dir, s_inc);
    return true;

}