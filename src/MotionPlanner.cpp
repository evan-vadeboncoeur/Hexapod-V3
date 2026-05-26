#include "MotionPlanner.h"

MotionPlanner::MotionPlanner(){

}

MotionPlanner::MotionPlanner(Leg* legs){
    this->legs = legs;
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
    dir = d;
}
void MotionPlanner::setTargetSteps(int s){
    steps = s;
}

void MotionPlanner::setupTripod(int steps, int dir, int s_inc){
    setDirection(dir);
    setTargetSteps(steps);
    setDistanceIncrement(s_inc);
}

bool MotionPlanner::tripodGait(int steps, int dir, int s_inc){
    setupTripod(steps, dir, s_inc);
    return true;

}