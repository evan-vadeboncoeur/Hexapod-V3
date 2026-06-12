#include "Body.h"

Body::Body(){

}

Body::Body(Leg l[NUM_LEGS]) // initialize each leg before contructing the rest of the object
    : legs{
        Leg(LEG_0, LEG_0_J0, LEG_0_J1, LEG_0_J2, CCW_CONFIG),
        Leg(LEG_1, LEG_1_J0, LEG_1_J1, LEG_1_J2, CCW_CONFIG),
        Leg(LEG_2, LEG_2_J0, LEG_2_J1, LEG_2_J2, CCW_CONFIG),
        Leg(LEG_3, LEG_3_J0, LEG_3_J1, LEG_3_J2, CCW_CONFIG),
        Leg(LEG_4, LEG_4_J0, LEG_4_J1, LEG_4_J2, CCW_CONFIG),
        Leg(LEG_5, LEG_5_J0, LEG_5_J1, LEG_5_J2, CCW_CONFIG)
    } 
{
    computeAlphaI();
}

void Body::unpackTwist(){
    body_velocity = Vector(twist.getX1(), twist.getX1());
    bv = body_velocity.getMagnitude();
    theta = body_velocity.getTheta();
    vx = bv*cos(theta);
    vy = bv*sin(theta);
}

// compute rotation angles for RBTF between body and leg frame
void Body::computeAlphaI(){
    for(int i=0; i<(NUM_LEGS-1); i++){
        alpha_i[i] = M_PI_3*i + M_PI_6;
    }
}