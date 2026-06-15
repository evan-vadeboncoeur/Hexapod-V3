#include "Vector.h"

// simple vector class for up to three inputs
// stores magnitude and coordinate data
// used for joint vectors, and position vectors, speed vectors, and probably twist vector
Vector::Vector(){
    x1 = 0.0, x2=0.0, x3=0.0;
}

Vector::Vector(float xv1, float xv2, float xv3){
    x1 = xv1;
    x2 = xv2;
    x3 = xv3;
    m = sqrt(x1*x1 + x2*x2 + x3*x3);
    t = atan2(x2,x1); // give angle between x and y components
}

Vector::Vector(float xv1, float xv2){
    x1 = xv1;
    x2 = xv2;
    x3 = 0.0;
    m = sqrt(x1*x1 + x2*x2 + x3*x3);
    t = atan2(x2,x1); // give angle between x and y components
}

Vector Vector::cross(Vector a, Vector b){
    return Vector(a.getX2()*b.getX3() - a.getX3()*b.getX2(), -(a.getX1()*b.getX3() - a.getX3()-b.getX1()), a.getX1()*b.getX2() - a.getX2()*b.getX1());
}