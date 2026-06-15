#ifndef VECTOR_H
#define VECTOR_H
#include <math.h>

class Vector{
    public:
        Vector();
        Vector(float xv1, float xv2, float xv3);
        Vector(float xv1, float xv2);
        float getX1(){return x1;}
        float getX2(){return x2;}
        float getX3(){return x3;}
        float setX1(float x_1){x1 = x_1;}
        float setX2(float x_2){x2 = x_2;}
        float setX3(float x_3){x3 = x_3;}
        float getMagnitude(){return m;}
        float getTheta(){return t;}
        Vector cross(Vector a, Vector b);
    private:
        float x1, x2, x3, m, t;
};

#endif