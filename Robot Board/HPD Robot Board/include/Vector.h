#ifndef VECTOR_H
#define VECTOR_H
#include <math.h>

class Vector{
    public:
        Vector();
        Vector(float xv, float yv, float zv);
        Vector(float xv, float yv);
        float getX(){return x;}
        float getY(){return y;}
        float getZ(){return z;}
        float getMagnitude(){return m;}
        float getTheta(){return t;}
    private:
        float x, y, z, m, t;
};

#endif