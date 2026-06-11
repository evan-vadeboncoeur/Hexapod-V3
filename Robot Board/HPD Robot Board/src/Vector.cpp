#include "Vector.h"

Vector::Vector(float xv, float yv, float zv){
    x = xv;
    y = yv;
    z = zv;
    m = sqrt(x*x + y*y + z*z);
    //t = atan2(y,x); // check for angle sense...
}

Vector::Vector(float xv, float yv){
    x = xv;
    y = yv;
    z = 0.0;
    m = sqrt(x*x + y*y + z*z);
}