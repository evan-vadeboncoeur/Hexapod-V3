#ifndef TRIPOD_H
#define TRIPOD_H

#include "Leg.h"

class Tripod {
    private:
        Leg legs[3];
        int ids[3];
        char direction; // walking direction dependent on leg physical location
    public:
        Tripod(Leg leg_c[]);
};

#endif