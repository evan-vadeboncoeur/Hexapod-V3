#include <Arduino.h>
#include "Hexapod.h"

// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?

Leg L0 = Leg(1, 2, 3, 0);
Leg L1 = Leg(4, 5, 6, 1);
Leg L2 = Leg(7, 8, 9, 2);
Leg L3 = Leg(10, 11, 12, 3);
Leg L4 = Leg(13, 14, 15, 4);
Leg L5 = Leg(16, 17, 18, 5);

Leg Legs[6] = {L0, L1, L2, L3, L4, L5};

MotionPlanner plan = MotionPlanner(Legs);

Hexapod chester = Hexapod(plan);


void setup(){

}

void loop() {
  // put your main code here, to run repeatedly:
}

