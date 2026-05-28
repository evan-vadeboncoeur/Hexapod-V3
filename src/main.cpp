#include <Arduino.h>
#include "Hexapod.h"

// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?
Servo s = Servo();

//Leg L0 = Leg(1, 2, 3, 0);
// Leg L1 = Leg(4, 5, 6, 1);
// Leg L2 = Leg(7, 8, 9, 2);
// Leg L3 = Leg(10, 11, 12, 3);
// Leg L4 = Leg(13, 14, 15, 4);
// Leg L5 = Leg(16, 17, 18, 5);

//Leg Legs[6] = {L0, L1, L2, L3, L4, L5};
#ifndef IK_DEBUG
MotionPlanner plan = MotionPlanner(Legs);

Hexapod cheeto = Hexapod(plan);
#endif

C_Position pos = C_Position(270.0, -30.0, -80.0);
C_Position test_1 = C_Position(180.0, 0.0, 0.0);
C_Position test_2 = C_Position(-150.0, -221.0, 67.0);

void setup(){
  Serial.begin(9600);
  s.attach(9);
}

void loop() {
  delay(2000);
  s.write(map(135, FOOT_SERVO_MIN, FOOT_SERVO_MAX_INT, PWM_MIN, PWM_MAX));
  //L0.moveToIK(test_2, true);
  #ifdef IK_DEBUG
  delay(2000);
  exit(1);
  #endif
}

