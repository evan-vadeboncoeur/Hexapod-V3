#include <Arduino.h>
#include "Hexapod.h"

// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?
//Servo s0, s1, s2;

Leg L0 = Leg(LEG_0_J0, LEG_0_J1, LEG_0_J2, LEG_0);
// Leg L1 = Leg(4, 5, 6, LEG_1);
// Leg L2 = Leg(7, 8, 9, LEG_2);
// Leg L3 = Leg(10, 11, 12, LEG_3);
// Leg L4 = Leg(13, 14, 15, LEG_4);
// Leg L5 = Leg(16, 17, 18, LEG_5);

//Leg Legs[6] = {L0, L1, L2, L3, L4, L5};
#ifndef IK_DEBUG
//MotionPlanner plan = MotionPlanner(Legs);

//Hexapod cheeto = Hexapod(plan);
#endif

C_Position pos = C_Position(270.0, -30.0, 80.0);
C_Position test_1 = C_Position(180.0, 0.0, 0.0);
C_Position test_2 = C_Position(-150.0, -221.0, 67.0);
J_Position srvo_tst = J_Position(3.1415/2, 3.1415/2, 2.0);

void setup(){
  Serial.begin(9600);
  // pinMode(7, OUTPUT);
  // pinMode(8, OUTPUT);
  // pinMode(9, OUTPUT);
  //  s0.attach(7);
  //  s1.attach(8);
  //  s2.attach(9);
}

void loop() {
  delay(2000);

  // s0.write((int)83.66);
  // delay(1000);
  // s1.write((int)105.55);
  // delay(1000);
  // s2.write((int)1944.00);
  //L0.moveToIK(pos, ELBOW_DOWN);
  L0.moveToJV(srvo_tst);
  //int x = map(21, FOOT_SERVO_MIN, FOOT_SERVO_MAX, PWM_MIN, PWM_MAX);
  //Serial.println(x);
  //#ifdef IK_DEBUG
  delay(1000);
  exit(1);
  //#endif
}

