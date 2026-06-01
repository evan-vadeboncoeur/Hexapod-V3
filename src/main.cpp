#include <Arduino.h>
#include "Hexapod.h"

// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?
//Servo s0, s1, s2;
Leg L0, L1, L2, L3, L4, L5;
// test positions
C_Position pos = C_Position(270.0, 80.0, -80.0);
C_Position pos2 = C_Position(180.0, 0.0, 0.0);
C_Position pos3 = C_Position(-150.0, -221.0, 67.0);
J_Position home = J_Position(0.0, 0.0, 0.0);

void setup(){
  Serial.begin(9600); // open before creating legs (at least in testing phases)
  L0 = Leg(LEG_0_J0, LEG_0_J1, LEG_0_J2, LEG_0);
// Leg L1 = Leg(4, 5, 6, LEG_1);
// Leg L2 = Leg(19, 20, 21, LEG_2);
// Leg L3 = Leg(10, 11, 12, LEG_3);
// Leg L4 = Leg(13, 14, 15, LEG_4);
// Leg L5 = Leg(16, 17, 18, LEG_5);

//Leg* Legs[6] = {&L0, &L1, &L2, &L3, &L4, &L5};

//MotionPlanner plan = MotionPlanner(Legs, 0);

//Hexapod cheeto = Hexapod(plan);

}

void loop() {
  delay(2000);
  L0.moveToIK(pos, ELBOW_DOWN, LHS);
  //L0.moveToJV(home);
  //int x = map(21, FOOT_SERVO_MIN, FOOT_SERVO_MAX, PWM_MIN, PWM_MAX);
  //Serial.println(x);
  //#ifdef IK_DEBUG
  
  //s2.write((int)1300);
  //delay(2000);
  //s0.write(90);
  delay(2000);
  exit(1);
  //#endif
}

