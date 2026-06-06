#include <Arduino.h>
#include "Hexapod.h"
//#define LEG_SETUP_DEBUG

// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?
#ifdef LEG_SETUP_DEBUG
Servo s0, s1, s2;
int j0 = LEG_0_J0;
int j1 = LEG_0_J1;
int j2 = LEG_0_J2;
#endif
Leg L0, L1, L2, L3, L4, L5;

Leg* legs[NUM_LEGS];
// test positions
C_Position pos = C_Position(270.0, 80.0, -80.0);
C_Position pos2 = C_Position(180.0, 0.0, 0.0);
C_Position pos3 = C_Position(-150.0, -221.0, 67.0);
J_Position home = J_Position(0.0, 0.0, 0.0);

MotionPlanner plan = MotionPlanner();

void setup(){
  Serial.begin(9600); // open before creating legs (at least in testing phases)
  delay(1000);

  #ifdef GLOBAL_DEBUG
  Serial.println("In setup");
  #endif

  #ifndef LEG_SETUP_DEBUG
  L0 = Leg(LEG_0_J0, LEG_0_J1, LEG_0_J2, LEG_0);
  L1 = Leg(LEG_1_J0, LEG_1_J1, LEG_1_J2, LEG_1);
  L2 = Leg(LEG_2_J0, LEG_2_J1, LEG_2_J2, LEG_2);
  L3 = Leg(LEG_3_J0, LEG_3_J1, LEG_3_J2, LEG_3);
  L4 = Leg(LEG_4_J0, LEG_4_J1, LEG_4_J2, LEG_4);
  L5 = Leg(LEG_5_J0, LEG_5_J1, LEG_5_J2, LEG_5);

  legs[LEG_0] = &L0;
  legs[LEG_1] = &L1;
  legs[LEG_2] = &L2;
  legs[LEG_3] = &L3;
  legs[LEG_4] = &L4;
  legs[LEG_5] = &L5;
  //*(legs + LEG_0) = &L0; (equivalent to the above)

  plan = MotionPlanner(legs, 0);
  #endif
  #ifdef LEG_SETUP_DEBUG
  s0.attach(j0, PWM_MIN, PWM_MAX);
  s1.attach(j1, PWM_MIN, PWM_MAX);
  s2.attach(j2, PWM_MIN, PWM_MAX);
  #endif
//Hexapod cheeto = Hexapod(plan);
  plan.powerOnSequence();
}

void loop() {
  #ifdef GLOBAL_DEBUG
  Serial.println("In loop");
  #endif
  delay(2000);
  #ifdef LEG_SETUP_DEBUG
  s0.write(HPS_2018_CTR);
  delay(2000);
  s1.write(HPS_2018_CTR);
  delay(2000);
  s2.write(HPS_2027_CTR); // avg is middle, not 1000us
  delay(2000);
  #endif
  #ifdef PLAN_DEBUG
  //plan.setupTripod(10, 1, 8);
  //plan.tripodGait(4, 1, 8);
  // plan.moveStorage();
  // delay(3000);
  // plan.moveHome();
  // delay(3000);
  // plan.moveStance();
  //L0.moveToJV(home);  
  plan.powerOffSequence();
  #endif
  delay(2000);
  exit(1);
  
}

