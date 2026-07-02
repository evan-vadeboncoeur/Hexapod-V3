#include <Arduino.h>
#include "Body.h"

//#include "Hexapod.h"
#define LEG_SETUP_DEBUG

// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?
#ifdef LEG_SETUP_DEBUG
Servo s0, s1, s2;
int j0 = LEG_0_J0;
int j1 = LEG_0_J1;
int j2 = LEG_0_J2;
#endif

float duty_f = 0.5;
float cycle_time = 1.5;

#define LEG_KINEMATICS_DEBUG
#ifdef LEG_KINEMATICS_DEBUG
Leg leg_test = Leg(LEG_0, j0, j1, j2, CCW_CONFIG);
Vector home_j = Vector(0.0, 0.0, 0.0);
Vector storage_j = Vector(0.0, -1.216, -1.763);
Vector idle_j = Vector(0.0, -0.977, 2.146);
Vector home_p = Vector(310.0, 0.0, 0.0);
Vector storage_p = Vector(-46.76, 0.0, 108.68); // CAD output
Vector idle_p = Vector(178.755, 0.0, 63.631); // CAD output
#endif

//MotionPlanner plan = MotionPlanner();
//CommunicationManager cm = CommunicationManager(CE_H, CSN_H);

void setup(){
  Serial.begin(9600); // open before creating legs (at least in testing phases)
  delay(1000);
  //cm.commBegin();
  delay(50);
  Serial.println("In setup");
  #ifdef GLOBAL_DEBUG
  Serial.println("In setup");
  #endif

  #ifndef LEG_SETUP_DEBUG
  Body b = Body(duty_f, cycle_time);
  b.velocityCommand(Vector(10.0, 0.0, 0.05));
  delay(1000);
  //exit(1);
  // legs[LEG_0] = &L0;
  // legs[LEG_1] = &L1;
  // legs[LEG_2] = &L2;
  // legs[LEG_3] = &L3;
  // legs[LEG_4] = &L4;
  // legs[LEG_5] = &L5;
  //*(legs + LEG_0) = &L0; (equivalent to the above)

  // plan = MotionPlanner(legs, 0);
  #endif
  #ifdef LEG_SETUP_DEBUG
  // s0.attach(j0, PWM_MIN, PWM_MAX);
  // s1.attach(j1, PWM_MIN, PWM_MAX);
  // s2.attach(j2, PWM_MIN, PWM_MAX);

  #endif
//Hexapod cheeto = Hexapod(plan);
//plan.powerOnSequence();
}

void loop() {
  #ifdef GLOBAL_DEBUG
  Serial.println("In loop");
  #endif
  delay(2000);
  #ifdef LEG_KINEMATICS_DEBUG
  
  //leg_test.moveFootToJV(home_j);
  leg_test.moveFootToJV(idle_j);
  //leg_test.moveFootToJV(storage_j);
  //leg_test.moveFootToPV(home_p, ELBOW_DOWN);
  //leg_test.moveFootToPV(storage_p, ELBOW_UP);
  leg_test.moveFootToPV(idle_p, ELBOW_DOWN);
  #endif
  #ifdef LEG_SETUP_DEBUG
  // s0.write(HPS_2018_CTR);
  // delay(2000);
  // s1.write(HPS_2018_CTR);
  // delay(2000);
  // s2.write(HPS_2027_CTR); // avg is middle, not 1000us
  // delay(2000);
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
  //plan.powerOffSequence();
  #endif
  #ifdef COMM_H_DEBUG
  cm.receiveMessage();
  
  #endif
  delay(2000);
  exit(1);
  
}

