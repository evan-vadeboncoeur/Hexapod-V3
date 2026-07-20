#include <Arduino.h>
#include "MotionPlanner.h"
#include "Hexapod.h"

#define GLOBAL_DEBUG
#define LEG_SETUP_DEBUG
#define BODY_DEBUG
// might have to do joints & everything else BEFORE runtime, too, so that it's on the heap, not the stack?
#ifdef LEG_SETUP_DEBUG
Servo s0, s1, s2;
int j0 = LEG_1_J0;
int j1 = LEG_1_J1;
int j2 = LEG_1_J2;
#endif

// Hexpaod Variables
int g = 0;
float duty_f = 0.5;
float cycle_time = 1.5;
float step_h = 55.0;
float vel = 140.0;

//#define LEG_KINEMATICS_DEBUG
#ifdef LEG_KINEMATICS_DEBUG
Leg leg_test = Leg(LEG_0, j0, j1, j2, CCW_CONFIG);
Vector home_j = Vector(0.0, 0.0, 0.0);
Vector storage_j = Vector(0.0, -1.216, -1.763);
Vector idle_j = Vector(0.0, -0.977, 2.146);
Vector home_p = Vector(310.0, 0.0, 0.0);
Vector storage_p = Vector(-46.76, 0.0, 108.68); // CAD output
Vector idle_p = Vector(178.755, 0.0, -63.631); // CAD output
Vector idle_j2 = Vector(0.0, -0.643, 1.9211971);
Vector idle_p2 = Vector(185.29, 0.00, -89.66);
#endif

#ifndef LEG_SETUP_DEBUG
Hexapod* hp = nullptr;
//MotionPlanner* mp = nullptr;
//CommunicationManager* cm = nullptr;
Vector twist = Vector(110.0, 0.0, 0.0);
Vector twist2 = Vector(0.0, 0.0, 0.4);

//Vector twist = Vector(0.0, 0.0, 0.3);
#endif

void setup(){
  #ifdef GLOBAL_DEBUG
  Serial.begin(9600); // open before creating legs (at least in testing phases)
  delay(1000);
  Serial.println("#####In setup#####");
  #endif

  #ifndef LEG_SETUP_DEBUG
  //mp = new MotionPlanner(g, duty_f, cycle_time, step_h);
  //static MotionPlanner planner(g, duty_f, cycle_time, step_h); // statically stored for life of program
  //mp = &planner;
  //mp->powerOnSequence();
  static Hexapod hexa(g, duty_f, cycle_time, step_h);
  hp = &hexa;
  hp->commInit();
  //hp->startupHexapod();
  #endif

  #ifdef COMM_DEBUG
  //static CommunicationManager comm;
  //cm = &comm;
  //cm->commBegin();
  #endif
 

  #ifdef LEG_SETUP_DEBUG
  s0.attach(j0, PWM_MIN, PWM_MAX);
  s1.attach(j1, PWM_MIN, PWM_MAX);
  s2.attach(j2, PWM_MIN, PWM_MAX);
  #endif
  
 delay(1000);
}

void loop() {
  #ifdef GLOBAL_DEBUG
  Serial.println("#####In loop#####");
  #endif
  #ifdef LEG_KINEMATICS_DEBUG
  //leg_test.moveFootToJV(home_j);
  //leg_test.moveFootToJV(idle_j);
  //leg_test.moveFootToJV(storage_j);
  //leg_test.moveFootToPV(home_p, ELBOW_DOWN);
  //leg_test.moveFootToPV(storage_p, ELBOW_UP);
  //leg_test.moveFootToPV(idle_p, ELBOW_DOWN);
  leg_test.moveFootToJV(idle_j2);
  leg_test.moveFootToPV(idle_p2, ELBOW_DOWN);
  #endif
  #ifdef LEG_SETUP_DEBUG
  s0.write(HPS_2018_CTR);
  delay(1000);
  s1.write(HPS_2018_CTR);
  delay(1000);
  s2.write(HPS_2027_CTR); // avg is middle, not 1000us
  delay(1000);
  //s2.write(545); // test deadband
  delay(2000);
  #endif
  #ifdef BODY_DEBUG
  
  #endif
  #ifdef COMM_DEBUG
  //cm->receivePacket();
  //delay(200);
  #endif
  //hp->stateManager();
  #ifndef LEG_SETUP_DEBUG
  // mp->movementSetup(0, twist);
  // delay(1000);
  // mp->moveIdle();
  // delay(1000);
  // mp->movementSetup(0, twist2);
  // delay(3000);
  //Serial.println(digitalRead(2));
  //mp->powerOffSequence();
  #endif
  delay(2000);
  exit(1);
  
}

