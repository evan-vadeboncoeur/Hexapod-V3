#ifndef JOINT_H
#define JOINT_H
//#define JOINT_DEBUG

#define JOINT_DELAY (30)
#define HPS_2018_CTR (90)
#define HPS_2027_CTR (1500)
#define PWM_MIN (500)
#define PWM_MAX (2500)

#include "Servo.h"
#include "HardwareSerial.h"
class Joint{
    public:
        Joint();
        Joint(int id);
        Joint(int id, int pwm_min, int pwm_max);
        float getAngle();
        void setupServo();
        void setAngle(int);
    private:
        int servo_id;
        float angle;
        Servo servo;

};

#endif