#ifndef JOINT_H
#define JOINT_H

#define PWM_MIN (500)
#define PWM_MAX (2500)

#include "Servo.h"

class Joint{
    public:
        Joint();
        Joint(int id);
        Joint(int id, int pwm_min, int pwm_max);
        float getAngle();
        void setupServo();
        void setAngle(float);
    private:
        int servo_id;
        float angle;
        Servo servo;
};

#endif