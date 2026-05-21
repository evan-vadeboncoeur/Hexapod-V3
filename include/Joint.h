#ifndef JOINT_H
#define JOINT_H

#include "Servo.h"

class Joint{
    public:
        Joint();
        Joint(int id);
        float getAngle();
        void setupServo();
    private:
        int servo_id;
        float angle;
        Servo servo;
};

#endif