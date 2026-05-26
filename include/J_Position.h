#ifndef J_POS_H
#define J_POS_H

class J_Position{
    private:
        float t1, t2, t3;
    public:
        J_Position();
        J_Position(float, float, float);
        float getT1();
        float getT2();
        float getT3();
        float setT1(float);
        float setT2(float);
        float setT3(float);
};

#endif