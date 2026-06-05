#ifndef J_POS_H
#define J_POS_H
#define J_POS_DEBUG

class J_Position{
    private:
        float t1, t2, t3;
    public:
        J_Position();
        J_Position(float, float, float);
        float getT1();
        float getT2();
        float getT3();
        void setT1(float);
        void setT2(float);
        void setT3(float);
};

#endif