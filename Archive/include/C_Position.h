#ifndef C_POS
#define C_POS

class C_Position{
    public:
        C_Position();
        C_Position(float, float, float);
        void setPosition(float x_s, float y_s, float z_s);
        float getX();
        float getY();
        float getZ();
    private:
        float x, y, z;
};

#endif