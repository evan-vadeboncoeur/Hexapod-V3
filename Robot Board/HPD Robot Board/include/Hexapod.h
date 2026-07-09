#ifndef HEXAPOD_H
#define HEXAPOD_H
//#define GLOBAL_DEBUG

#include "MotionPlanner.h"
#include "CommunicationManager.h"
#include "BoardManager.h"

#define HEXAPOD_LOOP_DELAY (50)

class Hexapod{
    private:
        MotionPlanner plan;
        CommunicationManager radio;
        BoardManager board;
        Packet command;
        int gait, gait_old;
        bool power_off = false, power_on=false;
        Vector twist = Vector(0.0, 0.0, 0.0);
        Vector p_g; // global coordinate, if needed...
        enum State{WAITING, WALKING, TURNING}state=WAITING;
        void processPacket();
        void gaitSetup();
        void gaitShutdown();
        void checkBattery();
        void walk();
        void turn();

    public:
        Hexapod(int g, float df, float tc, float sh); // constructor 1 (all objects instantiated)
        void opMode(); // set operation mode: teleop (0), computer/robot (1)
        void startupHexapod();
        void shutdownHexapod();
        void stateManager(); // basically the hexapod's while loop
};

#endif