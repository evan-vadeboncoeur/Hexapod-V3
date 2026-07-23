#ifndef HEXAPOD_H
#define HEXAPOD_H
//#define HEXAPOD_DEBUG

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
        uint8_t gait, gait_old;
        bool power_off = false, power_on=false, powered_on = false, turnc = false;
        uint64_t b_check = 0, b_check_prev = 0;
        uint16_t b_time = 1000;
        Vector twist = Vector(0.0, 0.0, 0.0);
        Vector p_g; // global coordinate, if needed...
        enum State{WAITING, WALKING, TURNING, POWER_ON, POWER_OFF}state=WAITING;
        void processPacket();
        void gaitSetup();
        void gaitShutdown();
        void checkBattery();
        void walk();
        void turn();
        void getCommand();
        uint64_t prev_robot=0, prev_comm=0;
        uint16_t robot_update=15, comm_update=10;
        
    public:
        Hexapod(uint8_t g, float df, float tc, float sh); // constructor 1 (all objects instantiated)
        void startupHexapod();
        void homeHexapod();
        void shutdownHexapod();
        void stateManager(); // basically the hexapod's while loop
};

#endif