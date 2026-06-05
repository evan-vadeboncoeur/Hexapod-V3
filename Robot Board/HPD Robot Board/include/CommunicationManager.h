#ifndef CMMNCN_H
#define CMMNCN_H


// nrf24l01 communication manager: receive and trasmit code
// packet consists of:
// first bit: control mode (teleop/robot control)
// next 2 bits: gait mode (TRIPOD, RIPPLE, WAVE, QUADRUPED) (2 dipswitch)
// next 4 bits: direction (FORWARD, BACK, LEFT_UP, LEFT_DOWN, RIGHT_UP, RIGHT_DOWN, ROTATE_CW, ROTATE_CCW) (analog joystick L)
// next 4 bits: steps (dependent on analog read of how far stick is moved)

class CommunicationManager{
    public:
        enum Gait {TRIPOD, RIPPLE, WAVE, QUADRUPED} gait;
        enum Direction {FORWARD, BACKWARD, LEFT_UP, LEFT_DOWN, RIGHT_UP, RIGHT_DOWN, ROTATE_CW, ROTATE_CCW};
        
};

#endif