#include "robot_core.h"

Robotic_frame::Robotic_frame(uint8_t _RX, uint8_t _TX, uint8_t _botbaudRate) : RX(_RX), TX(_TX), botbaudRate(_botbaudRate),
_botSerial(_RX, _TX), motor1(1), motor2(2), motor3(3), motor4(4){}

void Robotic_frame::setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4){
    motor_1 = _motor_1;
    motor_2 = _motor_2;
    motor_3 = _motor_3;
    motor_4 = _motor_4;

    motor1 = AF_DCMotor(motor_1);
    motor2 = AF_DCMotor(motor_2);
    motor3 = AF_DCMotor(motor_3);
    motor4 = AF_DCMotor(motor_4);
    
}


void Robotic_frame::begin(){
   _botSerial.begin(botbaudRate);
   setMotor(motor_1, motor_2, motor_3, motor_4);
}


