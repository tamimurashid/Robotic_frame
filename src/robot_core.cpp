#include "robot_core.h"

Robotic_frame::Robotic_frame(uint8_t _RX, uint8_t _TX, unsigned long  _botbaudRate) : RX(_RX), TX(_TX), botbaudRate(_botbaudRate),
_botSerial(_RX, _TX), motor1(nullptr), motor2(nullptr), motor3(nullptr), motor4(nullptr){}

void Robotic_frame::setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4){
    motor_1 = _motor_1;
    motor_2 = _motor_2;
    motor_3 = _motor_3;
    motor_4 = _motor_4;

    motor1 = new  AF_DCMotor(motor_1);
    motor2 = new  AF_DCMotor(motor_2);
    motor3 = new  AF_DCMotor(motor_3);
    motor4 = new  AF_DCMotor(motor_4);
    
}


void Robotic_frame::begin(){


   _botSerial.begin(botbaudRate);

   setMotor(motor_1, motor_2, motor_3, motor_4);
}

void Robotic_frame::forward(uint8_t _speed){
    speed = _speed;

    motor1 ->setSpeed(speed);
    motor2 ->setSpeed(speed);
    motor3 ->setSpeed(speed);
    motor4 ->setSpeed(speed);

    motor1 ->run(FORWARD);
    motor2 ->run(FORWARD);
    motor3 ->run(FORWARD);
    motor4 ->run(FORWARD);
}

void Robotic_frame::backward(uint8_t _speed){
    speed = _speed;

    motor1 ->setSpeed(speed);
    motor2 ->setSpeed(speed);
    motor3 ->setSpeed(speed);
    motor4 ->setSpeed(speed);

    motor1 ->run(BACKWARD);
    motor2 ->run(BACKWARD);
    motor3 ->run(BACKWARD);
    motor4 ->run(BACKWARD);
}

void Robotic_frame::left(uint8_t _speed){
    speed = _speed;

    motor1 ->setSpeed(speed);
    motor2 ->setSpeed(speed);
    motor3 ->setSpeed(speed);
    motor4 ->setSpeed(speed);

    motor1 ->run(FORWARD);
    motor2 ->run(FORWARD);
    motor3 ->run(BACKWARD);
    motor4 ->run(BACKWARD);
}

void Robotic_frame::right(uint8_t _speed){
    speed = _speed;

    motor1 ->setSpeed(speed);
    motor2 ->setSpeed(speed);
    motor3 ->setSpeed(speed);
    motor4 ->setSpeed(speed);

    motor1 ->run(BACKWARD);
    motor2 ->run(BACKWARD);
    motor3 ->run(FORWARD);
    motor4 ->run(FORWARD);
}


