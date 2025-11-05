#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>






class Robotic_frame{
    private:
        uint8_t  RX, TX, motor_1, motor_2, motor_3, motor_4;
        uint8_t speed;
        long int botbaudRate;
        SoftwareSerial _botSerial;
        AF_DCMotor  *motor1;  
        AF_DCMotor  *motor2;
        AF_DCMotor  *motor3;
        AF_DCMotor  *motor4;
   


    public:
        Robotic_frame(uint8_t _RX, uint8_t _TX, long int _botbaudRate);
        void begin();
        void setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4);
        void forward(uint8_t _speed);
        void backward(uint8_t  _speed);
        void left(uint8_t _speed);
        void right(uint8_t _speed);
    

   


};




#endif