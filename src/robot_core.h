#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>






class Robotic_frame{
    private:
        uint8_t  RX, TX, botbaudRate, motor_1, motor_2, motor_3, motor_4;
        SoftwareSerial _botSerial;
        AF_DCMotor  motor1, motor2, motor3, motor4;
   


    public:
        Robotic_frame(uint8_t _RX, uint8_t _TX, uint8_t _botbaudRate);
        void begin();
        void setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4);
    

   


};




#endif