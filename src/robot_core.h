#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>

class Robotic_frame{
    private:
    uint8_t  motor_1;
    uint8_t  motor_2;
    uint8_t  motor_3;
    uint8_t  motor_4;
    uint8_t  RX;
    uint8_t  TX;
    uint8_t  botbaudRate;


  public:
    Robotic_frame(uint8_t _RX, uint8_t _TX, uint8_t _botbaudRate);
    void begin();
    void setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4);

   


};




#endif