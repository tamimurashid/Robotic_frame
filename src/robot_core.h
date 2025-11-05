/*
 * Robotics Framework Core Library
 * --------------------------------
 * Author: Tamimu Said Rashid
 * License: MIT
 *
 * Description:
 * This library serves as the foundation for a modular robotics framework
 * designed to simplify robotic programming using:
 *   - Arduino Uno
 *   - Arduino Mega
 *   - ESP32 (in combination with Raspberry Pi)
 *
 * Overview:
 * The core file provides essential components for robotic movement control.
 * It is primarily built to support the following motor drivers:
 *   - Adafruit Motor Shield (recommended for best performance)
 *   - LM2903 Motor Driver
 *
 * The framework is extensible and supports the integration of additional
 * components through SoftwareSerial, enabling communication with:
 *   - Bluetooth modules (e.g., HC-05 / HC-06)
 *   - Other Arduino boards
 *   - ESP32 boards
 *   - Raspberry Pi systems
 *
 * Using this framework, you can design and build robots that combine
 * hardware control with higher-level processing such as machine learning
 * and algorithmic automation—without repeatedly rewriting core logic.
 *
 * Notes:
 * - The project is currently implemented as a library but is being expanded
 *   into a full framework to support advanced robotic architectures.
 * - Contributions and extensions are welcome under the MIT License.
 */

#ifndef ROBOT_FRAME_H
#define ROBOT_FRAME_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <AFMotor.h>



class Robotic_frame{
    private:
        uint8_t  RX, TX, motor_1, motor_2, motor_3, motor_4;
        uint8_t speed;
        unsigned long  botbaudRate;
        SoftwareSerial _botSerial;
        AF_DCMotor  *motor1;  
        AF_DCMotor  *motor2;
        AF_DCMotor  *motor3;
        AF_DCMotor  *motor4;
   


    public:
        Robotic_frame(uint8_t _RX, uint8_t _TX, unsigned long  _botbaudRate);
        void begin();
        void setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4);
        void forward(uint8_t _speed);
        void backward(uint8_t  _speed);
        void left(uint8_t _speed);
        void right(uint8_t _speed);
    

   


};




#endif