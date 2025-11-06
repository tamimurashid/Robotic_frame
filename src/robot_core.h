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
#include <AFMotor.h> // Library for adafruit motor driver 
// #define MOTOR_SHIELD 1
// #define L298N_MOTOR 2


enum MotorDriverType{
    MOTOR_SHIELD,
    L298N_MOTOR 
};

class Robotic_frame{
    private:
        uint8_t  RX, TX, motor_1, motor_2, motor_3, motor_4, _buzzerpin, _frontled, _backled;
        uint8_t speed = 255; // set speed to maximum if in motor driver L298N enA and enB is not used incase 
        unsigned long  botbaudRate;

        SoftwareSerial _botSerial;
        MotorDriverType  motorDriverType;


        AF_DCMotor  *motor1;  // pointer for motor objects from 1 to 4
        AF_DCMotor  *motor2;
        AF_DCMotor  *motor3;
        AF_DCMotor  *motor4;

        // --- L298N Motor Driver pins ---
        uint8_t in1, in2, enA;
        uint8_t in3, in4, enB;
   


    public:
        Robotic_frame(uint8_t _RX, uint8_t _TX, unsigned long  _botbaudRate);


        // Destructor
       ~Robotic_frame();
         

        //----------- Setup -------------// 

        void begin();

        void setMotorDriver(MotorDriverType type){
            motorDriverType = type;
        } // choose kind of motor driver .


         //-------- Adafruit motor(L293D MOTOR_SHIELD) Configuration ------------//

        void setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4);


        //---------- L298N_MOTOR  Configuration -------------//
        void attachMotor(uint8_t _in1, uint8_t _in2, uint8_t _enA, uint8_t _in3, uint8_t _in4, uint8_t _enB);



        void forward(uint8_t _speed);

        void backward(uint8_t  _speed);

        void left(uint8_t _speed);

        void right(uint8_t _speed);

        void upright(uint8_t _speed);

        void upleft(uint8_t _speed);

        void downright(uint8_t _speed);

        void downleft(uint8_t _speed);

        void bt_control();

        void horn(uint8_t buzzerpin);
        void no_horn();

        void frontlight_on(uint8_t frontled);

        void frontlight_off(uint8_t frontled);

        void backlight_on(uint8_t backled);

        void backlight_off(uint8_t backled);

        void stop();
        char command;
    

   


};




#endif