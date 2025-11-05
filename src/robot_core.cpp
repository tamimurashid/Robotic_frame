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


#include "robot_core.h"

Robotic_frame::Robotic_frame(uint8_t _RX, uint8_t _TX, unsigned long  _botbaudRate) : RX(_RX), TX(_TX), 
botbaudRate(_botbaudRate), _botSerial(_RX, _TX), motor1(nullptr), motor2(nullptr), motor3(nullptr), motor4(nullptr){}

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
void Robotic_frame::attachMotor(uint8_t _in1, uint8_t _in2, uint8_t _enA, uint8_t _in3, uint8_t _in4, uint8_t _enB){
    in1 = _in1;
    in2 = _in2;
    in3 = _in3;
    in4 = _in4;
    enA = _enA;
    enB = _enB;

    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);

}

void Robotic_frame::begin(){

   _botSerial.begin(botbaudRate);

   if(motorDriverType == MOTOR_SHIELD){
    setMotor(motor_1, motor_2, motor_3, motor_4);
    
   } else if (motorDriverType == L298N_MOTOR){
    attachMotor(in1, in2, enA, in3, in4, enB);
   }
   

   
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

    if(motorDriverType == MOTOR_SHIELD){
        motor1 ->setSpeed(speed);
        motor2 ->setSpeed(speed);
        motor3 ->setSpeed(speed);
        motor4 ->setSpeed(speed);

        motor1 ->run(FORWARD);
        motor2 ->run(FORWARD);
        motor3 ->run(BACKWARD);
        motor4 ->run(BACKWARD);

    } else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed);
        analogWrite(enB, speed);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }

    
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

void Robotic_frame::stop(){

    motor1 ->run(RELEASE);
    motor2 ->run(RELEASE);
    motor3 ->run(RELEASE);
    motor4 ->run(RELEASE);
}


