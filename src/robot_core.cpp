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
#include "sensors.h"

Robotic_frame::Robotic_frame(uint8_t _RX, uint8_t _TX, unsigned long  _botbaudRate) : RX(_RX), TX(_TX), 
botbaudRate(_botbaudRate), _botSerial(_RX, _TX), motor1(nullptr), motor2(nullptr), motor3(nullptr), motor4(nullptr){}

Robotic_frame::~Robotic_frame() {
    delete motor1;
    delete motor2;
    delete motor3;
    delete motor4;
    // Optional: set pointers to nullptr (safe practice)
    motor1 = nullptr;
    motor2 = nullptr;
    motor3 = nullptr;
    motor4 = nullptr;
}
void Robotic_frame::setMotor(uint8_t _motor_1, uint8_t _motor_2, uint8_t _motor_3, uint8_t _motor_4){
    motor_1 = _motor_1;
    motor_2 = _motor_2;
    motor_3 = _motor_3;
    motor_4 = _motor_4;
   delete motor1; motor1 = new  AF_DCMotor(motor_1);
   delete motor2; motor2 = new  AF_DCMotor(motor_2);
   delete motor3; motor3 = new  AF_DCMotor(motor_3);
   delete motor4; motor4 = new  AF_DCMotor(motor_4);
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
    if(motorDriverType == MOTOR_SHIELD){
        motor1 ->setSpeed(speed);
        motor2 ->setSpeed(speed);
        motor3 ->setSpeed(speed);
        motor4 ->setSpeed(speed);
        motor1 ->run(FORWARD);
        motor2 ->run(FORWARD);
        motor3 ->run(FORWARD);
        motor4 ->run(FORWARD);
    }else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed);
        analogWrite(enB, speed);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);

    } 
}
void Robotic_frame::backward(uint8_t _speed){
    speed = _speed;
    if(motorDriverType == MOTOR_SHIELD){
        motor1 ->setSpeed(speed);
        motor2 ->setSpeed(speed);
        motor3 ->setSpeed(speed);
        motor4 ->setSpeed(speed);
        motor1 ->run(BACKWARD);
        motor2 ->run(BACKWARD);
        motor3 ->run(BACKWARD);
        motor4 ->run(BACKWARD);
    }else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed);
        analogWrite(enB, speed);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }  
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
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }  
}
void Robotic_frame::right(uint8_t _speed){
    speed = _speed;
    if(motorDriverType == MOTOR_SHIELD){
        motor1 ->setSpeed(speed);
        motor2 ->setSpeed(speed);
        motor3 ->setSpeed(speed);
        motor4 ->setSpeed(speed);
        motor1 ->run(BACKWARD);
        motor2 ->run(BACKWARD);
        motor3 ->run(FORWARD);
        motor4 ->run(FORWARD);
    } else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed);
        analogWrite(enB, speed);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
}
void Robotic_frame::upright(uint8_t _speed){
    speed = _speed;
    if(motorDriverType == MOTOR_SHIELD){
        motor1->setSpeed(speed);    // left motors
        motor2->setSpeed(speed/2);  // slow down right side
        motor3->setSpeed(speed);    
        motor4->setSpeed(speed/2);
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor3->run(FORWARD);
        motor4->run(FORWARD);
    }
    else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed);   // left motors
        analogWrite(enB, speed/2); // right motors
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        digitalWrite(in3,HIGH);
        digitalWrite(in4,LOW);
    }
}
void Robotic_frame::upleft(uint8_t _speed){
    speed = _speed;
    if(motorDriverType == MOTOR_SHIELD){
        motor1->setSpeed(speed/2);    //  slow down left motors
        motor2->setSpeed(speed);  // right side
        motor3->setSpeed(speed/2);    
        motor4->setSpeed(speed);
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor3->run(FORWARD);
        motor4->run(FORWARD);
    }
    else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed/2);   // left motors
        analogWrite(enB, speed); // right motors
        digitalWrite(in1,HIGH);
        digitalWrite(in2,LOW);
        digitalWrite(in3,HIGH);
        digitalWrite(in4,LOW);
    }
}
void Robotic_frame::downright(uint8_t _speed){
    speed = _speed;
    if(motorDriverType == MOTOR_SHIELD){
        motor1->setSpeed(speed);      // left motors full speed
        motor2->setSpeed(speed/2);    // right motors slower
        motor3->setSpeed(speed);      
        motor4->setSpeed(speed/2);

        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        motor3->run(BACKWARD);
        motor4->run(BACKWARD);
    }
    else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed);       // left motors full speed
        analogWrite(enB, speed/2);     // right motors slower
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
}
void Robotic_frame::downleft(uint8_t _speed){
    speed = _speed;
    if(motorDriverType == MOTOR_SHIELD){
        motor1->setSpeed(speed/2);    // left motors slower
        motor2->setSpeed(speed);      // right motors full speed
        motor3->setSpeed(speed/2);    
        motor4->setSpeed(speed);

        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        motor3->run(BACKWARD);
        motor4->run(BACKWARD);
    }
    else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, speed/2);     // left motors slower
        analogWrite(enB, speed);       // right motors full speed
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }
}
void Robotic_frame::stop(){
    if(motorDriverType == MOTOR_SHIELD){
        motor1 ->run(RELEASE);
        motor2 ->run(RELEASE);
        motor3 ->run(RELEASE);
        motor4 ->run(RELEASE);
    } else if(motorDriverType == L298N_MOTOR){
        analogWrite(enA, 0); analogWrite(enB, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);

    }
 
}

void Robotic_frame::horn(uint8_t buzzerpin){
    _buzzerpin = buzzerpin;

    pinMode(_buzzerpin, OUTPUT);
        // Alternate between 500 Hz and 600 Hz quickly
    for (int i = 0; i < 5; i++) {
        tone(_buzzerpin, 500); 
        delay(200);
        tone(_buzzerpin, 600);
        delay(200);
    }
    
    noTone(_buzzerpin);
    delay(1000); // wait before next horn
}

void Robotic_frame::no_horn(){
    noTone(_buzzerpin);
}

void Robotic_frame::setFrontlight(uint8_t frontled){
    _frontled = frontled;
    pinMode(_frontled, OUTPUT);

}

void Robotic_frame::setBacklight(uint8_t backled){
    _backled = backled;
    pinMode(_backled, OUTPUT);

}

void Robotic_frame::frontlight_on(){
    
    digitalWrite(_frontled, HIGH);
}

void Robotic_frame::frontlight_off(){

    digitalWrite(_frontled, LOW);
}

void Robotic_frame::backlight_off(){
    
    digitalWrite(_backled, HIGH);
}

void Robotic_frame::backlight_on(){

    digitalWrite(_backled, LOW);
}


void Robotic_frame:: setServo(uint8_t servoPin){
    _servoPin = servoPin;
    _servo.attach(_servoPin);
}


void Robotic_frame:: writeServo(uint8_t angle){
    _angle = angle;
    _servo.write(_angle);
}

void Sensors::ultrasonic(uint8_t trigpin, uint8_t echopin){
    _trigpin = trigpin;
    _echopin = echopin;

    pinMode(_trigpin, OUTPUT);
    pinMode(_echopin, INPUT);

}

long Sensors::readDistance(){
    digitalWrite(_trigpin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trigpin, HIGH);
    delayMicroseconds(10);

    digitalWrite(_trigpin, LOW);

    long duration = pulseIn(_echopin, HIGH);

    long distance = duration * 0.034 / 2;

    return distance;

}

void Robotic_frame::obstacle_control(){
    long frontdistance  = readDistance();
    if(frontdistance > SAFE_DISTANCE){
        forward(speed);
        return;

    }
    stop();

      //scan left 
    writeServo(0);
    delay(500);
    long leftdistance = readDistance();


       //scan right
    writeServo(180);
    delay(500);
    long rightdistance = readDistance();
    

    writeServo(90);
    delay(300);

    if(leftdistance > rightdistance && leftdistance > SAFE_DISTANCE){
        left(speed);
    }




    
}

void Robotic_frame::bt_control(){
    if(_botSerial.available()){
        command = _botSerial.read();
        Serial.print("The receive command is: ");
        Serial.println(command);
        switch (command)
        {    // Speed gears control, this works for some apps.
            case '1': speed = 28; break;
            case '2': speed = 56; break;
            case '3': speed = 85; break;
            case '4': speed = 113; break;
            case '5': speed = 142; break;
            case '6': speed = 170; break;
            case '7': speed = 199; break;
            case '8': speed = 227; break;
            case '9': speed = 255; break;
            // Normal movement forward, backward, left and right 
            case 'F': forward(speed); break;
            case 'B': backward(speed); break;
            case 'L': left(speed); break;
            case 'R': right(speed); break;
            // diagonal movement  upleft, upright, downleft  and downright
            case 'G': upleft(speed); break;
            case 'I': upright(speed); break;
            case 'H': downleft(speed); break;
            case 'J': downright(speed); break;
            // For horn if available 
            case 'V': horn(_buzzerpin);
            case 'v': no_horn();
            // For light front and back if available 
            case 'W': frontlight_on();
            case 'w': frontlight_off();
            case 'U': backlight_on();
            case 'u': backlight_off();
            default: stop(); break;
           
        }
    }   
}


