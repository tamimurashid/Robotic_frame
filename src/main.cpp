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

#define RX 2
#define TX 3
#define bot_baud 9600
#define motor1  8
#define motor2  7
#define motor3  5
#define motor4  4
#define speed1 9
#define speed2 10
#define frontlight 11
#define backlight 12



Robotic_frame mybot(RX, TX, bot_baud);
void setup() {
  
  Serial.begin(9600);
  mybot.setFrontlight(frontlight);
  mybot.setBacklight(backlight);
  mybot.setMotorDriver(L298N_MOTOR);
  mybot.attachMotor(motor1, motor2, speed1, motor3, motor4, speed2);
  mybot.begin();
 
}

void loop() {
 mybot.bt_control();

}
