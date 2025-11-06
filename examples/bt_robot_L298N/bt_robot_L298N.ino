/*
 * Example: Using Robotic_frame with L298N Motor Driver
 * ----------------------------------------------------
 * Author: Tamimu Said Rashid
 * License: MIT
 *
 * Description:
 * This example demonstrates how to use the Robotic_frame library 
 * to control a two-motor robotic vehicle using the L298N motor driver.
 * 
 * The setup includes:
 *   - Bluetooth serial communication
 *   - Front and back headlights
 *   - Forward, backward, left, and right movement control
 *
 * Hardware:
 *   - Arduino Uno or Mega
 *   - L298N motor driver module
 *   - Two DC motors (Left and Right)
 *   - Optional: Bluetooth module (HC-05 / HC-06)
 *   - Optional: LEDs for frontlight and backlight
 *
 * Connections:
 *   RX (Arduino pin 2)  -> TX (Bluetooth module)
 *   TX (Arduino pin 3)  -> RX (Bluetooth module)
 *
 *   Motor A:
 *     IN1 -> Pin 8
 *     IN2 -> Pin 7
 *     ENA -> Pin 9  (PWM speed control)
 *
 *   Motor B:
 *     IN3 -> Pin 5
 *     IN4 -> Pin 4
 *     ENB -> Pin 10 (PWM speed control)
 *
 *   Frontlight LED -> Pin 11
 *   Backlight LED  -> Pin 12
 *
 * Default serial baud rate: 115200
 * 
 * Control logic:
 *   - Commands are sent from the Bluetooth app to control direction and speed.
 *   - The robot can move forward, backward, left, or right.
 *   - Lights can be toggled using custom commands from the app.
 */

#include <robot_core.h>

// --- Define communication pins and motor pins ---
#define RX 2
#define TX 3
#define bot_baud 115200   // Baud rate for SoftwareSerial communication

// --- Motor driver (L298N) connections ---
#define motor1  8   // IN1
#define motor2  7   // IN2
#define motor3  5   // IN3
#define motor4  4   // IN4
#define speed1 9    // ENA (PWM)
#define speed2 10   // ENB (PWM)

// --- Lighting pins ---
#define frontlight 11
#define backlight 12

// --- Create robot object ---
Robotic_frame mybot(RX, TX, bot_baud);

void setup() {
  Serial.begin(9600);  // Optional serial monitor debugging

  // --- Configure hardware components ---
  mybot.setFrontlight(frontlight);   // Attach front light pin
  mybot.setBacklight(backlight);     // Attach back light pin
  mybot.setMotorDriver(L298N_MOTOR); // Select L298N motor driver mode

  // --- Attach L298N motor pins (IN1, IN2, ENA, IN3, IN4, ENB) ---
  mybot.attachMotor(motor1, motor2, speed1, motor3, motor4, speed2);

  // --- Initialize robot communication and configuration ---
  mybot.begin();
}

void loop() {
  // Continuously check for Bluetooth commands and control robot behavior
  mybot.bt_control();
}
