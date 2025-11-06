/*
 * Example: Using Robotic_frame with Adafruit Motor Shield (L293D)
 * ---------------------------------------------------------------
 * Author: Tamimu Said Rashid
 * License: MIT
 *
 * Description:
 * This example demonstrates how to use the Robotic_frame library 
 * to control a four-motor robotic vehicle using the Adafruit Motor Shield (L293D).
 * 
 * The setup includes:
 *   - Adafruit Motor Shield connected to Arduino Uno or Mega
 *   - Four DC motors (M1–M4)
 *   - Bluetooth module for wireless control (HC-05 / HC-06)
 *   - Optional front and back LED lights
 *
 * Hardware:
 *   - Arduino Uno or Mega
 *   - Adafruit Motor Shield (L293D version)
 *   - Four DC motors connected to M1, M2, M3, M4 ports
 *   - Optional: Bluetooth module on pins 2 (RX), 3 (TX)
 *   - Optional: LEDs for front and back lighting
 *
 * Control logic:
 *   - Commands are sent from the Bluetooth app (e.g., “F” for forward, “B” for backward).
 *   - The Robotic_frame library handles direction, speed, and light control.
 *   - Lights and horn can be controlled through dedicated commands.
 *
 * Notes:
 *   - Motor numbering corresponds to the Adafruit Motor Shield’s labeled outputs (M1–M4).
 *   - Speed is set using PWM (0–255).
 */

#include "robot_core.h"

// --- Define communication pins and motor channels ---
#define RX 2
#define TX 3
#define bot_baud 115200  // SoftwareSerial baud rate

// --- Adafruit Motor Shield motor channels ---
#define motor1 1   // M1
#define motor2 2   // M2
#define motor3 3   // M3
#define motor4 4   // M4

// --- Lighting pins (optional) ---
#define frontlight 11
#define backlight 12

// --- Create robot object ---
Robotic_frame mybot(RX, TX, bot_baud);

void setup() {
  Serial.begin(9600); // Optional serial debugging

  // --- Configure additional components ---
  mybot.setFrontlight(frontlight);   // Assign front LED pin
  mybot.setBacklight(backlight);     // Assign back LED pin

  // --- Select motor driver type (Adafruit Motor Shield) ---
  mybot.setMotorDriver(MOTOR_SHIELD);

  // --- Attach motors by their channel numbers (M1–M4) ---
  mybot.setMotor(motor1, motor2, motor3, motor4);

  // --- Initialize communication and motor setup ---
  mybot.begin();
}

void loop() {
  // Continuously check for Bluetooth commands and control robot behavior
  mybot.bt_control();
}
