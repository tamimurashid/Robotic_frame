#include "robot_core.h"

#define RX 2
#define TX 3
#define bot_baud 115400
#define motor1  1
#define motor2  2
#define motor3  3
#define motor4  4



Robotic_frame mybot(RX, TX, 115400);
void setup() {
  mybot.setMotor(motor1, motor2,  motor3, motor4);
  mybot.begin();
 
}

void loop() {

  
  

}
