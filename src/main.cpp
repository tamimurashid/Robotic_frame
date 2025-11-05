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
  
  mybot.forward(255);
  delay(1000);
  mybot.backward(255);
  delay(1000);
  mybot.left(255);
  delay(1000);
  mybot.right(255);

  
  

}
