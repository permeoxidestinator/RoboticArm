/*
   This example shows you how to do the first steps with the Sainsmart 6-Axis robot:
   https://www.sainsmart.com/products/6-axis-desktop-robotic-arm-assembled
   It will initialize the robot and move it to multiple positions
*/


//include the library
#include <RoboticArm.h>

//create the RoboticArm as object
RoboticArm bot;

void setup()
{
  //Open the serial connection
  Serial.begin(115200);
  //Print the current robot state on the serial monitor
  Serial.println(bot.getState());
  //Allow the robot to send its state via the serial connection by itself
  bot.setSerialSendState(true);
  //attach the servos of the robotic arm
  //CAUTION! This will move every single servo to its home-position with a delay of 1000ms!
  bot.attach(11, 10, 9, 6, 5, 3);
  delay(2000);

  //move the robot to different positions
  bot.move(-20, 105, 200, 0, 0, 0);
  //Always drive small steps! (<10mm)
  //A big movement could cause your robot to move very quickly and destroy its servos, the power source or environment!
  delay(2000);
  //just change the U-axis
  bot.move(-20, 105, 200, 30, 0, 0);
  delay(2000);
  //save a position in an array
  int myPosition[6] = {50, 100, 200, -30, 20, 0};
  //move the robot to the saved position
  bot.move(myPosition);
  delay(2000);
  //move to the home-position where the robot started
  bot.moveHome();
}

void loop()
{
  //do nothing
}
