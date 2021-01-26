# RoboticArm
Arduino library for a 6-axis robotic arm

Written 2021 by Joel Lüdi
Version: 1
Published: 2021

This library allows you to manage your 6-Axis robot.
It should work with any kind of robotic arm, but was written in the intention to work with this one from Sainsmart:
https://www.sainsmart.com/products/6-axis-desktop-robotic-arm-assembled
For the wiring and setup consider this page:
http://wiki.sainsmart.com/index.php/DIY_6-Axis_Servos_Control_Palletizing_Robot_Arm_Model_for_Arduino_UNO_MEGA2560

Supported functions:
* Managing of all Servo-side programming
* Output of the current state of the robot as feedback
* moveTo-function: sends the robot to a given target-position without a speed limit

Note:
* All user side units are either in [mm] or [°]
* CAUTION! Be always careful when working with the moving robot!
* Always drive small steps! (<10mm)
* A very big step could cause your robot to move very quickly and destroy its servos, the power sourve or environment! (Yes it happened to me)

I hope you'll be able to realize cool projects with this library
Happy tinkering!

Methods:

	//attaches all Servos to the given pins
-	attach(int pinAxis1, int pinAxis2, int pinAxis3, int pinAxis4, int pinAxis5, int pinAxis6);

	//moves the robot to a coordinate with the current set speed
-	void move(int X, int Y, int Z, int U, int V, int W);

	//moves the robot to a saved position with the current set speed
-	void move(int inPosition[6]);

	//Moves to the home-position {0, 125, 220, 0, 0, 0}
-	void moveHome();

	//gets the current state as a String for observing or further programming
-	String getState();

	//enables or disables the Serial printing of the current state
-	void setSerialSendState(bool sendState);
