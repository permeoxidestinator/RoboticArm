/*------------------------------------------------
Written 2021 by Joel Lüdi
Version: 1
Published: 26.01.2021

This library allows you to manage your 6-Axis robot.
It was written in the intention to work with this one from Sainsmart:
https://www.sainsmart.com/products/6-axis-desktop-robotic-arm-assembled
For the wiring and setup consider this page:
http://wiki.sainsmart.com/index.php/DIY_6-Axis_Servos_Control_Palletizing_Robot_Arm_Model_for_Arduino_UNO_MEGA2560

Supported functions:
* Managing of all Servo-side programming
* Output of the current state of the robot as feedback
* move-function: sends the robot to a given target-position without a speed limit

Note:
* All user side units are either in [mm] or [°]
* CAUTION! Be always careful when working with the moving robot!
* Always drive small steps! (<10mm)
* A very big step could cause your robot to move very quickly and destroy its servos, the power source or environment! (Yes it happened to me)

I hope you'll be able to realize cool projects with this library
Happy tinkering!
------------------------------------------------*/

#ifndef RoboticArm_h
#define RoboticArm_h

#include "Arduino.h"
#include "Servo.h"

class RoboticArm
{
public:
	//constructor
	RoboticArm();

	//attaches all Servos to the given pins
	void attach(int pinAxis1, int pinAxis2, int pinAxis3, int pinAxis4, int pinAxis5, int pinAxis6);

	//moves the robot to a coordinate
	void move(int X, int Y, int Z, int U, int V, int W);

	//moves the robot to a saved position
	void move(int inPosition[6]);

	//Moves to the home-position {0, 125, 220, 0, 0, 0}
	void moveHome();

	//gets the current state as a String for observing or further programming
	String getState();

	//enables or disables the Serial printing of the current state
	void setSerialSendState(bool sendState);

private:

	//roboter parameters
	Servo servoAxis1, servoAxis2, servoAxis3, servoAxis4, servoAxis5, servoAxis6;
	const double MAXALLOWEDDIFFERENCE = 0.1;
	const int HEIGHTSHOULDER = 100;	//[mm]
	const int LENGTHFOREARM = 125;	//[mm]
	const int LENGTHUPPERARM = 120;	//[mm]
	const int homeCoord[6] = { 0,125,220,0,0,0 };	//default home position [mm/mm/mm/°/°/°]
	const int axisLimit[6][2] = { {0,180},{5,140},{30,155},{0,180},{0,150},{0,180} };
	/*Axis limits:
	Axis1: 0° - 180°
	Axis2: 5° - 140°
	Axis3: 30° - 155°
	Axis4: 0° - 180°
	Axis5: 0° - 150°
	Axis6: 0° - 180°
	*/

	//roboter variables
	String state = "";
	bool sendSerialEnabled = false;

	//coordinates and calculation
	int fingerCoord[6] = { 0,125,220,0,0,0 };	//target coordinates of the robots finger [mm/mm/mm/°/°/°] (default in home position)
	double wristCoord[6];	//target coordinates of the robots wrist [mm/mm/mm/rad/rad/rad]
	double mathAngle[6];	//calculated mathematical angle of the axis [rad]
	int machineAngle[6] = { 90,15,90,90,90,90 };	//adjusted angle to the machine axis [°] (default in home position)
	//fingerCoord -> wristCoord -> mathAngle -> machineAnlge

	//mathematical variables and constants
	double vecVecProd[3];
	double berhx2;
	double berhy2;
	double berOH[3];
	double berx0;
	double bery0;
	double berz0;
	double berx1;
	double bery1;
	double berz1;
	double berdxy;
	double berdxyz;
	double tempBetha1;
	double tempBetha2;
	double tempGamma1;
	double tempGamma2;
	double calcX;
	double calcY;
	double calcZ;

	//outputs the current robot state as String and via Serial.print()
	void sendState(String message);

	//outputs a String with the given coordinates
	String coordToString(int X, int Y, int Z, int U, int V, int W);

	//returns the length of a given vector
	double calcLength(double inVector[3]);

	//returns the scalar product of two given vectors
	double calcScalProd(double inVector1[], double inVector2[]);

	//returns the vector product of two given vectors into the variable vecVecProd
	void calcVecProd(double inVector1[], double inVector2[]);

	//returns the angle between two given vectors
	double calcAngle(double inVector1[], double inVector2[]);

	//converts the angle from ° to rad
	double toRad(double angleDegree);

	//converts the angle from rad to °
	double toDegree(double angleRad);

	//calculates from machine angles to coordinates
	void calcCoordinates();

	//calculates coordinate to machine angles
	bool calcMachineAngles();

	//returns an error-String with the unreachable coordinates
	String calcError();
};

#endif