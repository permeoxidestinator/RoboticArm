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



#include "Arduino.h"
#include "RoboticArm.h"

#define Xaxis 0
#define Yaxis 1
#define Zaxis 2
#define Uaxis 3
#define Vaxis 4
#define Waxis 5

#define ALFA 0
#define BETHA 1
#define GAMMA 2
#define DELTA 3
#define EPSILON 4
#define ZETA 5

#define LOWERLIMIT false
#define UPPERLIMIT true


RoboticArm::RoboticArm()
{
	//Can only be read via the getState()-Method, since the constructor is executed before the user can set sendSerialEnabled true
	sendState("Waiting to be attached");
}

void RoboticArm::attach(int pinAxis1, int pinAxis2, int pinAxis3, int pinAxis4, int pinAxis5, int pinAxis6)
{
	//attaches all Servos to the given pins
	//with 1000ms pause each servo will power up alone to prevent a current-peak
	sendState("Attaching");
	servoAxis1.attach(pinAxis1);
	servoAxis1.write(machineAngle[0]);
	delay(1000);
	servoAxis2.attach(pinAxis2);
	servoAxis2.write(machineAngle[1]);
	delay(1000);
	servoAxis3.attach(pinAxis3);
	servoAxis3.write(machineAngle[2]);
	delay(1000);
	servoAxis4.attach(pinAxis4);
	servoAxis4.write(machineAngle[3]);
	delay(1000);
	servoAxis5.attach(pinAxis5);
	servoAxis5.write(machineAngle[4]);
	delay(1000);
	servoAxis6.attach(pinAxis6);
	servoAxis6.write(machineAngle[5]);
	sendState("Attached");
}

void RoboticArm::move(int X, int Y, int Z, int U, int V, int W)
{
	//moves the robot to a coordinate
	fingerCoord[Xaxis] = X;
	fingerCoord[Yaxis] = Y;
	fingerCoord[Zaxis] = Z;
	fingerCoord[Uaxis] = U;
	fingerCoord[Vaxis] = V;
	fingerCoord[Waxis] = W;
	if (calcMachineAngles())
	{
		sendState("Moving to " + coordToString(X, Y, Z, U, V, W));
		servoAxis1.write(machineAngle[0]);
		servoAxis2.write(machineAngle[1]);
		servoAxis3.write(machineAngle[2]);
		servoAxis4.write(machineAngle[3]);
		servoAxis5.write(machineAngle[4]);
		servoAxis6.write(machineAngle[5]);
	}
}

void RoboticArm::move(int inPosition[6])
{
	//moves the robot to a saved position
	move(inPosition[0], inPosition[1], inPosition[2], inPosition[3], inPosition[4], inPosition[5]);
}

void RoboticArm::moveHome()
{
	//Moves to the home-position
	move(homeCoord);
}

String RoboticArm::getState()
{
	//gets the current state as a String for observing or further programming
	return state;
}

void RoboticArm::setSerialSendState(bool sendState)
{
	//enables or disables the Serial printing of the current state
	sendSerialEnabled = sendState;
}

void RoboticArm::sendState(String message)	//outputs the current robot state as String and via Serial.print()
{
	state = message;
	if (sendSerialEnabled) Serial.println(message);
}

String RoboticArm::coordToString(int X, int Y, int Z, int U, int V, int W)
{
	//outputs a String with the given coordinates
	return String(X) + "|" + String(Y) + "|" + String(Z) + "|" + String(U) + "|" + String(V) + "|" + String(W);
}

double RoboticArm::calcLength(double inVector[3])	//	returns the length of a given vector
{
	return sqrt(inVector[Xaxis] * inVector[Xaxis] + inVector[Yaxis] * inVector[Yaxis] + inVector[Zaxis] * inVector[Zaxis]);
}

double RoboticArm::calcScalProd(double inVector1[], double inVector2[])	//returns the scalar product of two given vectors
{
	return inVector1[Xaxis] * inVector2[Xaxis] + inVector1[Yaxis] * inVector2[Yaxis] + inVector1[Zaxis] * inVector2[Zaxis];
}

void RoboticArm::calcVecProd(double inVector1[], double inVector2[])	//returns the vector product of two given vectors into the variable vecVecProd
{
	vecVecProd[Xaxis] = inVector1[Yaxis] * inVector2[Zaxis] - inVector1[Zaxis] * inVector2[Yaxis];
	vecVecProd[Yaxis] = inVector1[Zaxis] * inVector2[Xaxis] - inVector1[Xaxis] * inVector2[Zaxis];
	vecVecProd[Zaxis] = inVector1[Xaxis] * inVector2[Yaxis] - inVector1[Yaxis] * inVector2[Xaxis];
}

double RoboticArm::calcAngle(double inVector1[3], double inVector2[3])	//returns the angle between two given vectors
{
	return acos(calcScalProd(inVector1, inVector2) / (calcLength(inVector1) * calcLength(inVector2)));
}

double RoboticArm::toRad(double angleDegree)	//converts the angle from ° to rad
{
	return angleDegree * M_PI / 180;
}

double RoboticArm::toDegree(double angleRad)	//converts the angle from rad to °
{
	return angleRad * 180 / M_PI;
}

void RoboticArm::calcCoordinates()	//calculates from machine angles to coordinates
{
	double oy = cos(mathAngle[BETHA]) * LENGTHUPPERARM;
	double uy = cos(mathAngle[GAMMA] - mathAngle[BETHA]) * LENGTHFOREARM;
	double Ex = -cos(mathAngle[ALFA]) * oy;
	double Ey = -sin(mathAngle[ALFA]) * oy;
	double Ez = sin(mathAngle[BETHA]) * LENGTHUPPERARM;
	calcX = cos(mathAngle[ALFA]) * uy + Ex;
	calcY = sin(mathAngle[ALFA]) * uy + Ey;
	calcZ = sin(mathAngle[GAMMA] - mathAngle[BETHA]) * LENGTHFOREARM + Ez + HEIGHTSHOULDER;
}

bool RoboticArm::calcMachineAngles()	//calculates coordinate to machine angles
{
	// *** fingerCoord -> wristCoord ***
	//since not calculated yet, the wristCoord are exactly the fingerCorrd
	//the fingercoordinates inthe U,V,W-Axis are translatet from [°] to [rad]
	wristCoord[Xaxis] = fingerCoord[Xaxis];
	wristCoord[Yaxis] = fingerCoord[Yaxis];
	wristCoord[Zaxis] = fingerCoord[Zaxis];
	wristCoord[Uaxis] = toRad(fingerCoord[Uaxis]) + M_PI / 2;
	wristCoord[Vaxis] = toRad(fingerCoord[Vaxis]);
	wristCoord[Waxis] = toRad(fingerCoord[Waxis]);

	// *** wristCoord -> mathAngle ***
	//if you understand the following code, please explain me
	//i am just copying what i wrote two years ago
	berx0 = wristCoord[Xaxis];
	bery0 = wristCoord[Yaxis];
	berz0 = wristCoord[Zaxis] - HEIGHTSHOULDER;
	berx1 = berx0;
	bery1 = bery0;
	berz1 = berz0;

	berdxy = sqrt(berx1 * berx1 + bery1 * bery1);
	berdxyz = sqrt(berx1 * berx1 + bery1 * bery1 + berz1 * berz1);

	mathAngle[ALFA] = atan(bery1 / berx1);
	if (berx1 == 0) mathAngle[ALFA] = M_PI / 2;
	if (berx0 < 0) mathAngle[ALFA] += M_PI;
	tempBetha1 = acos((LENGTHUPPERARM * LENGTHUPPERARM + berdxyz * berdxyz - LENGTHFOREARM * LENGTHFOREARM) / (2 * LENGTHUPPERARM * berdxyz));
	tempBetha2 = atan(berz1 / berdxy);
	mathAngle[BETHA] = M_PI - tempBetha1 - tempBetha2;
	tempGamma1 = asin(berdxyz * sin(tempBetha1) / LENGTHFOREARM);
	tempGamma2 = M_PI - tempGamma1;

	//calculate the differences
	mathAngle[GAMMA] = tempGamma1;
	calcCoordinates();
	double diffGamma1 = sqrt(calcX * calcX + calcY * calcY + calcZ * calcZ) - sqrt(wristCoord[Xaxis] * wristCoord[Xaxis] + wristCoord[Yaxis] * wristCoord[Yaxis] + wristCoord[Zaxis] * wristCoord[Zaxis]);
	mathAngle[GAMMA] = tempGamma2;
	calcCoordinates();
	double diffGamma2 = sqrt(calcX * calcX + calcY * calcY + calcZ * calcZ) - sqrt(wristCoord[Xaxis] * wristCoord[Xaxis] + wristCoord[Yaxis] * wristCoord[Yaxis] + wristCoord[Zaxis] * wristCoord[Zaxis]);
	if (diffGamma1 > MAXALLOWEDDIFFERENCE && diffGamma2 > MAXALLOWEDDIFFERENCE)
	{
		sendState(calcError());
		return false;
	}
	if (diffGamma1 < diffGamma2) mathAngle[GAMMA] = tempGamma1;
	else mathAngle[GAMMA] = tempGamma2;

	berhx2 = -sin(wristCoord[Vaxis]) * cos(wristCoord[Uaxis]) * cos(-wristCoord[Waxis]) + sin(wristCoord[Uaxis]) * sin(-wristCoord[Waxis]);
	berhy2 = -sin(wristCoord[Vaxis]) * sin(wristCoord[Uaxis]) * cos(-wristCoord[Waxis]) - cos(wristCoord[Uaxis]) * sin(-wristCoord[Waxis]);
	double berH0[] = { cos(wristCoord[Uaxis]) * cos(wristCoord[Vaxis]), sin(wristCoord[Uaxis]) * cos(wristCoord[Vaxis]), sin(wristCoord[Vaxis]) };
	double berH1[] = { -cos(wristCoord[Uaxis]) * sin(wristCoord[Vaxis]), -sin(wristCoord[Uaxis]) * sin(wristCoord[Vaxis]), cos(wristCoord[Vaxis]) };
	double berH2[] = { berhx2, berhy2, sqrt(1 - berhx2 * berhx2 - berhy2 * berhy2) };
	double berH3[] = { sin(-wristCoord[Uaxis]), cos(-wristCoord[Uaxis]), 0 };
	double berO0[] = { cos(mathAngle[ALFA]) * cos(mathAngle[GAMMA] - mathAngle[BETHA]), sin(mathAngle[ALFA]) * cos(mathAngle[GAMMA] - mathAngle[BETHA]), sin(mathAngle[GAMMA] - mathAngle[BETHA]) };
	double berO1[] = { -cos(mathAngle[ALFA]) * sin(mathAngle[GAMMA] - mathAngle[BETHA]), -sin(mathAngle[ALFA]) * sin(mathAngle[GAMMA] - mathAngle[BETHA]), cos(mathAngle[GAMMA] - mathAngle[BETHA]) };
	double berO3[] = { -sin(mathAngle[ALFA]), -cos(mathAngle[ALFA]), 0 };

	calcVecProd(berO0, berH0);
	for (int a = 0; a < 3; a++)berOH[a] = vecVecProd[a];
	if (berO0[Xaxis] == berH0[Xaxis] && berO0[Yaxis] == berH0[Yaxis] && berO0[Zaxis] == berH0[Zaxis])
	{
		for (int a = 0; a < 3; a++) berOH[a] = berO3[a];
	}
	mathAngle[DELTA] = calcAngle(berO1, berOH);
	mathAngle[EPSILON] = calcAngle(berO0, berH0);
	if (wristCoord[Vaxis] < 0) mathAngle[EPSILON] = mathAngle[EPSILON] * -1;
	mathAngle[ZETA] = calcAngle(berH2, berOH);

	// *** mathAngle -> machineAngle ***
	//Adjusts the calculated mathAngles to the robot
	machineAngle[0] = toDegree(mathAngle[ALFA]);
	machineAngle[1] = 15 - toDegree(mathAngle[GAMMA]) + toDegree(mathAngle[BETHA]);
	machineAngle[2] = 180 - toDegree(mathAngle[BETHA]);
	machineAngle[3] = toDegree(mathAngle[DELTA]);
	machineAngle[4] = 90 + toDegree(mathAngle[EPSILON]);
	machineAngle[5] = 180 - toDegree(mathAngle[ZETA]);

	//verify machineAngles
	for (int a = 0; a < 6; a++)
	{
		if (machineAngle[a] < axisLimit[a][LOWERLIMIT] || machineAngle[a] > axisLimit[a][UPPERLIMIT])
		{
			sendState(calcError());
			return false;
		}
	}
	return true;
}

String RoboticArm::calcError()
{
	//returns an error-String with the unreachable coordinates
	return "Cannot reach position " + coordToString(fingerCoord[0], fingerCoord[1], fingerCoord[2], fingerCoord[3], fingerCoord[4], fingerCoord[5]);
}