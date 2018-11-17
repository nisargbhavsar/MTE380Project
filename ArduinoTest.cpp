#include <Arduino.h>

#include <Wire.h>

//Sensor Libraries
#include "VL53L1X.h" //TOF
#include "MPU6050_tockn.h" //IMU
#include "SensorIMU.h"
#include "AFMotor.h"
#include "ST_HW_HC_SR04.h"

//Function Prototypes
void printCurrIMUData(unsigned long);
void alignToWall(ST_HW_HC_SR04*);
void turn90Deg(bool);
//void setMotorSpeeds(int rightMotorDir, int rightMotorSpeed, int leftMotorDir, int leftMotorSpeed);
void setLeftMotor(int leftMotorDir, int leftMotorSpeed);
void setRightMotor(int rightMotorDir, int rightMotorSpeed);
void stopLeftMotor();
void stopRightMotor();

const int ROT_SPEED = 255;

const int BWD = 1;
const int FWD = 0;
const int ALIGN_TOL = 1;
const int MS_ROTATE = 200;

VL53L1X TOF;
Sensor_IMU IMU;

ST_HW_HC_SR04* Ultrasonic;

AF_DCMotor rightMotor(4); //right
AF_DCMotor leftMotor(3); //left

unsigned long startMillis = 0, currMillis = 0, deltaMillis = 0, IMU_lastReadTime = 0, lastAlignTime = 0;  //some global variables available anywhere in the program
float testZ;
int hitTime;
float offsetXAngle=0, offsetYAngle=0, offsetZAngle=0;

// Right and left motors
int RightMotorDir = 12, RightMotorBrake = 9, LeftMotorDir = 13, LeftMotorBrake = 8, RightMotorSpeed = 3, LeftMotorSpeed = 11;


void setup() {
	Serial.begin(115200);
    while(!Serial); // Wait for the Serial connection;
	Wire.begin();

	Wire.setClock(400000); // use 400 kHz I2C

	//Serial.println("IMU Object Test");

	//IMU.initialize();

	 // TRIG = 3, ECHO= 2
	Ultrasonic = new ST_HW_HC_SR04(3, 2);

	//delay(1000);

	//String offsets = "Offsets: " + (String) offsetXAngle + ", " + (String)offsetYAngle +", "+(String)offsetXAngle;

	//Serial.println(offsets);

	startMillis = millis();  //initial start time

	pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
	pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin

	//Setup Channel B
	pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
	pinMode(LeftMotorBrake, OUTPUT);  //Initiates Brake Channel A pin

//	// turn left
//	setLeftMotor(BWD, 75);
//	setRightMotor(FWD, 75);
//
//	delay(1000);
//	stopLeftMotor();
//	stopRightMotor();
//	delay(3000);
//
//	// turn right
//	setLeftMotor(FWD, 75);
//	setRightMotor(BWD, 75);
//	delay(1000);
//	stopLeftMotor();
//	stopRightMotor();


	//setLeftMotor(FWD, 255);
	//setRightMotor(FWD,255);

	String temp = "Aligning to wall";
	Serial.println(temp);
	alignToWall(Ultrasonic);
}


void loop(void)
{

//	currMillis = millis();
//	deltaMillis = currMillis - startMillis;
//
//	delay(4000);
//		String temp = "Aligning to wall";
//		Serial.println(temp);
//		alignToWall(Ultrasonic);
//		//IMUData tempData = IMU.getData();
//		//tempData.angle[2]-=offsetZAngle;
//		//IMU.printData(tempData);
//
//		//IMU.IMU->update();
//
//
//		//Serial.print("Time: ");
//		//Serial.println(currMillis - lastAlignTime);
//		lastAlignTime = currMillis;



	//forward @ full speed


}

void turn90Deg(bool isLeft){
	// set motor speeds
	leftMotor.setSpeed(0);
	rightMotor.setSpeed(0);

	float TOL = 1, currZ = 0, initZ = 0;
	IMUData data;
	for (int i = 0; i < 10; i++) {
		data = IMU.getData();
	//	Serial.print(IMU->getAngleZ());
	//	Serial.print(", ");
		initZ += data.angle[2];
	}
	initZ /= 10;
	currZ = initZ;


	if (isLeft) {
		leftMotor.run(BACKWARD);
		rightMotor.run(FORWARD);
	}
	else {
		leftMotor.run(FORWARD);
		rightMotor.run(BACKWARD);
	}

	leftMotor.setSpeed(100);
	rightMotor.setSpeed(100);

	while(abs(abs(currZ-initZ) - 90) > TOL) {
		delay(100);
		data = IMU.getData();

		currZ = data.gyro[2];

		float currX = data.gyro[0];
		float currY = data.gyro[1];
		String currOrientation = "currX: "+ (String)currX + "currY: " + (String)currY + "currZ: " + (String)currZ;
		Serial.println(currOrientation);
	}

	// set motor speeds
	leftMotor.run(FORWARD);
	rightMotor.run(FORWARD);
	leftMotor.setSpeed(0);
	rightMotor.setSpeed(0);

}


/*	Print current imu data in CSV format
 * 	HEADER:
 * 		AccX, AccY, AccZ, GyroX, GyroY, GyroZ, AccAngleX, AccAngleY, AccAngleZ, GyroAngleX, GyroAngleY, GyroAngleZ, FinalAngleX, FinalAngleY, FinalAngleZ
 * 	Only call if time between calls exceeds 1000 ms
 */
void printCurrIMUData(unsigned long currMillis){
	if((currMillis - IMU_lastReadTime) > 200){
//		Serial.print(testIMUPtr2->getAccX());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getAccY());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getAccZ());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getGyroX());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getGyroY());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getGyroZ());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getAccAngleX());
//		Serial.print(", ");
//		Serial.print(testIMUPtr2->getAccAngleY());
//		Serial.print(", ");
		Serial.print(IMU.IMU->getGyroAngleX());
		Serial.print(", ");
		Serial.print(IMU.IMU->getGyroAngleY());
		Serial.print(", ");
		Serial.print(IMU.IMU->getGyroAngleZ());
		Serial.print(", ");
		Serial.print(IMU.IMU->getAngleX());
		Serial.print(", ");
		Serial.print(IMU.IMU->getAngleY());
		Serial.print(", ");
		Serial.println(IMU.IMU->getAngleZ());
		IMU_lastReadTime = currMillis;
	}
}
/*
 * Test implementation of following a wall using solely the Ultrasonic sensor
 *
 */
void alignToWall(ST_HW_HC_SR04* sensor){
	bool turnLeft = false;
	stopLeftMotor();
	stopRightMotor();

	//Distances are in cm, limit precision to 1cm
	double initDist = 0, rightDist = 0, leftDist = 0, currDist = 0, leftError = 0, rightError = 0, prevDist = 0;

	for (int i = 0; i < 50;  i++){
		hitTime = sensor->getHitTime();
		initDist += (double)(hitTime / 29.1);
	}
	initDist /= 50;

//	String temp = "initDist: " + (String)initDist + " ";
//	Serial.print(temp);
	//Try turning left to minimize error
	setLeftMotor(BWD, ROT_SPEED);
	setRightMotor(FWD, ROT_SPEED);

	delay(MS_ROTATE);

	stopLeftMotor();
	stopRightMotor();

	delay(100);

	leftDist = 0;
	for (int i = 0; i < 50;  i++){
		hitTime = sensor->getHitTime();
		leftDist += (double)(hitTime / 29.1);
	}
	leftDist /= 50;

//	temp = "leftDist: " + (String)leftDist + " ";
//	Serial.print(temp);

	delay(100);
	//Try turning right to minimize error
	setLeftMotor(FWD, ROT_SPEED);
	setRightMotor(BWD, ROT_SPEED);

	delay(MS_ROTATE);
	stopLeftMotor();
	stopRightMotor();
	delay(500);

	setLeftMotor(FWD, ROT_SPEED);
	setRightMotor(BWD, ROT_SPEED);

	delay(MS_ROTATE);
	stopLeftMotor();
	stopRightMotor();
	delay(100);

	rightDist = 0;
	for (int i = 0; i < 50;  i++){
		hitTime = sensor->getHitTime();
		rightDist += (double)(hitTime / 29.1);
	}
	rightDist /= 50;

	String temp = "leftDist: " + (String)leftDist + " initDist: " + (String)initDist + " rightDist: " + (String)rightDist;
	Serial.println(temp);

	if((leftDist < 2 || rightDist < 2 || initDist < 2 ) || abs(leftDist - rightDist) < ALIGN_TOL){
		if(leftDist < 2 || rightDist < 2 || initDist < 2 )
			temp = "unknown state";
		else
				temp = "Aligned already";
		Serial.println(temp);

		//Turn left to realign with initial position
		delay(500);
		setLeftMotor(BWD, ROT_SPEED);
		setRightMotor(FWD, ROT_SPEED);
		delay(MS_ROTATE);
		stopLeftMotor();
		stopRightMotor();

	}

	else {
		if(leftDist > initDist && initDist > rightDist){
			//turn right to realign with wall
//			setLeftMotor(FWD, ROT_SPEED);
//			setRightMotor(BWD, ROT_SPEED);
			temp = "Turn Right";
			Serial.println(temp);
			turnLeft = false;
		}
		else if (leftDist > rightDist && rightDist > initDist){
			//turn left to realign with wall
//			setLeftMotor(BWD, ROT_SPEED);
//			setRightMotor(FWD, ROT_SPEED);
			temp = "Turn Left";
			Serial.println(temp);
			turnLeft = true;
		}
		else if (leftDist < rightDist && leftDist > initDist){
			//turn left to realign with wall
//			setLeftMotor(BWD, ROT_SPEED);
//			setRightMotor(FWD, ROT_SPEED);
			temp = "Turn Left";
			Serial.println(temp);
			turnLeft = false;
		}
		else if (leftDist < rightDist && rightDist > initDist){
			//turn left to realign with wall
//			setLeftMotor(BWD, ROT_SPEED);
//			setRightMotor(FWD, ROT_SPEED);
			temp = "Turn Left";
			Serial.println(temp);
			turnLeft = true;
		}
		currDist = rightDist;
		delay(100);

		if(turnLeft){
			setLeftMotor(BWD, ROT_SPEED);
			setRightMotor(FWD, ROT_SPEED);
		}
		else{
			setLeftMotor(FWD, ROT_SPEED);
			setRightMotor(BWD, ROT_SPEED);
		}
		temp = "Going inside while loop";
		Serial.println(temp);

		do{
			prevDist = currDist;
			delay(150);
			currDist = 0;

			while(currDist < 2){
				for(int i = 0; i < 50; i++){
					hitTime = sensor->getHitTime();
					currDist += (double)(hitTime / 29.1);
				}
				currDist /=50;
			}
			temp = "PrevDist: " + (String)prevDist + " currDist: " +(String) currDist;
			Serial.println(temp);
		}while((currDist - prevDist < 0));

		temp = "Outof loop";
		Serial.println(temp);
	}
	stopLeftMotor();
	stopRightMotor();
}

void setLeftMotor(int leftMotorDir, int leftMotorSpeed){
  digitalWrite(LeftMotorDir, leftMotorDir); //Establishes forward direction of Channel A
  digitalWrite(LeftMotorBrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(LeftMotorSpeed, leftMotorSpeed);   //Spins the motor on Channel A at full speed
}

void setRightMotor(int rightMotorDir, int rightMotorSpeed){
  digitalWrite(RightMotorDir, rightMotorDir); //Establishes forward direction of Channel A
  digitalWrite(RightMotorBrake, LOW);   //Disengage the Brake for Channel A
  analogWrite(RightMotorSpeed, rightMotorSpeed);   //Spins the motor on Channel A at full speed
}

void stopLeftMotor() {
	setLeftMotor(FWD, 0);
	digitalWrite(LeftMotorBrake, HIGH);
}
void stopRightMotor(){
	setRightMotor(FWD, 0);
	digitalWrite(RightMotorBrake, HIGH);
}
