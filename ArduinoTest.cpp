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


int DATA_BUFFER_LENGTH = 5;

VL53L1X TOF;
Sensor_IMU IMU;

ST_HW_HC_SR04* Ultrasonic;

AF_DCMotor rightMotor(4); //right
AF_DCMotor leftMotor(3); //left

unsigned long startMillis = 0, currMillis = 0, deltaMillis = 0, IMU_lastReadTime = 0, lastAlignTime = 0;  //some global variables available anywhere in the program
float testZ;
int hitTime;
float offsetXAngle=0, offsetYAngle=0, offsetZAngle=0;


void setup() {
	Serial.begin(115200);
    while(!Serial); // Wait for the Serial connection;
	Wire.begin();

	Wire.setClock(400000); // use 400 kHz I2C

	Serial.println("IMU Object Test");

	IMU.initialize();

	 // TRIG = 40, ECHO= 42
	Ultrasonic = new ST_HW_HC_SR04(40, 42);



	//String offsets = "Offsets: " + (String) offsetXAngle + ", " + (String)offsetYAngle +", "+(String)offsetXAngle;

	//Serial.println(offsets);

	startMillis = millis();  //initial start time
//	rightMotor.run(FORWARD);
//	leftMotor.run(FORWARD);
//	rightMotor.setSpeed(100);
//	leftMotor.setSpeed(100);
}


void loop(void)
{

	currMillis = millis();
	deltaMillis = currMillis - startMillis;


	if((currMillis - lastAlignTime) > 250){
		//alignToWall(testUR);
		IMUData tempData = IMU.getData();
		tempData.angle[2]-=offsetZAngle;
		IMU.printData(tempData);

		//IMU.IMU->update();


		//Serial.print("Time: ");
		//Serial.println(currMillis - lastAlignTime);
		lastAlignTime = currMillis;

	}
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
	if(currMillis - IMU_lastReadTime > 200){
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

	rightMotor.run(FORWARD);
	leftMotor.run(FORWARD);

	//Distances are in cm, limit precision to 1cm
	int initDist = 0, currDist = 0, initError = 0;

	for (int i = 0; i < 50;  i++){
		hitTime = sensor->getHitTime();
		initDist += (int)(hitTime / 29);
	}
	initDist /= 50;
	Serial.println(initDist);

	//Move 'straight'
	rightMotor.setSpeed(100);
	leftMotor.setSpeed(100);

	currDist = (int)(sensor->getHitTime() / 29);
	initError = currDist - initDist;

	//Try turning left to minimize error
	while((abs(currDist - initDist) > 1) && (abs(initError) > abs(currDist - initDist))){
		leftMotor.run(BACKWARD);
		leftMotor.setSpeed(100);
		delay(100);
		currDist = (int)(sensor->getHitTime() / 29);
	}

	//Turning left didn't work, try turning right
	if(abs(initError) > abs(currDist - initDist)){
		while((abs(currDist - initDist) > 1)){
			leftMotor.run(FORWARD);
			leftMotor.setSpeed(100);
			rightMotor.run(BACKWARD);
			rightMotor.setSpeed(100);
			delay(100);
			currDist = (int)(sensor->getHitTime() / 29);
		}
	}

	rightMotor.run(FORWARD);
	leftMotor.run(FORWARD);
	rightMotor.setSpeed(100);
	leftMotor.setSpeed(100);
}

