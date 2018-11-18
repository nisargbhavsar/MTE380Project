#include <Arduino.h>
#include <Wire.h>

//Sensor Libraries
#include "VL53L1X.h" //TOF
//#include "SparkFun_VL53L1X_Arduino_Library.h"
#include "MPU6050_tockn.h" //IMU
#include "SensorIMU.h"
#include "AFMotor.h"
#include "ST_HW_HC_SR04.h"

#define encoder0PinA  2
#define encoder0PinB  3


//Function Prototypes
double getUltrasonicReading(ST_HW_HC_SR04*, int);
void printCurrIMUData(unsigned long);
void alignToWall(ST_HW_HC_SR04*);
void alignToWallWithTwo(ST_HW_HC_SR04*, ST_HW_HC_SR04*);

void turn90Deg(bool);

//void setMotorSpeeds(int rightMotorDir, int rightMotorSpeed, int leftMotorDir, int leftMotorSpeed);
void setLeftMotor(int leftMotorDir, int leftMotorSpeed);
void setRightMotor(int rightMotorDir, int rightMotorSpeed);
void stopLeftMotor();
void stopRightMotor();

void doEncoderA();
void doEncoderB();

const int ROT_SPEED = 60;

const int BWD = 0;
const int FWD = 1;
const double ALIGN_TOL = 0.5;
const int MS_ROTATE = 200;

volatile unsigned int encoder0Pos = 0;


VL53L1X TOF;
Sensor_IMU IMU;

ST_HW_HC_SR04* UltrasonicLeft1;
ST_HW_HC_SR04* UltrasonicLeft2;


unsigned long startMillis = 0, currMillis = 0, deltaMillis = 0, IMU_lastReadTime = 0, lastAlignTime = 0;  //some global variables available anywhere in the program
float testZ;
int hitTime;
float offsetXAngle=0, offsetYAngle=0, offsetZAngle=0;

// Right and left motors
int RightMotorDir = 12, RightMotorBrake = 9, LeftMotorDir = 13, LeftMotorBrake = 8, RightMotorSpeed = 3, LeftMotorSpeed = 11;

//0,1,2,4,7,10


void setup() {
	Serial.begin(115200);
    while(!Serial); // Wait for the Serial connection;
	Wire.begin();

	Wire.setClock(400000); // use 400 kHz I2C
//	TOF.begin();
	//Serial.println("IMU Object Test");

	//IMU.initialize();

	//ULTRASONICS
	 // TRIG = 1, ECHO= 0
	//UltrasonicLeft1 = new ST_HW_HC_SR04(10, 7);
	//UltrasonicLeft2 = new ST_HW_HC_SR04(4, 2);

	//delay(1000);

	//String offsets = "Offsets: " + (String) offsetXAngle + ", " + (String)offsetYAngle +", "+(String)offsetXAngle;

	//Serial.println(offsets);

	startMillis = millis();  //initial start time

	//MOTOR CONTROLLER PIN SETUP
	pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
	pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin

	//Setup Channel B
	pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
	pinMode(LeftMotorBrake, OUTPUT);  //Initiates Brake Channel A pin

	setLeftMotor (FWD,200);
	setRightMotor(FWD,200);

	//String temp = "Aligning to wall";
	//Serial.println(temp);
	//alignToWallWithTwo(UltrasonicLeft1, UltrasonicLeft2);

	//ENCODER SETUP
	pinMode(encoder0PinA, INPUT);
	pinMode(encoder0PinB, INPUT);

	// encoder pin on interrupt 0 (pin 2)
	attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);

	// encoder pin on interrupt 1 (pin 3)
	attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);

	Serial.begin (9600);

}


void loop(void)
{
	while (TOF.newDataReady() == false)
		delay(5);

	int distance = TOF.getDistance(); //Get the result of the measurement from the sensor

	Serial.print("Distance(mm): ");
	Serial.print(distance);

	//	alignToWallWithTwo(UltrasonicLeft1, UltrasonicLeft2);
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
//	leftMotor.setSpeed(0);
//	rightMotor.setSpeed(0);
//
//	float TOL = 1, currZ = 0, initZ = 0;
//	IMUData data;
//	for (int i = 0; i < 10; i++) {
//		data = IMU.getData();
//	//	Serial.print(IMU->getAngleZ());
//	//	Serial.print(", ");
//		initZ += data.angle[2];
//	}
//	initZ /= 10;
//	currZ = initZ;
//
//
//	if (isLeft) {
//		leftMotor.run(BACKWARD);
//		rightMotor.run(FORWARD);
//	}
//	else {
//		leftMotor.run(FORWARD);
//		rightMotor.run(BACKWARD);
//	}
//
//	leftMotor.setSpeed(100);
//	rightMotor.setSpeed(100);
//
//	while(abs(abs(currZ-initZ) - 90) > TOL) {
//		delay(100);
//		data = IMU.getData();
//
//		currZ = data.gyro[2];
//
//		float currX = data.gyro[0];
//		float currY = data.gyro[1];
//		String currOrientation = "currX: "+ (String)currX + "currY: " + (String)currY + "currZ: " + (String)currZ;
//		Serial.println(currOrientation);
//	}
//
//	// set motor speeds
//	leftMotor.run(FORWARD);
//	rightMotor.run(FORWARD);
//	leftMotor.setSpeed(0);
//	rightMotor.setSpeed(0);

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

	initDist = getUltrasonicReading(sensor, 50);

	//Try turning left to minimize error
	setLeftMotor(BWD, ROT_SPEED);
	setRightMotor(FWD, ROT_SPEED);

	delay(MS_ROTATE);

	stopLeftMotor();
	stopRightMotor();

	delay(MS_ROTATE);

	leftDist = getUltrasonicReading(sensor, 50);

	delay(MS_ROTATE);

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
	delay(MS_ROTATE);

	rightDist = getUltrasonicReading(sensor, 50);

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
			temp = "Turn Right";
			Serial.println(temp);
			turnLeft = false;
		}
		else if (leftDist > rightDist && rightDist > initDist){
			//turn left to realign with wall
			temp = "Turn Left";
			Serial.println(temp);
			turnLeft = true;
		}
		else if (leftDist < rightDist && leftDist > initDist){
			//turn left to realign with wall
			temp = "Turn Left";
			Serial.println(temp);
			turnLeft = false;
		}
		else if (leftDist < rightDist && rightDist > initDist){
			//turn left to realign with wall
			temp = "Turn Left";
			Serial.println(temp);
			turnLeft = true;
		}
		currDist = rightDist;
		delay(MS_ROTATE);

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
				currDist = getUltrasonicReading(sensor, 50);
			}
			temp = "PrevDist: " + (String)prevDist + " currDist: " +(String) currDist;
			Serial.println(temp);
		}while(currDist - prevDist > ALIGN_TOL);

		temp = "Outof loop";
		Serial.println(temp);
	}
	stopLeftMotor();
	stopRightMotor();
}

void alignToWallWithTwo(ST_HW_HC_SR04* sensor1, ST_HW_HC_SR04* sensor2){
	stopLeftMotor();
	stopRightMotor();
	bool turnLeft = false;
	double leftDist = getUltrasonicReading(sensor1, 10);
	double rightDist = getUltrasonicReading(sensor2, 10);

	String temp = "leftDist: " + (String) leftDist + "rightDist: " + (String)rightDist;
	Serial.println(temp);
	temp = "abs(leftDist - rightDist): " + (String)(abs(leftDist - rightDist));
	Serial.println(temp);
	temp = (String)((abs(leftDist - rightDist) - ALIGN_TOL) > 0);
	Serial.println(temp);

	if((abs(leftDist - rightDist) - ALIGN_TOL) > 0){
		if(rightDist < leftDist){
			//turn right
			setLeftMotor(BWD, ROT_SPEED);
			setRightMotor(FWD, ROT_SPEED);
			temp = "Turn right";
			Serial.println(temp);
		}
		else{
			setLeftMotor(FWD, ROT_SPEED);
			setRightMotor(BWD, ROT_SPEED);
			temp = "Turn left";
			Serial.println(temp);
		}
		do{
			leftDist = getUltrasonicReading(sensor1, 10);
			rightDist = getUltrasonicReading(sensor2, 10);
			//delay(50);
			String temp = "leftDist: " + (String) leftDist + "rightDist: " + (String)rightDist;
			Serial.println(temp);
		}while(abs(leftDist - rightDist) > ALIGN_TOL);
	}
	else{
		String temp = "Already aligned";
		Serial.println(temp);
		setLeftMotor(FWD, ROT_SPEED);
		setRightMotor(FWD, ROT_SPEED);

	}
//	stopRightMotor();
//	stopLeftMotor();
}

double getUltrasonicReading(ST_HW_HC_SR04* sensor, int numRead){
	double returnVal = 0;
	int hitTime = 0;

	for (int i = 0; i < numRead;  i++){
		hitTime = sensor->getHitTime();
		returnVal += (double)(hitTime / 29.1);
	}
	returnVal /= numRead;
	return returnVal;
}

void setLeftMotor(int leftMotorDir, int leftMotorSpeed){
  digitalWrite(LeftMotorDir, leftMotorDir);
  digitalWrite(LeftMotorBrake, LOW);
  analogWrite(LeftMotorSpeed, leftMotorSpeed);
}

void setRightMotor(int rightMotorDir, int rightMotorSpeed){
  digitalWrite(RightMotorDir, rightMotorDir);
  digitalWrite(RightMotorBrake, LOW);
  analogWrite(RightMotorSpeed, rightMotorSpeed);
}

void stopLeftMotor() {
	setLeftMotor(FWD, 0);
	digitalWrite(LeftMotorBrake, HIGH);
}
void stopRightMotor(){
	setRightMotor(FWD, 0);
	digitalWrite(RightMotorBrake, HIGH);
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }

  // Look for a high-to-low on channel B

  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

