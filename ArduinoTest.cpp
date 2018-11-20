#include <Arduino.h>
#include <Wire.h>

//Sensor Libraries
#include "VL53L1X.h" //TOF
#include "SensorIMU.h"
#include "ST_HW_HC_SR04.h"

const unsigned int TOF_MEASURE_TIME = 50000;


//#include "vl53l1_api.h"

// FUNCTION PROTOTYPES
double getUltrasonicReading(ST_HW_HC_SR04*, int);
//void printCurrIMUData(unsigned long);
//void alignToWall(ST_HW_HC_SR04*);
//void alignToWallWithTwo(ST_HW_HC_SR04*, ST_HW_HC_SR04*);
void rotate90Deg(int);
void setLeftMotor(int leftMotorDir, int leftMotorSpeed);
void setRightMotor(int rightMotorDir, int rightMotorSpeed);
void stopMotorsXYZ();

void stopLeftMotor();
void stopRightMotor();
void driveStraight(int, int);
void locateTarget();
bool detectTarget(double [], int);
void printArr(double arr[], int size_arr);
void addToBuffer(double data[], int size_arr, double new_data);
double getMeasurements(int);
void driveStraightToDist(int dir, int motor_speed, int distance);
void correctOrientation();
void chaseDownTarget();
void intializeL289NMotorShield();

// CONSTANTS
const int ROT_SPEED = 60;
const int STRAIGHT_SCANNING_SPEED = 100;
const int STRAIGHT_SPEED = 200;
const int MAX_MOTOR_SPEED = 250;
const int DOWN_HUMP_SPEED = 100;

const int BWD = 0;
const int FWD = 1;
const int RIGHT = 0;
const int LEFT = 1;
const double ALIGN_TOL = 0.5;
const int WALL_DIST = 2000; // in mm
const int TARGET_TOL = 200;
const int REQ_OBJ_DETECTIONS = 5;

const int FACING_UP = 1;
const int FACING_STRAIGHT = 0;
const int FACING_DOWN = 2;

const int ROTATE_TOL = 1;

const int MS_ROTATE = 200;
//Previous motor shield defns
//RightMotorBrake = 9, RightMotorSpeed = 3;
//int LeftMotorDir = 13, LeftMotorBrake = 8, LeftMotorSpeed = 11;

VL53L1X TOF;

int status;
int RightMotorEnable = 10, RightMotorDir1 = 9, RightMotorDir2 = 8, LeftMotorEnable = 11, LeftMotorDir1 = 7, LeftMotorDir2 = 6;

ST_HW_HC_SR04* UltrasonicFront;
Sensor_IMU IMU;

void setup() {
  intializeL289NMotorShield();
  stopLeftMotor();
  stopRightMotor();

  Serial.begin(115200);
  while(!Serial); // Wait for the Serial connection;
  Wire.begin();

  Wire.setClock(400000); // use 400 kHz I2C
  IMU.initialize();

//  UltrasonicFront = new ST_HW_HC_SR04(4,3);
//
////  //MOTOR CONTROLLER PIN SETUP
////  pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
////  pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin
////
////  //Setup Channel B
////  pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
////  pinMode(LeftMotorBrake, OUTPUT);  //Initiates Brake Channel A pin
//
//
//  TOF.setDistanceMode(2);
//  //bool test = TOF.setMeasurementTimingBudget(TOF_MEASURE_TIME);
//  TOF.softReset();


  // SET MOTOR SPEEDS
//    setLeftMotor (FWD, MAX_MOTOR_SPEED);
//    setRightMotor(FWD, MAX_MOTOR_SPEED);
//    Serial.print("moved motors");
//
//    delay(500);
    rotate90Deg(LEFT);
    //Serial.print("Done turning Left");


  // get TOS measurement
  //double sideTofReading = getMeasurements(3);
  //Serial.println((String)sideTofReading);
}
//bool test = 0;
void loop() {
//    rotate90Deg(test);
//    test = !test;
//    delay(500);
  // OVERALL MISSION CONTROL

//  // Get to wall
//  correctOrientation();
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 1710); //  distance from middle of metal wall to side wall
//  rotate90Deg(RIGHT);
//  correctOrientation();
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 2000); //  distance from metal wall to back wall
//  correctOrientation();
//
  // Get on wall
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  delay(1000);
//  while (IMU.onWall() != FACING_UP);
//
//  Serial.println("now facing up");
//  stopLeftMotor();
//  stopRightMotor();

  // Get over wall
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 800);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while (IMU.onWall() != FACING_STRAIGHT);
//  Serial.println("now facing straight");
//  driveStraight(FWD, DOWN_HUMP_SPEED);
//  while (IMU.onWall() != FACING_DOWN);
//  Serial.println("now facing down");
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while (IMU.onWall() == FACING_DOWN);
//  Serial.println("now facing down");
//  stopLeftMotor();
//  stopRightMotor();
//
//
////  // get to initial target searching position
////  correctOrientation();
//  rotate90Deg(LEFT);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED); // get to side
//  delay(500);
//  stopMotorsXYZ();
//  rotate90Deg(RIGHT);
//  correctOrientation();
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 2300); // get to corner
////
////  // locate target
//  locateTarget();
//  rotate90Deg(RIGHT);
//
////  // get to target
//  chaseDownTarget();
//
////  // get back to wall
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 2300);
//  rotate90Deg(RIGHT);
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  rotate90Deg(LEFT);
//  correctOrientation();
////
//  // Get on wall
//    driveStraight(FWD, MAX_MOTOR_SPEED);
//    while (IMU.onWall() != FACING_UP);
//    Serial.println("now facing up");
//    stopLeftMotor();
//    stopRightMotor();
//
//    // Get over wall
//    driveStraightToDist(FWD, MAX_MOTOR_SPEED, 800);
//    correctOrientation();
//    driveStraight(FWD, MAX_MOTOR_SPEED);
//    while (IMU.onWall() != FACING_STRAIGHT);
//    Serial.println("now facing straight");
//    driveStraight(FWD, DOWN_HUMP_SPEED);
//    while (IMU.onWall() != FACING_DOWN);
//    Serial.println("now facing down");
//    driveStraight(FWD, MAX_MOTOR_SPEED);
//    while (IMU.onWall() == FACING_DOWN);
//    Serial.println("now facing down");
//    stopLeftMotor();
//    stopRightMotor();
//
//
//  // get back onto initial base
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  stopMotorsXYZ();
//  delay(500);
//  rotate90Deg(LEFT);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  delay(2000);
//  stopMotorsXYZ();
//
  delay(10000);
}

// MOTOR CONTROL
void setLeftMotor(int leftMotorDir, int leftMotorSpeed){
  if(leftMotorDir == 0){
     digitalWrite(LeftMotorDir1, LOW);
    digitalWrite(LeftMotorDir2, HIGH);
  }
  else{
    digitalWrite(LeftMotorDir1, HIGH);
    digitalWrite(LeftMotorDir2, LOW);
  }
  // set speed out of possible range 0~255
  int speed = leftMotorSpeed%256;
  analogWrite(LeftMotorEnable, speed);
}

void setRightMotor(int rightMotorDir, int rightMotorSpeed){
  // this function will run the LeftMotor
   if(rightMotorDir == 0){
    digitalWrite(RightMotorDir1, LOW);
    digitalWrite(RightMotorDir2, HIGH);
  }
  else{
    digitalWrite(RightMotorDir1, HIGH);
    digitalWrite(RightMotorDir2, LOW);
  }
  // set speed out of possible range 0~255
  int speed = rightMotorSpeed%256;
  analogWrite(RightMotorEnable, speed);
}

void stopLeftMotor() {
  setLeftMotor(FWD, 0);
//  digitalWrite(LeftMotorBrake, HIGH);
}
void stopRightMotor(){
  setRightMotor(FWD, 0);
//  digitalWrite(RightMotorBrake, HIGH);
}

void driveStraight(int dir, int motor_speed) {
  // maintaining straightness with two back TOF to be incorporated
  setLeftMotor (dir,motor_speed);
  setRightMotor(dir,motor_speed);
}

void driveStraightToDist(int dir, int motor_speed, int distance) {
  // maintaining straightness with two back TOF to be incorporated
  setLeftMotor (dir,motor_speed);
  setRightMotor(dir,motor_speed);

//  // REPLACE THIS
//  delay(500);
//  stopMotorsXYZ();
}

void stopMotorsXYZ(){
  stopLeftMotor();
  stopRightMotor();
}

void correctOrientation() {
  // INSERT CODE HERE
}

// Motor shield
//void intializeArduinoMotorControllerPins() {
//   //MOTOR CONTROLLER PIN SETUP
//  pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
//  pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin
//  //Setup Channel B
//  pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
//  pinMode(LeftMotorBrake, OUTPUT); //Initiates Brake Channel A pin
// }

void intializeL289NMotorShield() {
  pinMode(RightMotorEnable, OUTPUT);
  pinMode(LeftMotorEnable, OUTPUT);
  pinMode(RightMotorDir1, OUTPUT);
  pinMode(RightMotorDir2, OUTPUT);
  pinMode(LeftMotorDir1, OUTPUT);
  pinMode(LeftMotorDir2, OUTPUT);
 }

// TOF
double getMeasurements(int measurements)
{
	double totalDist = 0;

	for(int i =0; i < measurements; i++){
		totalDist += (double)TOF.getDistance();
	}

	return totalDist/measurements;
}

// COURSE FUNCTIONS

// drives straight until it sees the target
void locateTarget() {

  // buffer for TOF readings
  int size_tof_buf = 10;
  double tofReadings[size_tof_buf] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  bool foundTarget = false;

  // go forward
  driveStraight(FWD, STRAIGHT_SCANNING_SPEED);

  // until object detected
  double sideTofReading = 0;

  for (int i = 0; i < size_tof_buf; i++) {
    sideTofReading = getMeasurements(3);
    addToBuffer(tofReadings, size_tof_buf, sideTofReading);
  }

  while(!foundTarget) {
    sideTofReading = getMeasurements(3);
    addToBuffer(tofReadings, size_tof_buf, sideTofReading);
    printArr(tofReadings, size_tof_buf);
    foundTarget = detectTarget(tofReadings, size_tof_buf);
    delay(100);
  }

  // stop motors
  stopMotorsXYZ();
  Serial.println("Detected target!");
}

void chaseDownTarget() {
      //Turn right to realign with initial position
      Serial.println("start chasing");
    delay(500);

    setRightMotor(FWD, 150);
    setLeftMotor(BWD, 150);

    delay(MS_ROTATE);
    stopLeftMotor();
    stopRightMotor();
    delay(500);
    //start go forward
    driveStraight(FWD, 180);


    //stop when within the range
  bool reachTarget = false;
  while(!reachTarget) {
    double distanceTarget = getUltrasonicReading(UltrasonicFront, 5);

    Serial.println(distanceTarget);
    if((distanceTarget < 10) && (distanceTarget > 0 )){ // FIX SO THAT IT HAS LESS THAN ZERO TOO
      reachTarget = 1;
     }
  }
    stopMotorsXYZ();
  Serial.println("stopped at target!");
}

// HELPER FUNCTIONS

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

// read in a buffer of last 10 TOF scans; return true if target detected
bool detectTarget(double data[], int size_data) {

  int num_obj_detections = 0;
  for (int i = 0; i < size_data; i++) {
    if (WALL_DIST - data[size_data - i] > TARGET_TOL) {
      num_obj_detections++;
    }
  }

  return num_obj_detections > REQ_OBJ_DETECTIONS;
}

void printArr(double arr[], int size_arr){
  Serial.print("ARR: ");
  for (int i = 0; i < size_arr; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void addToBuffer(double data[], int size_arr, double new_data){
  for (int i = 0; i < size_arr - 1; i++) {
    data[i] = data[i+1];
  }
  data[size_arr - 1] = new_data;
}

void rotate90Deg(int isLeft){
  stopLeftMotor();
  stopRightMotor();
  delay(MS_ROTATE);

  if(isLeft)
    Serial.println("Turning Left");
  else
    Serial.println("Turning Right");

  float currZ = 0, initZ = 0;

  IMUData data;
  for (int i = 0; i < 10; i++) {
    data = IMU.getData();
  //  Serial.print(IMU->getAngleZ());
  //  Serial.print(", ");
    initZ += data.angle[2];
  }
  initZ /= 10;

  currZ = 0;
  if (isLeft) {
    setLeftMotor(BWD, ROT_SPEED);
    setRightMotor(FWD, ROT_SPEED);
  }
  else {
    setLeftMotor(FWD, ROT_SPEED);
    setRightMotor(BWD, ROT_SPEED);
  }
  String temp;
  while((abs(currZ-initZ) - 80) < (ROTATE_TOL*2)) {
    delay(MS_ROTATE);
    data = IMU.getData();
    currZ = data.angle[2];
  //maybe change to:
//currZ = data.gyro[2];
    Serial.println(currZ);
//    temp = "CurrZ: " + (String) currZ;
//    Serial.println(temp);
//    temp = "initZ: " + (String) initZ;
//    Serial.println(temp);
//    String currOrientation = "currX: "+ (String)currX + "currY: " + (String)currY + "currZ: " + (String)currZ;
//    Serial.println(currOrientation);
  }

  // set motor speeds
  stopLeftMotor();
  stopRightMotor();
}







//#include <Arduino.h>
//#include <Wire.h>
//
////Sensor Libraries
//#include "VL53L1X.h" //TOF
//#include "MPU6050_tockn.h" //IMU
//#include "SensorIMU.h"
//#include "ST_HW_HC_SR04.h"
//#include "Encoder.h"
//
//#define encoder0PinA  2
//#define encoder0PinB  3
//Encoder encoder(encoder0PinA, encoder0PinB);
//
//
//
////Function Prototypes
//double getUltrasonicReading(ST_HW_HC_SR04*, int);
//void printCurrIMUData(unsigned long);
//void alignToWall(ST_HW_HC_SR04*);
//void alignToWallWithTwo(ST_HW_HC_SR04*, ST_HW_HC_SR04*);
//
//void turn90Deg(bool);
//
////void setMotorSpeeds(int rightMotorDir, int rightMotorSpeed, int leftMotorDir, int leftMotorSpeed);
//void intializeArduinoMotorControllerPins();
//void intializeL289NMotorShield();
//
//void setLeftMotor(int leftMotorDir, int leftMotorSpeed);
//void setRightMotor(int rightMotorDir, int rightMotorSpeed);
//void stopLeftMotor();
//void stopRightMotor();
//
//void doEncoder();
////void doEncoderB();
//
//const int ROT_SPEED = 255;
//
//const int BWD = 0;
//const int FWD = 1;
//#define ROTATE_TOL 1
//const double ALIGN_TOL = 0.5;
//const int MS_ROTATE = 200;
//
//volatile unsigned int encoder0Pos = 0;
//
//
//VL53L1X TOF;
//Sensor_IMU IMU;
//
//ST_HW_HC_SR04* UltrasonicLeft1;
//ST_HW_HC_SR04* UltrasonicLeft2;
//
//
//unsigned long startMillis = 0, currMillis = 0, deltaMillis = 0, IMU_lastReadTime = 0, lastAlignTime = 0;  //some global variables available anywhere in the program
//float testZ;
//int hitTime;
//float offsetXAngle=0, offsetYAngle=0, offsetZAngle=0;
//
///*
// * Right and left motors for Arduino MotorShield
// * Unused pins with Arduino Motorcontroller
// * 0,1,2,4,7,10
// */
//int RightMotorDir = 12, RightMotorBrake = 9, LeftMotorDir = 13, LeftMotorBrake = 8, RightMotorSpeed = 3, LeftMotorSpeed = 11;
//// connect motor controller pins to Arduino digital pins
//
////Pin definitions for L289N MotorShield
///*
// * RightMotorEnable = enA
// * RightMotorDir1 = in1
// * RightMotorDir2 = in2
// *
// * LeftMotorEnable = enB
// * LeftMotorDir1 = in3
// * LeftMotorDir2 = in4
// */
//int RightMotorEnable = 10, RightMotorDir1 = 9, RightMotorDir2 = 8, LeftMotorEnable = 5, LeftMotorDir1 = 7, LeftMotorDir2 = 6;
//
//
//
//void setup() {
//	//MOTOR CONTROLLER PIN SETUP
//	//intializeArduinoMotorControllerPins();
//
//	intializeL289NMotorShield();
//	stopRightMotor();
//	stopLeftMotor();
//
//	Serial.begin(115200);
//    while(!Serial); // Wait for the Serial connection;
//	Wire.begin();
//
//	Wire.setClock(400000); // use 400 kHz I2C
////	TOF.begin();
//	//Serial.println("IMU Turning 90 Deg Test");
//
//	IMU.initialize();
//
//	//ULTRASONICS
//	 // TRIG = 1, ECHO= 0
//	//UltrasonicLeft1 = new ST_HW_HC_SR04(10, 7);
//	//UltrasonicLeft2 = new ST_HW_HC_SR04(4, 2);
//
//	//delay(1000);
//
//	//String offsets = "Offsets: " + (String) offsetXAngle + ", " + (String)offsetYAngle +", "+(String)offsetXAngle;
//
//	//Serial.println(offsets);
//
//	startMillis = millis();  //initial start time
//
//
//	setLeftMotor (FWD,250);
//	setRightMotor(FWD,250);
//
//	//String temp = "Aligning to wall";
//	//Serial.println(temp);
//	//alignToWallWithTwo(UltrasonicLeft1, UltrasonicLeft2);
//
//	//ENCODER SETUP
//   // attachInterrupt(digitalPinToInterrupt(0), doEncoder, CHANGE);
//
//
//}
//
//bool test = 0;
//
//void loop(void)
//{
////	delay(2000);
////	IMU.recalcOffsets();
////	delay(100);
////	turn90Deg(test);
////	test = !test;
//
////	while (TOF.newDataReady() == false)
////		delay(5);
////
////	int distance = TOF.getDistance(); //Get the result of the measurement from the sensor
////
////	Serial.print("Distance(mm): ");
////	Serial.print(distance);
//
//	//	alignToWallWithTwo(UltrasonicLeft1, UltrasonicLeft2);
////	currMillis = millis();
////	deltaMillis = currMillis - startMillis;
////
////	delay(4000);
////		String temp = "Aligning to wall";
////		Serial.println(temp);
////		alignToWall(Ultrasonic);
////		//IMUData tempData = IMU.getData();
////		//tempData.angle[2]-=offsetZAngle;
////		//IMU.printData(tempData);
////
////		//IMU.IMU->update();
////
////
////		//Serial.print("Time: ");
////		//Serial.println(currMillis - lastAlignTime);
////		lastAlignTime = currMillis;
//
//
//
//	//forward @ full speed
//
//
//}
//
//void turn90Deg(bool isLeft){
//	stopLeftMotor();
//	stopRightMotor();
//	delay(MS_ROTATE);
//
////	if(isLeft)
////		Serial.println("Turning Left");
////	else
////		Serial.println("Turning Right");
//
//	float currZ = 0, initZ = 0;
//
//	IMUData data;
//	for (int i = 0; i < 10; i++) {
//		data = IMU.getData();
//	//	Serial.print(IMU->getAngleZ());
//	//	Serial.print(", ");
//		initZ += data.angle[2];
//	}
//	initZ /= 10;
//
//	currZ = 0;
//	if (isLeft) {
//		setLeftMotor(BWD, ROT_SPEED);
//		setRightMotor(FWD, ROT_SPEED);
//	}
//	else {
//		setLeftMotor(FWD, ROT_SPEED);
//		setRightMotor(BWD, ROT_SPEED);
//	}
//	String temp;
//	while((abs(currZ-initZ) - 90) < (ROTATE_TOL*2)) {
//		delay(MS_ROTATE);
//		data = IMU.getData();
//		currZ = data.angle[2];
//	//maybe change to:
////currZ = data.gyro[2];
////		temp = "CurrZ: " + (String) currZ;
////		Serial.println(temp);
////		temp = "initZ: " + (String) initZ;
////		Serial.println(temp);
////		String currOrientation = "currX: "+ (String)currX + "currY: " + (String)currY + "currZ: " + (String)currZ;
////		Serial.println(currOrientation);
//	}
//
//	// set motor speeds
//	stopLeftMotor();
//	stopRightMotor();
//}
//
//
///*	Print current imu data in CSV format
// * 	HEADER:
// * 		AccX, AccY, AccZ, GyroX, GyroY, GyroZ, AccAngleX, AccAngleY, AccAngleZ, GyroAngleX, GyroAngleY, GyroAngleZ, FinalAngleX, FinalAngleY, FinalAngleZ
// * 	Only call if time between calls exceeds 1000 ms
// */
//void printCurrIMUData(unsigned long currMillis){
//	if((currMillis - IMU_lastReadTime) > 200){
////		Serial.print(testIMUPtr2->getAccX());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getAccY());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getAccZ());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getGyroX());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getGyroY());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getGyroZ());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getAccAngleX());
////		Serial.print(", ");
////		Serial.print(testIMUPtr2->getAccAngleY());
////		Serial.print(", ");
//		Serial.print(IMU.IMU->getGyroAngleX());
//		Serial.print(", ");
//		Serial.print(IMU.IMU->getGyroAngleY());
//		Serial.print(", ");
//		Serial.print(IMU.IMU->getGyroAngleZ());
//		Serial.print(", ");
//		Serial.print(IMU.IMU->getAngleX());
//		Serial.print(", ");
//		Serial.print(IMU.IMU->getAngleY());
//		Serial.print(", ");
//		Serial.println(IMU.IMU->getAngleZ());
//		IMU_lastReadTime = currMillis;
//	}
//}
///*
// * Test implementation of following a wall using solely the Ultrasonic sensor
// *
// */
//void alignToWall(ST_HW_HC_SR04* sensor){
//	bool turnLeft = false;
//	stopLeftMotor();
//	stopRightMotor();
//
//	//Distances are in cm, limit precision to 1cm
//	double initDist = 0, rightDist = 0, leftDist = 0, currDist = 0, leftError = 0, rightError = 0, prevDist = 0;
//
//	initDist = getUltrasonicReading(sensor, 50);
//
//	//Try turning left to minimize error
//	setLeftMotor(BWD, ROT_SPEED);
//	setRightMotor(FWD, ROT_SPEED);
//
//	delay(MS_ROTATE);
//
//	stopLeftMotor();
//	stopRightMotor();
//
//	delay(MS_ROTATE);
//
//	leftDist = getUltrasonicReading(sensor, 50);
//
//	delay(MS_ROTATE);
//
//	//Try turning right to minimize error
//	setLeftMotor(FWD, ROT_SPEED);
//	setRightMotor(BWD, ROT_SPEED);
//
//	delay(MS_ROTATE);
//
//	stopLeftMotor();
//	stopRightMotor();
//
//	delay(500);
//
//	setLeftMotor(FWD, ROT_SPEED);
//	setRightMotor(BWD, ROT_SPEED);
//
//	delay(MS_ROTATE);
//	stopLeftMotor();
//	stopRightMotor();
//	delay(MS_ROTATE);
//
//	rightDist = getUltrasonicReading(sensor, 50);
//
//	String temp = "leftDist: " + (String)leftDist + " initDist: " + (String)initDist + " rightDist: " + (String)rightDist;
//	Serial.println(temp);
//
//	if((leftDist < 2 || rightDist < 2 || initDist < 2 ) || abs(leftDist - rightDist) < ALIGN_TOL){
//		if(leftDist < 2 || rightDist < 2 || initDist < 2 )
//			temp = "unknown state";
//		else
//				temp = "Aligned already";
//		Serial.println(temp);
//
//		//Turn left to realign with initial position
//		delay(500);
//		setLeftMotor(BWD, ROT_SPEED);
//		setRightMotor(FWD, ROT_SPEED);
//		delay(MS_ROTATE);
//		stopLeftMotor();
//		stopRightMotor();
//
//	}
//
//	else {
//		if(leftDist > initDist && initDist > rightDist){
//			//turn right to realign with wall
//			temp = "Turn Right";
//			Serial.println(temp);
//			turnLeft = false;
//		}
//		else if (leftDist > rightDist && rightDist > initDist){
//			//turn left to realign with wall
//			temp = "Turn Left";
//			Serial.println(temp);
//			turnLeft = true;
//		}
//		else if (leftDist < rightDist && leftDist > initDist){
//			//turn left to realign with wall
//			temp = "Turn Left";
//			Serial.println(temp);
//			turnLeft = false;
//		}
//		else if (leftDist < rightDist && rightDist > initDist){
//			//turn left to realign with wall
//			temp = "Turn Left";
//			Serial.println(temp);
//			turnLeft = true;
//		}
//		currDist = rightDist;
//		delay(MS_ROTATE);
//
//		if(turnLeft){
//			setLeftMotor(BWD, ROT_SPEED);
//			setRightMotor(FWD, ROT_SPEED);
//		}
//		else{
//			setLeftMotor(FWD, ROT_SPEED);
//			setRightMotor(BWD, ROT_SPEED);
//		}
//		temp = "Going inside while loop";
//		Serial.println(temp);
//
//		do{
//			prevDist = currDist;
//			delay(150);
//			currDist = 0;
//
//			while(currDist < 2){
//				currDist = getUltrasonicReading(sensor, 50);
//			}
//			temp = "PrevDist: " + (String)prevDist + " currDist: " +(String) currDist;
//			Serial.println(temp);
//		}while(currDist - prevDist > ALIGN_TOL);
//
//		temp = "Outof loop";
//		Serial.println(temp);
//	}
//	stopLeftMotor();
//	stopRightMotor();
//}
//
//void alignToWallWithTwo(ST_HW_HC_SR04* sensor1, ST_HW_HC_SR04* sensor2){
//	stopLeftMotor();
//	stopRightMotor();
//	bool turnLeft = false;
//	double leftDist = getUltrasonicReading(sensor1, 10);
//	double rightDist = getUltrasonicReading(sensor2, 10);
//
//	String temp = "leftDist: " + (String) leftDist + "rightDist: " + (String)rightDist;
//	Serial.println(temp);
//	temp = "abs(leftDist - rightDist): " + (String)(abs(leftDist - rightDist));
//	Serial.println(temp);
//	temp = (String)((abs(leftDist - rightDist) - ALIGN_TOL) > 0);
//	Serial.println(temp);
//
//	if((abs(leftDist - rightDist) - ALIGN_TOL) > 0){
//		if(rightDist < leftDist){
//			//turn right
//			setLeftMotor(BWD, ROT_SPEED);
//			setRightMotor(FWD, ROT_SPEED);
//			temp = "Turn right";
//			Serial.println(temp);
//		}
//		else{
//			setLeftMotor(FWD, ROT_SPEED);
//			setRightMotor(BWD, ROT_SPEED);
//			temp = "Turn left";
//			Serial.println(temp);
//		}
//		do{
//			leftDist = getUltrasonicReading(sensor1, 10);
//			rightDist = getUltrasonicReading(sensor2, 10);
//			//delay(50);
//			String temp = "leftDist: " + (String) leftDist + "rightDist: " + (String)rightDist;
//			Serial.println(temp);
//		}while(abs(leftDist - rightDist) > ALIGN_TOL);
//	}
//	else{
//		String temp = "Already aligned";
//		Serial.println(temp);
//		setLeftMotor(FWD, ROT_SPEED);
//		setRightMotor(FWD, ROT_SPEED);
//
//	}
////	stopRightMotor();
////	stopLeftMotor();
//}
//
//double getUltrasonicReading(ST_HW_HC_SR04* sensor, int numRead){
//	double returnVal = 0;
//	int hitTime = 0;
//
//	for (int i = 0; i < numRead;  i++){
//		hitTime = sensor->getHitTime();
//		returnVal += (double)(hitTime / 29.1);
//	}
//	returnVal /= numRead;
//	return returnVal;
//}
//
//void setLeftMotor(int leftMotorDir, int leftMotorSpeed){
////  digitalWrite(LeftMotorDir, leftMotorDir);
////  digitalWrite(LeftMotorBrake, LOW);
////  analogWrite(LeftMotorSpeed, leftMotorSpeed);
//
//	// this function will run the LeftMotor
//	if(leftMotorDir == 0){
//		digitalWrite(LeftMotorDir1, LOW);
//		digitalWrite(LeftMotorDir2, HIGH);
//	}
//	else{
//		digitalWrite(LeftMotorDir1, HIGH);
//		digitalWrite(LeftMotorDir2, LOW);
//	}
//	// set speed out of possible range 0~255
//	int speed = leftMotorSpeed%255;
//	analogWrite(LeftMotorEnable, speed);
//}
//
//void setRightMotor(int rightMotorDir, int rightMotorSpeed){
////  digitalWrite(RightMotorDir, rightMotorDir);
////  digitalWrite(RightMotorBrake, LOW);
////  analogWrite(RightMotorSpeed, rightMotorSpeed);
//
//	// this function will run the LeftMotor
//	if(rightMotorDir == 0){
//		digitalWrite(RightMotorDir1, LOW);
//		digitalWrite(RightMotorDir2, HIGH);
//	}
//	else{
//		digitalWrite(RightMotorDir1, HIGH);
//		digitalWrite(RightMotorDir2, LOW);
//	}
//	// set speed out of possible range 0~255
//	int speed = rightMotorSpeed%255;
//	analogWrite(RightMotorEnable, speed);
//}
//
//void stopLeftMotor() {
//	setLeftMotor(FWD, 0);
//	//digitalWrite(LeftMotorBrake, HIGH);
//}
//void stopRightMotor(){
//	setRightMotor(FWD, 0);
//	//digitalWrite(RightMotorBrake, HIGH);
//}
//
//void doEncoderA() {
//  // look for a low-to-high on channel A
//  if (digitalRead(encoder0PinA) == HIGH) {
//
//    // check channel B to see which way encoder is turning
//    if (digitalRead(encoder0PinB) == LOW) {
//      encoder0Pos = encoder0Pos + 1;         // CW
//    }
//    else {
//      encoder0Pos = encoder0Pos - 1;         // CCW
//    }
//  }
//
//  else   // must be a high-to-low edge on channel A
//  {
//    // check channel B to see which way encoder is turning
//    if (digitalRead(encoder0PinB) == HIGH) {
//      encoder0Pos = encoder0Pos + 1;          // CW
//    }
//    else {
//      encoder0Pos = encoder0Pos - 1;          // CCW
//    }
//  }
//  Serial.println (encoder0Pos, DEC);
//  // use for debugging - remember to comment out
//}
//
//void doEncoderB() {
//  // look for a low-to-high on channel B
//  if (digitalRead(encoder0PinB) == HIGH) {
//
//    // check channel A to see which way encoder is turning
//    if (digitalRead(encoder0PinA) == HIGH) {
//      encoder0Pos = encoder0Pos + 1;         // CW
//    }
//    else {
//      encoder0Pos = encoder0Pos - 1;         // CCW
//    }
//  }
//
//  // Look for a high-to-low on channel B
//
//  else {
//    // check channel B to see which way encoder is turning
//    if (digitalRead(encoder0PinA) == LOW) {
//      encoder0Pos = encoder0Pos + 1;          // CW
//    }
//    else {
//      encoder0Pos = encoder0Pos - 1;          // CCW
//    }
//  }
//}
//
//void doEncoder(){
//	encoder.update();
//	Serial.println( encoder.getPosition() );
//}
//
//void intializeArduinoMotorControllerPins() {
//	//MOTOR CONTROLLER PIN SETUP
//	pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
//	pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin
//	//Setup Channel B
//	pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
//	pinMode(LeftMotorBrake, OUTPUT); //Initiates Brake Channel A pin
//}
//
//void intializeL289NMotorShield() {
//	pinMode(RightMotorEnable, OUTPUT);
//	pinMode(LeftMotorEnable, OUTPUT);
//	pinMode(RightMotorDir1, OUTPUT);
//	pinMode(RightMotorDir2, OUTPUT);
//	pinMode(LeftMotorDir1, OUTPUT);
//	pinMode(LeftMotorDir2, OUTPUT);
//}
