#include <Arduino.h>
#include <Wire.h>

//Sensor Libraries
#include "VL53L1X.h" //TOF
#include "MPU6050_tockn.h" //IMU
#include "SensorIMU.h"
#include "ST_HW_HC_SR04.h"
#include "vl53l1_api.h"

// FUNCTION PROTOTYPES
double getUltrasonicReading(ST_HW_HC_SR04*, int);
void printCurrIMUData(unsigned long);
//void alignToWall(ST_HW_HC_SR04*);
void alignToWallWithTwo(ST_HW_HC_SR04*, ST_HW_HC_SR04*);
void rotate90Deg(int);
void setLeftMotor(int leftMotorDir, int leftMotorSpeed);
void setRightMotor(int rightMotorDir, int rightMotorSpeed);
void stopLeftMotor();
void stopRightMotor();
void driveStraight(int, int);
void locateTarget();
bool detectTarget(double, int);
void printArr(double arr[], int size_arr);
void stopMotors();
double getMeasurements(int);
void driveStraightToDist(int dir, int motor_speed, int distance);
void correctOrientation();
void chaseDownTarget();
bool onHump();
bool facingDown();
bool facingUp();

// CONSTANTS
const int ROT_SPEED = 60;
const int STRAIGHT_SCANNING_SPEED = 100;
const int STRAIGHT_SPEED = 200;
const int MAX_MOTOR_SPEED = 250;
const int DOWN_HUMP_SPEED = 100;

const int BWD = 0;
const int FWD = 1;
const int RIGHT = 1;
const int LEFT = 0;
const double ALIGN_TOL = 0.5;
const int WALL_DIST = 2000; // in mm
const int TARGET_TOL = 200;
const int REQ_OBJ_DETECTIONS = 5;

const int FACING_UP = 1; 
const int FACING_STRAIGHT = 0;
const int FACING_DOWN = 2;

const int MS_ROTATE = 200;
int RightMotorDir = 12, RightMotorBrake = 9, RightMotorSpeed = 3;
int LeftMotorDir = 13, LeftMotorBrake = 8, LeftMotorSpeed = 11;
VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
int status;
int RightMotorEnable = 10, RightMotorDir1 = 9, RightMotorDir2 = 8, LeftMotorEnable = 5, LeftMotorDir1 = 7, LeftMotorDir2 = 6;

ST_HW_HC_SR04* UltrasonicFront;

void setup() {
  intializeL289NMotorShield();
  stopLeftMotor();
  stopRightMotor();
  
  Serial.begin(115200);
  while(!Serial); // Wait for the Serial connection;
  Wire.begin();

  Wire.setClock(400000); // use 400 kHz I2C

  //MOTOR CONTROLLER PIN SETUP
  pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
  pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin

  //Setup Channel B
  pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
  pinMode(LeftMotorBrake, OUTPUT);  //Initiates Brake Channel A pin

  uint8_t byteData;
  uint16_t wordData;

  Dev->I2cDevAddr = 0x52;

  VL53L1_software_reset(Dev);

  VL53L1_RdByte(Dev, 0x010F, &byteData);
  Serial.print(F("VL53L1X Model_ID: "));
  Serial.println(byteData, HEX);
  VL53L1_RdByte(Dev, 0x0110, &byteData);
  Serial.print(F("VL53L1X Module_Type: "));
  Serial.println(byteData, HEX);
  VL53L1_RdWord(Dev, 0x010F, &wordData);
  Serial.print(F("VL53L1X: "));
  Serial.println(wordData, HEX);

  Serial.println(F("Autonomous Ranging Test"));
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_MEDIUM);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 100); // reduced to 50 ms from 500 ms in ST example
  status = VL53L1_StartMeasurement(Dev);

  if(status)
  {
    Serial.println(F("VL53L1_StartMeasurement failed"));
//    while(1);
  }

  // SET MOTOR SPEEDS
    setLeftMotor (FWD, MAX_MOTOR_SPEED);
    setRightMotor(FWD, MAX_MOTOR_SPEED);
    Serial.print("moved motors");

  // get TOS measurementd
  //double sideTofReading = getMeasurements(3);
  //Serial.println((String)sideTofReading);
}

void loop() {

  // OVERALL MISSION CONTROL

//  // Get to wall
//  correctOrientation();
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 1710); //  distance from middle of metal wall to side wall
//  rotate90Deg(RIGHT);
//  correctOrientation();
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 2000); //  distance from metal wall to back wall
//  correctOrientation();
//
//  // Get on wall
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while ();
//  stopMotors();
//
//  // Get over wall
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 800);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while (!onHump());
//  driveStraight(FWD, DOWN_HUMP_SPEED);
//  while (!facingDown());
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while(facingDown());
//  stopMotors();
//
//  // get to initial target searching position
//  correctOrientation();
//  rotate90Deg(LEFT);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED); // get to side
//  delay(500);
//  stopMotors();
//  rotate90Deg(RIGHT);
//  correctOrientation();
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 2300); // get to corner
//
//  // locate target
//  locateTarget();
//  rotate90Deg(RIGHT);

//  // get to target
//  chaseDownTarget();

//  // get back to wall
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 2300);
//  rotate90Deg(RIGHT);
//  driveStraight(FWD, MAX_MOTOR_SPEED); 
//  rotate90Deg(LEFT);
//  correctOrientation();
//
//  // Get on wall
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while (!facingUp());
//  stopMotors();
//
//  // Get over wall
//  driveStraightToDist(FWD, MAX_MOTOR_SPEED, 800);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while (!onHump());
//  driveStraight(FWD, DOWN_HUMP_SPEED);
//  while (!facingDown());
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  while(facingDown());
//  stopMotors();
//
//  // get back onto initial base
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  stopMotors();
//  delay(500);
//  rotate90Deg(LEFT);
//  correctOrientation();
//  driveStraight(FWD, MAX_MOTOR_SPEED);
//  delay(2000);
//  stopMotors(); 
//
//  delay(10000);
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
  int speed = leftMotorSpeed%255;
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
  int speed = rightMotorSpeed%255;
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

  // REPLACE THIS
  delay(500);
  stopMotors();
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
}

void correctOrientation() {
  // INSERT CODE HERE
}

// Motor shield
void intializeArduinoMotorControllerPins() {
   //MOTOR CONTROLLER PIN SETUP
  pinMode(RightMotorDir, OUTPUT); //Initiates Motor Channel A pin
  pinMode(RightMotorBrake, OUTPUT); //Initiates Brake Channel A pin
  //Setup Channel B
  pinMode(LeftMotorDir, OUTPUT); //Initiates Motor Channel A pin
  pinMode(LeftMotorBrake, OUTPUT); //Initiates Brake Channel A pin
 }

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
  static VL53L1_RangingMeasurementData_t RangingData;
  double range = -1;
  double rangetotal = 0;

  for (int i = 0; i < measurements; i++)
  {

    status = VL53L1_WaitMeasurementDataReady(Dev);
    if (!status)
    {
      status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
      if (status == 0)
      {
        range = RangingData.RangeMilliMeter;
        status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
        rangetotal += range;
      }
      else
      {
        Serial.println("error");
        status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
        return -1;
      } 
    }
  }

  return (rangetotal /(measurements));
}

// IMU
bool onHump() {
  return true;
}

bool facingDown() {
  return true;
}

bool facingUp() {
  return true;
}

void rotate90Deg(int dir) {
  
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
  stopMotors();
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
    int distanceTarget = getUltrasonicReading(UltrasonicFront, 5);
    Serial.println(distanceTarget);
    if(distanceTarget < 10){
      reachTarget = 1;
      }
  }
    stopMotors();
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
