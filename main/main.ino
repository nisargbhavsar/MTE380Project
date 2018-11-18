#include <Arduino.h>
#include <Wire.h>

//Sensor Libraries
#include "VL53L1X.h" //TOF
#include "MPU6050_tockn.h" //IMU
#include "SensorIMU.h"
#include "AFMotor.h"
#include "ST_HW_HC_SR04.h"
#include "vl53l1_api.h"

// FUNCTION PROTOTYPES
double getUltrasonicReading(ST_HW_HC_SR04*, int);
void printCurrIMUData(unsigned long);
//void alignToWall(ST_HW_HC_SR04*);
void alignToWallWithTwo(ST_HW_HC_SR04*, ST_HW_HC_SR04*);
void turn90Deg(bool);
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

// CONSTANTS
const int ROT_SPEED = 60;
const int STRAIGHT_SCANNING_SPEED = 100;
const int BWD = 0;
const int FWD = 1;
const double ALIGN_TOL = 0.5;
const int WALL_DIST = 2000; // in mm
const int TARGET_TOL = 50;
const int REQ_OBJ_DETECTIONS = 5;

const int MS_ROTATE = 200;
int RightMotorDir = 12, RightMotorBrake = 9, RightMotorSpeed = 3;
int LeftMotorDir = 13, LeftMotorBrake = 8, LeftMotorSpeed = 11;
VL53L1_Dev_t                   dev;
VL53L1_DEV                     Dev = &dev;
int status;

void setup() {
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

  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

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
    while(1);
  }

//  locateTarget();

  // SET MOTOR SPEEDS
//  setLeftMotor (FWD, 250);
//  setRightMotor(FWD, 250);
}

void loop() {

double sideTofReading = getMeasurements(3);
Serial.print((String)sideTofReading);
}

// MOTOR CONTROL
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

void driveStraight(int dir, int motor_speed) {
  // maintaining straightness with two back TOF to be incorporated
  setLeftMotor (dir,motor_speed);
  setRightMotor(dir,motor_speed);
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
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

// COURSE FUNCTIONS

// drives straight and back
void locateTarget() {

  // buffer for TOF readings
  int size_tof_buf = 10;
  double tofReadings[size_tof_buf];

  bool foundTarget = false;
  
  // go forward 
  driveStraight(FWD, STRAIGHT_SCANNING_SPEED);

  // until object detected
  double sideTofReading = 0;
  while(!foundTarget) {
    Serial.print("bork");
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

// HELPER FUNCTIONS

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
  for (int i = 0; i < size_arr; i++) {
    Serial.print(String(arr[i]) + " ");
  }
  Serial.println();
}

void addToBuffer(double data[], int size_arr, double new_data){
  for (int i = 0; i < size_arr - 1; i++) {
    data[i] = data[i+1];
  }
  data[size_arr - 1] = new_data;
}
