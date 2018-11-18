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

void locateTarget();

// CONSTANTS
const int ROT_SPEED = 60;
const int BWD = 0;
const int FWD = 1;
const double ALIGN_TOL = 0.5;
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

//  setLeftMotor (FWD,200);
//  setRightMotor(FWD,200);

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
}

void loop() {
  setLeftMotor (FWD,200);
  setRightMotor(FWD,200);
  double range = getmeasurements(10);
  Serial.println(range);

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

// TOF
double getmeasurements(int measurements)
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
void locateTarget() {

}
