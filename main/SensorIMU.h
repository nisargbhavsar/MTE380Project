/*
 * SensorIMU.h
 *
 *  Created on: Nov 5, 2018
 *      Author: nisarg
 */
#ifndef SENSORIMU_H_
#define SENSORIMU_H_

#include "SensorData.h"
#include "Wire.h"
class MPU6050;

class Sensor_IMU{
public:
  Sensor_IMU();
  virtual ~Sensor_IMU();

  IMUData getData();
  void initialize();
  void printDataBuffer();
  void printData(IMUData);
  int onWall();
  void recalcOffsets();

  static const int maxNumData = 5;
  MPU6050* IMU;
  IMUData* dataBuffer;
  int counter, numData;
  float offsetXAngle,offsetYAngle,offsetZAngle;
};
#endif /* SENSORIMU_H_ */
