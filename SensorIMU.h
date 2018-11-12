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

	static const int maxNumData = 5;
	MPU6050* IMU;
	IMUData* dataBuffer;
	int counter = 0, numData = 0;
	float offsetXAngle,offsetYAngle,offsetZAngle;
};
#endif /* SENSORIMU_H_ */
