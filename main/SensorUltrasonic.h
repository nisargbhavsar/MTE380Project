/*
 * SensorUltrasonic.h
 *
 *  Created on: Nov 13, 2018
 *      Author: nisarg
 */

#ifndef SENSORULTRASONIC_H_
#define SENSORULTRASONIC_H_

#include "SensorData.h"
class ST_HW_HC_SR04;

class SensorUltrasonic {
public:
	SensorUltrasonic(int triggerPin, int echoPin);
	virtual ~SensorUltrasonic();

	UltrasonicData getData();
	void printDataBuffer();
	void printData(UltrasonicData);

	static const int toCm = 29;
	static const double toCmPrecise = 29.10;
	static const int maxNumData = 5;
	ST_HW_HC_SR04* Ultrasonic;
	UltrasonicData* dataBuffer;
	int counter, numData;
};


#endif /* SENSORULTRASONIC_H_ */
