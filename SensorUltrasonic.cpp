/*
 * SensorUltrasonic.cpp
 *
 *  Created on: Nov 13, 2018
 *      Author: nisarg
 */

#include "SensorUltrasonic.h"
#include "ST_HW_HC_SR04.h"


SensorUltrasonic::SensorUltrasonic(int triggerPin, int echoPin) {
	this->Ultrasonic = new ST_HW_HC_SR04(triggerPin, echoPin);
	this->dataBuffer = (UltrasonicData*) malloc(sizeof(UltrasonicData)*maxNumData);
	this->counter = -1;
	this->numData = 0;
}

SensorUltrasonic::~SensorUltrasonic() {
	delete this->Ultrasonic;
	delete this->dataBuffer;
}

UltrasonicData SensorUltrasonic::getData(){
	UltrasonicData temp;
	temp.distance = this->Ultrasonic->getHitTime() / this->toCm;
	temp.timeStamp = millis();
	return temp;
}



//
//UltrasonicData getData();
//	void initialize();
//	void printDataBuffer();
//	void printData(UltrasonicData);
