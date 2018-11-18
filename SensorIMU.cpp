/*
 * SensorIMU.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: nisarg
 */
#include "SensorIMU.h"
#include "MPU6050_tockn.h" //IMU

Sensor_IMU::Sensor_IMU(){
//	Serial.println("Inside Constructor");
	this->IMU = NULL;
	this->dataBuffer = NULL;
	this->numData = 0;
	this->counter = -1;
	this->offsetXAngle = this->offsetYAngle = this->offsetZAngle = 0;
//	Serial.println("Finished Constructor");
}

void Sensor_IMU::recalcOffsets() {
	Serial.println("Recalculating offsets");
	float x = 0, y = 0, z = 0;

	for (int i = 0; i < 50; i++) {
		this->IMU->update();
		x += this->IMU->getAngleX();
		y += this->IMU->getAngleY();
		z += this->IMU->getAngleZ();
	}
	this->offsetXAngle = x / 50;
	this->offsetYAngle = y / 50;
	this->offsetZAngle = z / 50;
//	String offsetTemp = "offsetx: " + (String) (this->offsetXAngle)
//			+ "offsety: " + (String) (this->offsetYAngle) + "offsetz: "
//			+ (String) (this->offsetZAngle);
//	Serial.println(offsetTemp);
}

void Sensor_IMU::initialize(){
	this->IMU = new MPU6050(Wire);
	this->dataBuffer = (IMUData*) malloc(sizeof(IMUData)* 5);
	this->numData = 0;
	this->counter =-1;

	(this->IMU)->begin();
	(this->IMU)->calcGyroOffsets(0);

	delay(100);

	recalcOffsets();
}

Sensor_IMU::~Sensor_IMU() {

	delete this->dataBuffer;
	delete this->IMU;
}

IMUData Sensor_IMU::getData(){
	(this->IMU)->update();
	IMUData returnData;

	this->counter ++;
	int tempint = this->counter;
	this->counter = (int)((tempint)%(this->maxNumData));

	if(this->numData < this->maxNumData)
		this->numData += 1;

	this->dataBuffer[this->counter].timeStamp = millis();
	this->dataBuffer[this->counter].accel[0] = (this->IMU)->getAccX();
	this->dataBuffer[this->counter].accel[1] = (this->IMU)->getAccY();
	this->dataBuffer[this->counter].accel[2] = (this->IMU)->getAccZ();

	this->dataBuffer[this->counter].gyro[0] = (this->IMU)->getGyroX();
	this->dataBuffer[this->counter].gyro[1] = (this->IMU)->getGyroY();
	this->dataBuffer[this->counter].gyro[2] = (this->IMU)->getGyroZ();

	this->dataBuffer[this->counter].angle[0] = (this->IMU)->getAngleX() - this->offsetXAngle;
	this->dataBuffer[this->counter].angle[1] = (this->IMU)->getAngleY() - this->offsetYAngle;
	this->dataBuffer[this->counter].angle[2] = (this->IMU)->getAngleZ() - this->offsetZAngle;

//
//	returnData.timeStamp = millis();
//	returnData.accel[0] = (this->IMU)->getAccX();
//	returnData.accel[1] = (this->IMU)->getAccY();
//	returnData.accel[2] = (this->IMU)->getAccZ();
//
//	returnData.gyro[0] = (this->IMU)->getGyroX();
//	returnData.gyro[1] = (this->IMU)->getGyroY();
//	returnData.gyro[2] = (this->IMU)->getGyroZ();
//
//	returnData.angle[0] = (this->IMU)->getAngleX() - this->offsetXAngle;
//	returnData.angle[1] = (this->IMU)->getAngleY() - this->offsetYAngle;
//	returnData.angle[2] = (this->IMU)->getAngleZ() - this->offsetZAngle;

//	this->dataBuffer[this->counter] = returnData;



	return this->dataBuffer[this->counter];
}

void Sensor_IMU::printDataBuffer(){
	for(int i = 0; i < this->numData; i++){
		String temp = "Data " + (String)i + " :";
		Serial.print(temp);
		printData(this->dataBuffer[i]);
	}
}

void Sensor_IMU::printData(IMUData data){
	Serial.print(data.accel[0]);
	Serial.print(", ");
	Serial.print(data.accel[1]);
	Serial.print(", ");
	Serial.print(data.accel[2]);
	Serial.print(", ");
	Serial.print(data.gyro[0]);
	Serial.print(", ");
	Serial.print(data.gyro[1]);
	Serial.print(", ");
	Serial.print(data.gyro[2]);
	Serial.print(", ");
	Serial.print(data.angle[0]);
	Serial.print(", ");
	Serial.print(data.angle[1]);
	Serial.print(", ");
	Serial.print(data.angle[2]);
	Serial.print(", ");
	Serial.println(data.timeStamp);
}
/*
 * Return 2 if on wall and desecending
 * Return 1 if on wall and ascending
 * Return 0 if not on wall
 */
int Sensor_IMU::onWall(){
	Serial.println("Inside onWall");

	IMUData data = this->getData();
	printData(data);

	Serial.println(this->offsetXAngle);
	Serial.println(this->offsetXAngle + 90);
	Serial.println(this->offsetXAngle + 180);


	if(data.angle[0] > this->offsetXAngle && data.angle[0] < this->offsetXAngle ){
		return 1;
	}
	else if(data.angle[0] > this->offsetXAngle + 90 && data.angle[0] < this->offsetXAngle + 180){
		return 2;
	}
	else
		return 0;
}


