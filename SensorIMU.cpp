/*
 * SensorIMU.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: nisarg
 */
#include "SensorIMU.h"
#include "MPU6050_tockn.h" //IMU

Sensor_IMU::Sensor_IMU(){
	Serial.begin(115200);
	Wire.setClock(400000); // use 400 kHz I2C
	Wire.begin();

//	while(!Wire.isEnabled()){
//		Wire.begin();
//		Serial.println("Not available");
//
//	}
	//Serial.println((int)Wire.available());
	//Wire.setClock(400000); // use 400 kHz I2C

	this->IMU = new MPU6050(Wire);
	this->dataBuffer = (IMUData*) malloc(sizeof(IMUData)* 5);
	this->numData = 0;
	Serial.println("Inside Constructor");
	//Serial.println(this->IMU->getGyroXoffset());

	(this->IMU)->begin();
	//Serial.println(this->IMU->getGyroXoffset());

	//(this->IMU)->calcGyroOffsets(1);
	Serial.println("Finished Constructor");

}

Sensor_IMU::~Sensor_IMU() {

	delete this->dataBuffer;
	delete this->IMU;
}

IMUData Sensor_IMU::getData(){
	(this->IMU)->update();
	IMUData returnData;
	returnData.timeStamp = millis();
	returnData.accel[0] = (this->IMU)->getAccX();
	returnData.accel[1] = (this->IMU)->getAccY();
	returnData.accel[2] = (this->IMU)->getAccZ();

	returnData.gyro[0] = (this->IMU)->getGyroX();
	returnData.gyro[1] = (this->IMU)->getGyroY();
	returnData.gyro[2] = (this->IMU)->getGyroZ();

	returnData.angle[0] = (this->IMU)->getAngleX();
	returnData.angle[1] = (this->IMU)->getAngleY();
	returnData.angle[2] = (this->IMU)->getAngleZ();

	this->dataBuffer[this->counter] = returnData;
	this->counter = (this->counter + 1) % this->maxNumData;
	this->numData += 1;

	return returnData;
}

void Sensor_IMU::printDataBuffer(){
	for(int i = 0; i < this->numData; i++){
		printData(this->dataBuffer[counter]);
		//Serial.println(" ");
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

