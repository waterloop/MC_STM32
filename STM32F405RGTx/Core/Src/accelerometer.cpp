#include "accelerometer.hpp"

Accelerometer::Accelerometer(I2C_HandleTypeDef *I2CConnection_){
	I2CConnection = I2CConnection_;
}
float Acceleromter::getAcceleration(axis accelerationAxis){
	return ACC[accelerationAxis];
}
float Acceleromter::getSpeed(axis speedAxis){
	return SPEED[speedAxis];
}
float Acceleromter::getPos(axis positionAxis){
	return POS[positionAxis];
}
float Acceleromter::getTimeDiff(){
	return timeDiff;
}
uint8_t Acceleromter::getRawAccelerationLow(axis accelerationAxis){
	return RAW_ACC_L[accelerationAxis];
}
uint8_t Acceleromter::getRawAccelerationHigh(axis accelerationAxis){
	return RAW_ACC_H[accelerationAxis];
}
void Acceleromter::update(){
	readRawAccleration();
	convertRawAcceleration();
	getTimeValues();
	updatePos();
	updateSpeed();
	PREVIOUS_TIME = CURRENT_TIME;
}


void Acceleromter::convertRawAcceleration(){
	for(int i =0 ; i<3; i++)
	{

		int16_t RAW_ACC = RAW_ACC_H[i];
		RAW_ACC <<= 8;
		RAW_ACC |= RAW_ACC_L[i];
		ACC[i] =  ((float)RAW_ACC / (float)SELECTED_ACC_RANGE)*GRAVITATIONAL_CONSTANT;
	}
}
void Acceleromter::readRawAcceleration(){
	// Get acceleration in X-AXIS
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_X_H, I2C_MEMADD_SIZE_8BIT, RAW_ACC_H[x_axis], 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_X_L, I2C_MEMADD_SIZE_8BIT, RAW_ACC_L[x_axis], 1, 1000);


	// Get acceleration in Y-AXIS
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Y_H, I2C_MEMADD_SIZE_8BIT, RAW_ACC_H[y_axis], 1000);
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Y_L, I2C_MEMADD_SIZE_8BIT, RAW_ACC_L[y_axis], 1, 1000);


	// Get acceleration in Z-AXIS
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Z_H, I2C_MEMADD_SIZE_8BIT, RAW_ACC_H[z_axis], 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, IMU_ADDR, REG_Z_L, I2C_MEMADD_SIZE_8BIT, RAW_ACC_L[z_axis], 1, 1000);
}
void Acceleromter::updatePos(){
	for(int i =0; i<3; i++){
		POS[i] += SPEED[i]*TIME_DIFF+0.5*ACC[i]*TIME_DIFF*TIME_DIFF;
	}
}
void Acceleromter::updateSpeed(){
	for(int i =0; i<3; i++){
		SPEED[i] += ACC[i]*TIME_DIFF;
		}
}
void Acceleromter::getTimeValues(){
	CURRENT_TIME = HAL_GetTick();
	TIME_DIFF = ((float)CURRENT_TIME-(float)PREVIOUS_TIME)/1000;
}
