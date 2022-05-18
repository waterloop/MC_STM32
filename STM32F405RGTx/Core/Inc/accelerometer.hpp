#pragma once

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include "main.h"

#define IMU_ADDR 0x19<<1	//| Use 8-bit address
#define REG_CTRL_1   0x20 	//| Control Register-1
#define REG_CTRL_4   0x23 	//| Control Register-4
#define REG_X_L 	  0x28 	//| X-Axis LSB
#define REG_X_H 	  0x29		//| X-Axis MSB
#define REG_Y_L 	  0x2A		//| Y-Axis LSB
#define REG_Y_H 	  0x2B		//| Y-Axis MSB
#define REG_Z_L 	  0x2C		//| Z-Axis LSB
#define REG_Z_H 	  0x2D		//| Z-Axis MSB

//Constants used to convert raw acceleration based on range
#define ACC_RANGE_2G   16384
#define ACC_RANGE_4G   8192
#define ACC_RANGE_8G   4096
#define ACC_RANGE_16G   2048

//Control register 4 values to change acceleration range
#define CTRL_REG_4_ACCL_RANGE_16G               0x30 // Full scale   +/-16g
#define CTRL_REG_4_ACCL_RANGE_8G                0x20 // Full scale   +/-8g
#define CTRL_REG_4_ACCL_RANGE_4G                0x10 // Full scale   +/-4g
#define CTRL_REG_4_ACCL_RANGE_2G                0x00 //Full scale = +/-2g

//Selected acceleration range
#define SELECTED_ACC_RANGE ACC_RANGE_2G
#define SELECTED_CTRL_REG_4_ACCL_RANGE CTRL_REG_4_ACCL_RANGE_2G

#define GRAVITATIONAL_CONSTANT 9.8

enum axis{
			x_axis,
			y_axis,
			z_axis};

class Accelerometer{
    public:
		float getAcceleration(axis accelerationAxis);
		float getSpeed(axis speedAxis);
		float getPos(axis accelerationAxis);
		float getTimeDiff();
		uint8_t getRawAccelerationLow(axis accelerationAxis);
		uint8_t getRawAccelerationHigh(axis accelerationAxis);
		void update();
		Accelerometer(I2C_HandleTypeDef *I2CConnection_);

    private:
        void convertRawAcceleration();
        void readRawAcceleration();
        void updatePos();
        void updateSpeed();
        void getTimeValues();

        float ACC[3] = {0,0,0};
        float POS[3] = {0,0,0};
        float SPEED[3] = {0,0,0};

        uint8_t RAW_ACC_L[3] = {0,0,0};
        uint8_t RAW_ACC_H[3] = {0,0,0};

        int PREVIOUS_TIME = 0;
        int CURRENT_TIME = 0;
        float TIME_DIFF = 0;

        I2C_HandleTypeDef *I2CConnection;

};


