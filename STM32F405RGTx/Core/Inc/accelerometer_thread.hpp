#pragma once

#include "main.h"
#include "accelerometer.hpp"

class AccelerometerThread{
    public:
        static void initialize();
        static void update();
        static I2C_HandleTypeDef *I2CConncetion;
        static Accelerometer accelerometer;
        static osThreadId_t getThreadId();
        static void stopMeasurements();
        static void resumeMeasurements();
    private:
        static RTOSThread thread;
};

