#pragma once
#include "util.hpp"

// TODO: these values might need to be adjusted as needed
#define ADC_NUM_CONVERSIONS         6U
#define ADC_DECIMATION_COEFF        256U

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

class MeasurementsThread {
    public:
        static void initialize();

        static void stopMeasurements();
        static void resumeMeasurements();
        static osThreadId_t getThreadId();

    private:
        static RTOSThread thread;
        static void runMeasurements(void* args);

        static uint16_t ADC_buffer[ADC_NUM_CONVERSIONS*ADC_DECIMATION_COEFF];

        static void processData();

        static void startADCandDMA();

};