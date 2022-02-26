#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "measurements_thread.hpp"
#include "mc.hpp"
#include "cmsis_os.h"
#include "threads.hpp"

// TODO: add defines here for conversions..

RTOSThread MeasurementsThread::thread;
uint16_t MeasurementsThread::ADC_buffer[];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    HAL_StatusTypeDef status = HAL_ADC_Stop_DMA(hadc);
    if(status != HAL_OK) {
        printf("Error: HAL_ADC_Stop_DMA failed with status code %d\r\n", status);
        Error_Handler();
    }
    osThreadFlagsSet(MeasurementsThread::getThreadId(), 0x00000001U);        // set flag to signal that ADC conversion has completed
}

void MeasurementsThread::initialize(){
    thread = RTOSThread(
        "measurements_thread",
        1024,
        osPriorityAboveNormal,
        runMeasurements
    );

}


void MeasurementsThread::processData() {
    for (uint8_t i{0}; i < ADC_NUM_CONVERSIONS; i++) {
        uint32_t sum = 0;
        for(uint16_t j{0}; j < ADC_DECIMATION_COEFF; j++) {
            sum += ADC_buffer[i + ADC_NUM_CONVERSIONS*j];
        }
        uint32_t val = (sum/ADC_DECIMATION_COEFF);

        switch (i) {
            // TODO need to figure out which variables need to be loaded for each 'i'
            // global_mc_data.pVa = ...;
            // global_mc_data.pVb = ...;
            // global_mc_data.pVc = ...;
            case 0:
                printf("Val = %lx\r\n", val);

        }
    }
}

void MeasurementsThread::startADCandDMA() {
    // lock kernel to prevent a context switch while starting the DMA
    osKernelLock();

    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(
        &hadc1, (uint32_t*)ADC_buffer, ADC_NUM_CONVERSIONS*ADC_DECIMATION_COEFF);
    
    if (status != HAL_OK) {
        printf("Error: HAL_ADC_Start_DMA failed with status code %d\r\n", status);
    }
    
    osKernelUnlock();
}

void MeasurementsThread::runMeasurements(void* args) {
    startADCandDMA();

    while (1) {
        // TODO: need to figure out what needs to be done here, is the function needed?
    }
}

void MeasurementsThread::stopMeasurements() {
    osThreadSuspend(thread.getThreadId());
}

void MeasurementsThread::resumeMeasurements() {
    osThreadResume(thread.getThreadId());
}

osThreadId_t MeasurementsThread::getThreadId() {
    return thread.getThreadId();
} 

