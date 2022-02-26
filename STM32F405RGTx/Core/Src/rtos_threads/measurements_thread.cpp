#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "measurements_thread.hpp"
#include "mc.hpp"
#include "cmsis_os.h"
#include "threads.hpp"

// TODO: add defines here for conversions..
#define CURRENT_SENSE_RESISTANCE                1E-3
#define ADC_TO_VOLTAGE(adc)                     ( adc * (3.3/(1 << 12)) )
#define VOLTAGE_TO_CURRENT(voltage)             (voltage / CURRENT_SENSE_RESISTANCE)
#define VOLTAGE_DIVIDER_CONVERSION(voltage)     ( voltage * 17.25 )

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

            // read and convert phase voltages and dc voltage
            case 0:
               g_mc_data.pVa =          VOLTAGE_DIVIDER_CONVERSION( ADC_TO_VOLTAGE( val ) );
            case 1:
               g_mc_data.pVb =          VOLTAGE_DIVIDER_CONVERSION( ADC_TO_VOLTAGE( val ) );
            case 2:
               g_mc_data.pVc =          VOLTAGE_DIVIDER_CONVERSION( ADC_TO_VOLTAGE( val ) );
            case 3:
               g_mc_data.dc_voltage =   VOLTAGE_DIVIDER_CONVERSION( ADC_TO_VOLTAGE( val ) );

            // read and convert phase currents
            case 4:
               g_mc_data.pIa =          VOLTAGE_TO_CURRENT( val );
            case 5:
               g_mc_data.pIb =          VOLTAGE_TO_CURRENT( val );
            case 6:
               g_mc_data.pIc =          VOLTAGE_TO_CURRENT( val );

            // read and convert adc temperatures
            // TODO: need to figure out how to convert these values
            // NOTE: the ordering might change depending physical arrangement later on
            case 7:
               g_mc_data.dc_cap_temp =  111;
            case 8:
               g_mc_data.fet_temps[0] = 111;
            case 9:
               g_mc_data.fet_temps[1] = 111;
            case 10:
               g_mc_data.fet_temps[2] = 111;

            case 111:
                printf("Val = %lx\r\n", val);

        }

        // update with functions once acceleration code is written
        // g_mc_data.curr_accel =
        // g_mc_data.curr_speed =
        // g_mc_data.curr_pos =
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

