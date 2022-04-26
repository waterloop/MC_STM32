#include <stdio.h>
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "timer_utils.h"
#include "drv8323.h"
#include "mc.hpp"
#include "threads.hpp"

Drv8323 drv8323;
MC g_mc_data;

int mc_entry() {
    printf("\r\n");
    printf("starting LED PWM...\r\n");
    start_rgb_pwm();


    // printf("initializing drivers...\r\n");
    // drv8323 = Drv8323_init();
    // if (Drv8323_setup(&drv8323) != DRV8323_OK) { Error_Handler(); };

    printf("initializing rtos kernel...\r\n");
    osKernelInitialize();

    printf("initializing rtos threads...\r\n");
    MeasurementsThread::initialize();
    StateMachineThread::initialize();
    LEDThread::initialize();
    SVPWMThread::initialize();
    CANThread::initialize();

    printf("starting rtos scheduler...\r\n");
    osKernelStart();

    // should never reach this point...
    Error_Handler();

    return 0;
}
