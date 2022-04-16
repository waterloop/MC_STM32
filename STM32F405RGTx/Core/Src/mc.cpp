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

// redirect stdin and stdout to UART1
void __io_putchar(uint8_t ch) {
    HAL_UART_Transmit(&huart4, &ch, 1, 0xffff);
}
uint8_t __io_getchar() {
    uint8_t ch;
    HAL_UART_Receive(&huart4, &ch, 1, 0xffff);
    HAL_UART_Transmit(&huart4, &ch, 1, 0xffff);
    return ch;
}

int mc_entry() {
    printf("\r\n");
    printf("initializing CAN...\r\n");
    if (CANBus_init(&hcan1, &htim7) != HAL_OK) { Error_Handler(); }
    if (CANBus_subscribe(STATE_CHANGE_REQ) != HAL_OK) { Error_Handler(); }
    //if (CANBus_subscribe_mask(BUS_TEST_REQ_BASE, BUS_TEST_REQ_MSK) != HAL_OK) { Error_Handler(); }
    //if (CANBus_subscribe_mask(BUS_TEST_RESP_BASE, BUS_TEST_MSK) != HAL_OK) { Error_Handler(); }

    printf("starting LED PWM...\r\n");
    start_rgb_pwm();


    // printf("initializing drivers...\r\n");
    // drv8323 = Drv8323_init();
    // if (Drv8323_setup(&drv8323) != DRV8323_OK) { Error_Handler(); };

    printf("initializing rtos kernel...\r\n");
    osKernelInitialize();

    printf("initializing rtos threads...\r\n");
    MeasurementsThread::initialize();
    // StateMachineThread::initialize();
    LEDThread::initialize();

    printf("starting rtos scheduler...\r\n");
    osKernelStart();

    // should never reach this point...
    Error_Handler();

    return 0;
}
