#include <stdio.h>
#include "main.h"
#include "cmsis_os.h"
#include "drv8323.h"
#include "mc.hpp"
#include "threads.hpp"
#include "can.h"
#include "state_machine.hpp"

MC global_mc_data;
Drv8323 drv8323;

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
    CANBus_subscribe(MANUAL_CONTROL_1);
    CANBus_subscribe(STATE_CHANGE_REQ);

    printf("initializing drivers...\r\n");
    drv8323 = Drv8323_init();
    if (Drv8323_setup(&drv8323) != DRV8323_OK) { Error_Handler(); };

    printf("initializing rtos kernel...\r\n");
    osKernelInitialize();

    printf("initializing rtos threads...\r\n");
    
    MeasurementsThread::initialize();
    StateMachineThread::initialize();

    printf("starting rtos scheduler...\r\n");
    osKernelStart();

    // should never reach this point...
    Error_Handler();

    return 0;
}

