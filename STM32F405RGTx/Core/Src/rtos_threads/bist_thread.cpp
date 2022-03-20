#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "bist_thread.hpp"
#include "util.hpp"

static const char* _banner = " _       __      __            __                      \r\n" \
                             "| |     / /___ _/ /____  _____/ /___  ____  ____       \r\n" \
                             "| | /| / / __ `/ __/ _ \\/ ___/ / __ \\/ __ \\/ __ \\  \r\n" \
                             "| |/ |/ / /_/ / /_/  __/ /  / / /_/ / /_/ / /_/ /      \r\n" \
                             "|__/|__/\\__,_/\\__/\\___/_/  /_/\\____/\\____/ .___/  \r\n" \
                             "                                        /_/            \r\n";

RTOSThread BistThread::thread_;

void BistThread::initialize() {
    BistThread::thread_ = RTOSThread(
            "bist_thread",
            1024,
            osPriorityLow,
            BistThread::runBist
    );
}

void BistThread::runBist(void* args) {
    printf("%s", _banner);
    printf("Motor Control BIST Console\r\n");
    printf("Type 'help' for a list of available commands...\r\n\r\n");

    uint8_t buff[20];
    uint32_t len = 20;
    while (1) {
        BistThread::_sinput("> ", buff, &len);

        // print measurements
        if (BistThread::_strcmp(buff, "p_measurements"))    { BistThread::_p_measurements(); }
        else if (BistThread::_strcmp(buff, "pm"))           { BistThread::_p_measurements(); }

        // help
        else if (BistThread::_strcmp(buff, "help"))         { BistThread::_help(); }

        // clear
        else if (BistThread::_strcmp(buff, "clear"))         { BistThread::_clear(); }

        else if (BistThread::_strcmp(buff, "")) { /* do nothing... */ }
        else { printf("invalid command...\r\n"); }

        len = 20;
    }
}

void BistThread::_print(uint8_t* str) {
    uint32_t len = 0;
    while (str[len] != '\0') { len += 1; }

    HAL_UART_Transmit(&huart1, str, len, 1);
}

void BistThread::_sinput(const char* prompt, uint8_t* buff, uint32_t* len) {
    BistThread::_print((uint8_t*)prompt);
    uint32_t curr_len = 0;

    while (curr_len < (*len - 1)) {
        uint8_t tmp;

        // try to receive 1 character with a timeout value of 1ms
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, &tmp, 1, 0);

        if (status == HAL_TIMEOUT) {
            // do nothing...
        }
        else if (status == HAL_OK) {
            HAL_UART_Transmit(&huart1, &tmp, 1, 0);
            if (tmp == '\n') {
                break;
            }
            else if (tmp == '\r') {
                BistThread::_print((uint8_t*)"\n");
                break;
            }
            else {
                buff[curr_len] = tmp;
                curr_len += 1;
            }
        }
        else if (status == HAL_ERROR) {
            Error_Handler();
        }

        // humans cannot type faster than 50ms per character...
        // if you can, then too bad!!!
        osDelay(BIST_PERIODICITY);
    }
    buff[curr_len++] = '\0';
    *len = curr_len;
}

uint8_t BistThread::_strcmp(uint8_t* a, const char* b) {
    return !strcmp((const char*)a, b);
}

void BistThread::_help() {
    BistThread::_print((uint8_t*)"p_measurements [pm]   --> print BMS measurements to the screen\r\n");
    BistThread::_print((uint8_t*)"clear                 --> clears the command interface\r\n");
}

void BistThread::_p_measurements() {
    printf("mc.pVa = %d V\r\n", (int)g_mc_data.pVa);
    printf("mc.pVb = %d V\r\n", (int)g_mc_data.pVb);
    printf("mc.pvC = %d V\r\n", (int)g_mc_data.pVc);
    
    printf("mc.dc_voltage = %d V\r\n", (int)g_mc_data.dc_voltage);
    
    printf("mc.pIa = %d A\r\n", (int)g_mc_data.pIa);
    printf("mc.pIb = %d A\r\n", (int)g_mc_data.pIb);
    printf("mc.pIc = %d A\r\n", (int)g_mc_data.pIc);
    
    printf("mc.fet_temps[0] = %d deg C\r\n", (int)g_mc_data.fet_temps[0]);
    printf("mc.fet_temps[1] = %d deg C\r\n", (int)g_mc_data.fet_temps[1]);
    printf("mc.fet_temps[2] = %d deg C\r\n", (int)g_mc_data.fet_temps[2]);

    printf("mc.dc_cap_temp = %d deg C\r\n", (int)g_mc_data.dc_cap_temp);
    
    printf("mc.curr_pos = %d m\r\n", (int)g_mc_data.curr_pos);
    printf("mc.curr_speed = %d m/s\r\n", (int)g_mc_data.curr_speed);
    printf("mc.curr_accel = %d m/s^2\r\n", (int)g_mc_data.curr_accel);
}

void BistThread::_clear() {
    printf("\033[2J");
}
