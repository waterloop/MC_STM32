#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "threads.hpp"
#include "mc.hpp"
#include "timer_utils.h"
#include "state_machine_thread.hpp"
#include "bist_thread.hpp"

static const char* _banner = " _       __      __            __                      \r\n" \
                             "| |     / /___ _/ /____  _____/ /___  ____  ____       \r\n" \
                             "| | /| / / __ `/ __/ _ \\/ ___/ / __ \\/ __ \\/ __ \\  \r\n" \
                             "| |/ |/ / /_/ / /_/  __/ /  / / /_/ / /_/ / /_/ /      \r\n" \
                             "|__/|__/\\__,_/\\__/\\___/_/  /_/\\____/\\____/ .___/  \r\n" \
                             "                                        /_/            \r\n";

RTOSThread BistThread::thread_;
void (*BistThread::callback)(void);

// Button ISR
// I'm not putting too much effort into making this ISR short, since we are inherantly
// not in a time/safety sensitive environment if we are using the BIST...
static uint32_t prev_time = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BIST_Pin) {
        uint32_t curr_time = HAL_GetTick();
        if ( (HAL_GetTick() - prev_time) > BUTTON_DEBOUNCE_TIME ) {
            BistThread::toggleBist();
            prev_time = curr_time;
        }
    }
}


void BistThread::initialize() {
    BistThread::thread_ = RTOSThread(
        "bist_thread",
        1024,
        BIST_THREAD_PRIORITY,
        BistThread::runBist
    );
    BistThread::callback = &BistThread::disabled_callback;
}

void BistThread::toggleBist() {
    if (BistThread::callback == &BistThread::enabled_callback) {
        BistThread::callback = &BistThread::disabled_callback;
    }
    else if (BistThread::callback == &BistThread::disabled_callback) {
        printf("%s", _banner);
        printf("Motor Controller BIST Console\r\n");
        printf("Type 'help' for a list of available commands...\r\n\r\n");
        BistThread::callback = &BistThread::enabled_callback;
    }
}

static uint8_t buff[20];
static uint32_t len = 20;
void BistThread::enabled_callback() {
    BistThread::_sinput("> ", buff, &len);

    if (BistThread::_strcmp(buff, "p_measurements"))    { BistThread::_p_measurements(); }
    else if (BistThread::_strcmp(buff, "pm"))           { BistThread::_p_measurements(); }
    else if (BistThread::_strcmp(buff, "rgb"))          { BistThread::_rgb(); }
    else if (BistThread::_strcmp(buff, "help"))         { BistThread::_help(); }
    else if (BistThread::_strcmp(buff, "toggle_fc"))    { BistThread::_toggle_fc(); }
    else if (BistThread::_strcmp(buff, "clear"))        { BistThread::_clear(); }
    else if (BistThread::_strcmp(buff, ""))             { /* do nothing... */ }

    else { printf("invalid command...\r\n"); }

    len = 20;
}
void BistThread::disabled_callback() { osThreadYield(); }

void BistThread::runBist(void* args) {
    // this implementation of disabling the thread isn't ideal but it should also be fine...
    while (1) {
        BistThread::callback();
    }
}

void BistThread::_print(uint8_t* str) {
    uint32_t len = 0;
    while (str[len] != '\0') { len += 1; }

    HAL_UART_Transmit(&huart4, str, len, 1);
}


void BistThread::_sinput(const char* prompt, uint8_t* buff, uint32_t* len) {
    BistThread::_print((uint8_t*)prompt);
    uint32_t curr_len = 0;

    while (curr_len < (*len - 1)) {
        uint8_t tmp;

        // try to receive 1 character with a timeout value of 1ms
        HAL_StatusTypeDef status = HAL_UART_Receive(&huart4, &tmp, 1, 0);

        if (status == HAL_TIMEOUT) {
            // do nothing...
        }
        else if (status == HAL_OK) {
            HAL_UART_Transmit(&huart4, &tmp, 1, 0);
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
        osDelay(BIST_THREAD_PERIODICITY);
    }
    buff[curr_len++] = '\0';
    *len = curr_len;
}

uint8_t BistThread::_strcmp(uint8_t* a, const char* b) {
    return !strcmp((const char*)a, b);
}

void BistThread::_help() {
    BistThread::_print((uint8_t*)"p_measurements [pm]   --> print Motor Controller measurements to the screen\r\n");
    BistThread::_print((uint8_t*)"rgb                   --> change the color of the RGB LED\r\n");
    BistThread::_print((uint8_t*)"toggle_fc             --> toggle fault checking in state machine\r\n");
    BistThread::_print((uint8_t*)"clear                 --> clears the command interface\r\n");
}

void BistThread::_p_measurements() {

    printf("mc.pVa = %dV\r\n", (int)(g_mc_data.pVa));
    printf("mc.pVb = %dV\r\n", (int)(g_mc_data.pVb));
    printf("mc.pVc = %dV\r\n", (int)(g_mc_data.pVc));

    printf("mc.pVDC = %dV\r\n", (int)(g_mc_data.dc_voltage));

    printf("mc.pIa = %d A\r\n", (int)g_mc_data.pIa);
    printf("mc.pIb = %d A\r\n", (int)g_mc_data.pIb);
    printf("mc.pIc = %d A\r\n", (int)g_mc_data.pIc);

    printf("mc.fet_temp1  = %d deg C\r\n", (int)(g_mc_data.fet_temps[0]));
    printf("mc.fet_temp2  = %d deg C\r\n", (int)(g_mc_data.fet_temps[0]));
    printf("mc.fet_temp3  = %d deg C\r\n", (int)(g_mc_data.fet_temps[0]));
    
    // TODO: need to verify the formatting/units of these
    printf("mc.curr_pos  = %d m\r\n", (int)(g_mc_data.curr_pos));
    printf("mc.curr_speed  = %d m/s\r\n", (int)(g_mc_data.curr_speed));
    printf("mc.curr_accel  = %d m/s^2\r\n", (int)(g_mc_data.curr_accel));

    printf("mc.target_speed  = %d m/s\r\n", (int)(g_mc_data.target_speed));
    printf("mc.track_length  = %d m\r\n", (int)(g_mc_data.track_length));

    printf("mc.ring_encoder_reported_speed  = %d m/s\r\n", (int)(g_mc_data.ring_encoder_reported_speed));


}
void BistThread::_rgb() {
    set_LED_intensity(RED, 0);
    set_LED_intensity(GREEN, 0);
    set_LED_intensity(BLUE, 0);

    uint8_t buff[5];
    uint32_t len = 5;

    BistThread::_sinput("R: ", buff, &len);
    float red = atof((const char*)buff);
    set_LED_intensity(RED, red);
    len = 5;

    BistThread::_sinput("G: ", buff, &len);
    float green = atof((const char*)buff);
    set_LED_intensity(GREEN, green);
    len = 5;

    BistThread::_sinput("B: ", buff, &len);
    float blue = atof((const char*)buff);
    set_LED_intensity(BLUE, blue);

}

void BistThread::_toggle_fc() {
    uint8_t buff[10];
    uint32_t len = 10;

    while (true) {
        BistThread::_sinput("on or off?: ", buff, &len);
        if (BistThread::_strcmp(buff, "on")) { 
            StateMachineThread::setFaultChecking(1);
            break;
        } else if (BistThread::_strcmp(buff, "off")) {
            StateMachineThread::setFaultChecking(0);
            break;
        } 
    }
}

void BistThread::_clear() {
    printf("\033[2J\r\n");
}
