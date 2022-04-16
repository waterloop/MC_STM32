#include "main.h"
#include "threads.hpp"
#include "timer_utils.h"
#include "led_thread.hpp"

RTOSThread LEDThread::thread_;

void LEDThread::initialize() {
    LEDThread::thread_ = RTOSThread(
        "debug_led_thread",
        500,
        osPriorityIdle,
        LEDThread::runLEDThread
    );
}

void LEDThread::setLED(LEDStatus status) {
    LEDThread::R = status.R;
    LEDThread::G = status.G;
    LEDThread::B = status.B;
    LEDThread::blink = status.blink;
    if (!status.blink) {
        LEDThread::on = 1;
    }
}

void LEDThread::runLEDThread(void* args) {
    while (1) {
        set_LED_intensity(RED, LEDThread::R*LEDThread::on);
        set_LED_intensity(GREEN, LEDThread::G*LEDThread::on);
        set_LED_intensity(BLUE, LEDThread::B*LEDThread::on);

        LEDThread::on ^= LEDThread::blink;

        osDelay(LED_THREAD_PERIOD);
    }
}

//    GPIOB->MODER &= ~(0b11 << (3*2));
//    GPIOB->MODER |= (0b01 << (3*2));


