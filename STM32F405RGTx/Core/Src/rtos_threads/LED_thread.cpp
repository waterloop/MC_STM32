#include "main.h"
#include "threads.hpp"
#include "timer_utils.h"
#include "LED_thread.hpp"

RTOSThread LEDThread::thread_;
float LEDThread::R;
float LEDThread::G;
float LEDThread::B;
uint8_t LEDThread::blink;
uint8_t LEDThread::on;


void LEDThread::initialize() {
    LEDThread::thread_ = RTOSThread(
        "debug_led_thread",
        500,
        LED_THREAD_PRIORITY,
        LEDThread::runLEDThread
    );
    LEDThread::R = 0;
    LEDThread::G = 0;
    LEDThread::B = 0;
    LEDThread::blink = false;
    LEDThread::on = true;
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
    LEDThread::on = 1;
    while (1) {
        set_LED_intensity(RED, LEDThread::R*LEDThread::on);
        set_LED_intensity(GREEN, LEDThread::G*LEDThread::on);
        set_LED_intensity(BLUE, LEDThread::B*LEDThread::on);

        LEDThread::on ^= LEDThread::blink;

        osDelay(LED_THREAD_PERIODICITY);
    }
}

