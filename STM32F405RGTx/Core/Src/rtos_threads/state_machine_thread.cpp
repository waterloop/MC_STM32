#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "state_machine_thread.hpp"
#include "threads.hpp"
#include "util.hpp"
#include "main.h"
#include "mc.hpp"
#include "can.h"
#include "timer_utils.h"

RTOSThread StateMachineThread::thread;
State_t StateMachineThread::state;


void StateMachineThread::runStateMachine(void *argument) {
    LEDStatus led = {
        .R = 0,
        .G = 0,
        .B = 0,
        .blink = 0
    };

    while (1) {
        switch (state) {
            case Initialize:
                state = InitializeEvent();
                break;

            case InitializeFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = false;
                LEDThread::setLED(led);

                state = InitializeFaultEvent();
                break;

            case Idle:
                led.R = 0.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = false;
                LEDThread::setLED(led);

                state = IdleEvent();
                break;

            case AutoPilot:
                led.R = 50.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = false;
                LEDThread::setLED(led);

                state = AutoPilotEvent();
                break;

            case ManualControl:
                led.R = 0.0;
                led.G = 0.0;
                led.B = 50.0;
                led.blink = false;
                LEDThread::setLED(led);

                state = ManualControlEvent();
                break;

            case NormalDangerFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = false;
                LEDThread::setLED(led);

                state = NormalDangerFaultEvent();
                break;

            case SevereDangerFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = true;
                LEDThread::setLED(led);

                state = SevereDangerFaultEvent();
                break;

            default:
                Error_Handler();
                break;
        }
        SendCANHeartbeat();
        osDelay(STATE_MACHINE_THREAD_PERIODICITY);
    }
}

void StateMachineThread::setState(State_t state_) {
    StateMachineThread::state = state_;
}


