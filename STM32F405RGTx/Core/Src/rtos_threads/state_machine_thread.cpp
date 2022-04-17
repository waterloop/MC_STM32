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

void StateMachineThread::setState(State_t state_) {
    StateMachineThread::state = state_;
}

void StateMachineThread::initialize() {
    StateMachineThread::thread = RTOSThread(
        "state_machine_thread",
        1024*3,
        osPriorityNormal,
        runStateMachine
    );
}

void StateMachineThread::runStateMachine(void* arg) {
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
                led.blink = 0;
                LEDThread::setLED(led);

                state = InitializeFaultEvent();
                break;

            case Idle:
                led.R = 0.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                state = IdleEvent();
                break;

            case AutoPilot:
                led.R = 50.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                state = AutoPilotEvent();
                break;

            case ManualControl:
                led.R = 0.0;
                led.G = 0.0;
                led.B = 50.0;
                led.blink = 0;
                LEDThread::setLED(led);

                state = ManualControlEvent();
                break;

            case NormalDangerFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                state = NormalDangerFaultEvent();
                break;

            case SevereDangerFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 1;
                LEDThread::setLED(led);

                state = SevereDangerFaultEvent();
                break;

            default:
                Error_Handler();
                break;
        }
        send_CAN_heartbeat();
        osDelay(STATE_MACHINE_THREAD_PERIODICITY);
    }
}

State_t StateMachineThread::NormalFaultChecking(void) {
    // uint8_t overvolt_faults = 0;
    // uint8_t undervolt_faults = 0;
    // uint8_t temp_faults = 0;
    uint8_t phase_msk = 0U;

    // Phase A
    if (
        (g_mc_data.fet_temps[0] > MAX_FET_TEMP_NORMAL) ||
        (g_mc_data.fet_temps[1] > MAX_FET_TEMP_NORMAL)
    ) {
        report_phase_overtemp();
    }

    // check for temperature faults
    for (int i = 0; i < NUM_MOSFETS; i++) {
        float mosfetTemp = g_mc_data.fet_temps[i];
        if (mosfetTemp > MAX_MOSFET_TEMP_NORMAL) {
            ++temp_faults;
        }
        if (temp_faults > MIN_TEMP_FAULTS) {
            // TODO: add error code
            return NormalDangerFault;
        }
    }


    // check undervolts and overvolts for phase outputs
    if (g_mc_data.pVa < MIN_VOLTAGE_NORMAL) {
        undervolt_faults++;
    }
    if (g_mc_data.pVb < MIN_VOLTAGE_NORMAL) {
        undervolt_faults++;
    }
    if (g_mc_data.pVc < MIN_VOLTAGE_NORMAL) {
        undervolt_faults++;
    }

    if (g_mc_data.pVa > MAX_VOLTAGE_NORMAL) {
        overvolt_faults++;
    }
    if (g_mc_data.pVb > MAX_VOLTAGE_NORMAL) {
        overvolt_faults++;
    }
    if (g_mc_data.pVc > MAX_VOLTAGE_NORMAL) {
        overvolt_faults++;
    }


    if (overvolt_faults > MIN_OVERVOLT_FAULTS) {
        return NormalDangerFault;
    }
    if (undervolt_faults > MIN_UNDERVOLT_FAULTS) {
        return NormalDangerFault;
    }

    // Check current for phase outputs
    if (
        (g_mc_data.pIa > MAX_CURRENT_NORMAL) ||
        (g_mc_data.pIb > MAX_CURRENT_NORMAL) ||
        (g_mc_data.pIc > MAX_CURRENT_NORMAL)
    ) {
        //TODO: error code
        return NormalDangerFault;
    }

    // DC fault checking
    if (g_mc_data.dc_voltage > MAX_DCVOLTAGE_NORMAL) {
        // TODO: error code
        return NormalDangerFault;
    }
    else if (g_mc_data.dc_voltage < MIN_DCVOLTAGE_NORMAL) {
        // TODO: error code
        return NormalDangerFault;
    }

    if (g_mc_data.dc_cap_temp > MAX_DCCAP_TEMP_NORMAL) {
        // TODO: error code
        return NormalDangerFault;
    }
    return NoFault;
}


