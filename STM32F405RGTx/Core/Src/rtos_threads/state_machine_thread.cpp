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

void StateMachineThread::setState(State_t state_) { StateMachineThread::state = state_; }

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
    StateMachineThread::state = Initialize;

    while (1) {
        switch (StateMachineThread::state) {
            case Initialize:
                StateMachineThread::state = Idle;
                // state = InitializeEvent();
                break;

            case InitializeFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                // state = InitializeFaultEvent();
                break;

            case Idle:
                led.R = 0.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                StateMachineThread::state = IdleEvent();
                break;

            case AutoPilot:
                led.R = 50.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                // state = AutoPilotEvent();
                break;

            case ManualControl:
                led.R = 0.0;
                led.G = 0.0;
                led.B = 50.0;
                led.blink = 0;
                LEDThread::setLED(led);

                // state = ManualControlEvent();
                break;

            case NormalDangerFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                // state = NormalDangerFaultEvent();
                break;

            case SevereDangerFault:
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 1;
                LEDThread::setLED(led);

                // state = SevereDangerFaultEvent();
                break;

            default:
                Error_Handler();
                break;
        }
        // send_CAN_heartbeat();
        osDelay(STATE_MACHINE_THREAD_PERIODICITY);
    }
}

uint8_t StateMachineThread::check_phase_overvolt(float threshold, uint8_t* phase_msk) {
    uint8_t has_overvolt = 0;
    if (g_mc_data.pVa > threshold) {
        has_overvolt |= 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if (g_mc_data.pVb > threshold) {
        has_overvolt |= 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if (g_mc_data.pVc > threshold) {
        has_overvolt |= 1;
        *phase_msk |= PHASE_C_FAULT;
    }
    return has_overvolt;
}
uint8_t StateMachineThread::check_phase_undervolt(float threshold, uint8_t* phase_msk) {
    uint8_t has_undervolt = 0;
    if (g_mc_data.pVa < threshold) {
        has_undervolt |= 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if (g_mc_data.pVb < threshold) {
        has_undervolt |= 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if (g_mc_data.pVc < threshold) {
        has_undervolt |= 1;
        *phase_msk |= PHASE_C_FAULT;
    }
    return has_undervolt;
}
uint8_t StateMachineThread::check_phase_overcurr(float threshold, uint8_t* phase_msk) {
    uint8_t has_overcurr = 0;
    if (g_mc_data.pIa > threshold) {
        has_overcurr |= 1;
        *phase_msk |= 1;
    }
    if (g_mc_data.pIb > threshold) {
        has_overcurr |= 1;
        *phase_msk |= 1;
    }
    if (g_mc_data.pIc > threshold) {
        has_overcurr |= 1;
        *phase_msk |= 1;
    }
    return has_overcurr;
}
uint8_t StateMachineThread::check_cap_overvolt(float threshold) {
    uint8_t has_overvolt = 0;
    // PUT LOGIC HERE, IDK THO aASLDKFJASDFJKA;SLDFJA;SKLD
    return has_overvolt;
}
uint8_t StateMachineThread::check_cap_undervolt(float threshold) {
    uint8_t has_undervolt = 0;
    // PUT LOGIC HERE, IDK THO ASDFSDFLKAJSDFLKAJSDLFKJALS;DFJ
    return has_undervolt;
}
uint8_t StateMachineThread::check_fet_overtemp(float threshold, uint8_t* phase_msk) {
    uint8_t has_overtemp = 0;
    if ( (g_mc_data.fet_temps[0] > threshold) || (g_mc_data.fet_temps[1] > threshold) ) {
        has_overtemp |= 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if ( (g_mc_data.fet_temps[2] > threshold) || (g_mc_data.fet_temps[3] > threshold) ) {
        has_overtemp |= 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if ( (g_mc_data.fet_temps[4] > threshold) || (g_mc_data.fet_temps[5] > threshold) ) {
        has_overtemp |= 1;
        *phase_msk |= PHASE_C_FAULT;
    }
    return has_overtemp;
}


State_t StateMachineThread::NormalFaultChecking(void) {
    // uint8_t overvolt_faults = 0;
    // uint8_t undervolt_faults = 0;
    // uint8_t temp_faults = 0;
    uint8_t phase_msk = 0;

    if (check_phase_overvolt(MAX_VOLTAGE_NORMAL, &phase_msk)) {
        report_phase_overvolt(phase_msk);
        return NormalDangerFault;
    }

    if (check_phase_undervolt(MIN_VOLTAGE_NORMAL, &phase_msk)) {
        report_phase_undervolt(phase_msk);
        return NormalDangerFault;
    }

    if (check_phase_overcurr(MAX_CURRENT_NORMAL, &phase_msk)) {
        report_phase_overcurrent(phase_msk);
        return NormalDangerFault;
    }

    if (check_cap_overvolt(MAX_DC_VOLTAGE_NORMAL)) {
        report_cap_overvolt();
        return NormalDangerFault;
    }

    if (check_cap_undervolt(MIN_DC_VOLTAGE_NORMAL)) {
        report_cap_undervolt();
        return NormalDangerFault;
    }

    // if (checK_motor_stall()) {
    //     report_motor_stall();
    //     return NormalDangerFault;
    // }

    if (check_fet_overtemp(MAX_FET_TEMP_NORMAL, &phase_msk)) {
        report_phase_overtemp(phase_msk);
        return NormalDangerFault;
    }
    
    


    // check for temperature faults
    // for (int i = 0; i < NUM_MOSFETS; i++) {
    //     float mosfetTemp = g_mc_data.fet_temps[i];
    //     if (mosfetTemp > MAX_MOSFET_TEMP_NORMAL) {
    //         ++temp_faults;
    //     }
    //     if (temp_faults > MIN_TEMP_FAULTS) {
    //         // TODO: add error code
    //         return NormalDangerFault;
    //     }
    // }


    // // check undervolts and overvolts for phase outputs
    // if (g_mc_data.pVa < MIN_VOLTAGE_NORMAL) {
    //     undervolt_faults++;
    // }
    // if (g_mc_data.pVb < MIN_VOLTAGE_NORMAL) {
    //     undervolt_faults++;
    // }
    // if (g_mc_data.pVc < MIN_VOLTAGE_NORMAL) {
    //     undervolt_faults++;
    // }

    // if (g_mc_data.pVa > MAX_VOLTAGE_NORMAL) {
    //     overvolt_faults++;
    // }
    // if (g_mc_data.pVb > MAX_VOLTAGE_NORMAL) {
    //     overvolt_faults++;
    // }
    // if (g_mc_data.pVc > MAX_VOLTAGE_NORMAL) {
    //     overvolt_faults++;
    // }


    // if (overvolt_faults > MIN_OVERVOLT_FAULTS) {
    //     return NormalDangerFault;
    // }
    // if (undervolt_faults > MIN_UNDERVOLT_FAULTS) {
    //     return NormalDangerFault;
    // }

    // // Check current for phase outputs
    // if (
    //     (g_mc_data.pIa > MAX_CURRENT_NORMAL) ||
    //     (g_mc_data.pIb > MAX_CURRENT_NORMAL) ||
    //     (g_mc_data.pIc > MAX_CURRENT_NORMAL)
    // ) {
    //     //TODO: error code
    //     return NormalDangerFault;
    // }

    // // DC fault checking
    // if (g_mc_data.dc_voltage > MAX_DCVOLTAGE_NORMAL) {
    //     // TODO: error code
    //     return NormalDangerFault;
    // }
    // else if (g_mc_data.dc_voltage < MIN_DCVOLTAGE_NORMAL) {
    //     // TODO: error code
    //     return NormalDangerFault;
    // }

    // if (g_mc_data.dc_cap_temp > MAX_DCCAP_TEMP_NORMAL) {
    //     // TODO: error code
    //     return NormalDangerFault;
    // }
    // return NoFault;
}


