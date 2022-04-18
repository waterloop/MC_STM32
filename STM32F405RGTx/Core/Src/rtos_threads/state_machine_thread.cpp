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
uint8_t StateMachineThread::enable_fault_check;

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
    StateMachineThread::state = Idle;
    StateMachineThread::enable_fault_check = true;

    while (1) {
        switch (StateMachineThread::state) {
            case Idle:
                led.R = 0.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                LEDThread::setLED(led);

                // StateMachineThread::state = IdleEvent();
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
        if (enable_fault_check) {
            State_t severe_check = StateMachineThread::SevereFaultChecking();
            State_t normal_check = StateMachineThread::NormalFaultChecking();
            if (severe_check != NoFault) {
                StateMachineThread::state = severe_check;
            } else if (normal_check != NoFault) {
                StateMachineThread::state = normal_check;
            }
        }
        osDelay(STATE_MACHINE_THREAD_PERIODICITY);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FAULT CHECKING
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
uint8_t StateMachineThread::fault_checking_routine(MCSeverityCode severity) {
    uint8_t phase_msk = 0;
    uint8_t has_fault = 0;

    float max_phase_volt, min_phase_volt, max_curr, max_cap_volt, min_cap_volt, max_fet_temp;
    switch (severity) {
        case WARNING : {
            max_phase_volt = MAX_VOLTAGE_NORMAL;
            min_phase_volt = MIN_VOLTAGE_NORMAL;
            max_curr = MAX_CURRENT_NORMAL;
            max_cap_volt = MAX_DC_VOLTAGE_NORMAL;
            min_cap_volt = MIN_DC_VOLTAGE_NORMAL;
            max_fet_temp = MAX_FET_TEMP_NORMAL;
            break;
        }
        case SEVERE : {
            max_phase_volt = MAX_VOLTAGE_SEVERE;
            min_phase_volt = MIN_VOLTAGE_SEVERE;
            max_curr = MAX_CURRENT_SEVERE;
            max_cap_volt = MAX_DC_VOLTAGE_SEVERE;
            min_cap_volt = MIN_DC_VOLTAGE_SEVERE;
            max_fet_temp = MAX_FET_TEMP_SEVERE;
            break;
        }
        default : {
            Error_Handler();
        }
    }

    if (check_phase_overvolt(max_phase_volt, &phase_msk)) {
        REPORT_FAULT(severity, PHASE_OVERVOLTAGE, phase_msk)
        has_fault |= 1;
    }

    if (check_phase_undervolt(min_phase_volt, &phase_msk)) {
        REPORT_FAULT(severity, PHASE_UNDERVOLTAGE, phase_msk);
        has_fault |= 1;
    }

    if (check_phase_overcurr(max_curr, &phase_msk)) {
        REPORT_FAULT(severity, PHASE_OVERCURRENT, phase_msk);
        has_fault |= 1;
    }

    if (check_cap_overvolt(max_cap_volt)) {
        REPORT_FAULT(severity, DC_CAP_OVERVOLTAGE, 0)
        has_fault |= 1;
    }

    if (check_cap_undervolt(min_cap_volt)) {
        REPORT_FAULT(severity, DC_CAP_UNDERVOLTAGE, 0)
        has_fault |= 1;
    }

    // if (checK_motor_stall()) {
    //     report_motor_stall();
    //     return NormalDangerFault;
    // }

    if (check_fet_overtemp(max_fet_temp, &phase_msk)) {
        REPORT_FAULT(severity, PHASE_OVERTEMPERATURE, phase_msk)
        has_fault |= 1;
    }

    return has_fault;
}

State_t StateMachineThread::NormalFaultChecking() {
    uint8_t has_fault = StateMachineThread::fault_checking_routine(WARNING);

    if (has_fault) { return NormalDangerFault; }
    else { return NoFault; }
} 
State_t StateMachineThread::SevereFaultChecking() {
    uint8_t has_fault = StateMachineThread::fault_checking_routine(SEVERE);

    if (has_fault) { return SevereDangerFault; }
    else { return NoFault; }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////



