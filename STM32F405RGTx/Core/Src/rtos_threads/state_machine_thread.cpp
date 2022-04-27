#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "threads.hpp"
#include "util.hpp"
#include "main.h"
#include "mc.hpp"
#include "can.h"
#include "timer_utils.h"
#include "state_machine_thread.hpp"

RTOSThread StateMachineThread::thread;
State_t StateMachineThread::state;
uint8_t StateMachineThread::enable_fault_check;
float StateMachineThread::temperature_limit;
float StateMachineThread::current_limit;

void StateMachineThread::setState(State_t state_) { StateMachineThread::state = state_; }
State_t StateMachineThread::getState() { return StateMachineThread::state; }

void StateMachineThread::initialize() {
    StateMachineThread::thread = RTOSThread(
        "state_machine_thread",
        1024*5,
        STATE_MACHINE_THREAD_PRIORITY,
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
    StateMachineThread::temperature_limit = MAX_FET_TEMP_NORMAL;
    StateMachineThread::current_limit = MAX_CURRENT_NORMAL;

    while (1) {
        switch (StateMachineThread::state) {
            case Idle : {
                led.R = 0.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                StateMachineThread::state = IdleEvent();
                break;
            }
            case AutoPilot : {
                led.R = 50.0;
                led.G = 50.0;
                led.B = 0.0;
                led.blink = 0;
                StateMachineThread::state = AutoPilotEvent();
                break;
            }
            case ManualControl : {
                led.R = 0.0;
                led.G = 0.0;
                led.B = 50.0;
                led.blink = 0;
                StateMachineThread::state = ManualControlEvent();
                break;
            }
            case NormalDangerFault : {
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 0;
                StateMachineThread::state = NormalDangerFaultEvent();
                break;
            }
            case SevereDangerFault : {
                led.R = 50.0;
                led.G = 0.0;
                led.B = 0.0;
                led.blink = 1;
                StateMachineThread::state = SevereDangerFaultEvent();
                break;
            }
            default : {
                Error_Handler();
                break;    
            }
        }
        LEDThread::setLED(led);

        if (enable_fault_check) {
            State_t severe_check = StateMachineThread::SevereFaultChecking();
            State_t normal_check = StateMachineThread::NormalFaultChecking();
            if (severe_check != NoFault) {
                StateMachineThread::state = severe_check;
            }
            else if (normal_check != NoFault) {
                StateMachineThread::state = normal_check;
            }
        }

        osDelay(STATE_MACHINE_THREAD_PERIODICITY);
    }
}

void StateMachineThread::ack_state_change(StateID requested_state) {
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK_ID, requested_state);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK, 0x00);
    send_frame(&tx_frame);
}
void StateMachineThread::nack_state_change(StateID requested_state) {
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK_ID, requested_state);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK, 0xFF);
    send_frame(&tx_frame);
}

State_t StateMachineThread::IdleEvent() {
    // TODO: turn off SVPWM, VHz, and PID once they're implemented
    MeasurementsThread::resumeMeasurements();
    stop_motor_pwm();

    StateID requested_state;
    osStatus_t status = osMessageQueueGet(g_state_change_req_queue, &requested_state, NULL, 0);
    if (status == osErrorTimeout) {
        return Idle;
    }
    else if (status == osOK) {
        if (requested_state == AUTO_PILOT) {
            return AutoPilot;
            StateMachineThread::ack_state_change(requested_state);
        }
        else if (requested_state == MANUAL_OPERATION_WAITING) {
            return ManualControl;
            StateMachineThread::ack_state_change(requested_state);
        }
        else {
            StateMachineThread::nack_state_change(requested_state);
            return Idle;
        }
    }
    else {
        Error_Handler();
    }
    return Idle;
}

State_t StateMachineThread::AutoPilotEvent() {
    // TODO: turn on SVPWM, VHz, and PID once they're implemented
    start_motor_pwm();
    g_mc_data.target_speed = AUTOPILOT_SPEED;

    StateID requested_state;
    osStatus status = osMessageQueueGet(g_state_change_req_queue, &requested_state, NULL, 0);
    if (status == osErrorTimeout) {
        return AutoPilot;
    }
    else if (status == osOK) {
        float distance_to_end = g_mc_data.track_length - g_mc_data.curr_pos;
        if ( (requested_state == EMERGENCY_BRAKE) || (requested_state == SYSTEM_FAILURE) ) {
            return SevereDangerFault;
        }
        else if ( (requested_state == BRAKING) || (distance_to_end <= DECELERATION_DISTANCE) ) {
            return Idle;
        }
    }
    else {
        Error_Handler();
    }
    return AutoPilot;
}
State_t StateMachineThread::ManualControlEvent() {
    // TODO: turn on SVPWM, VHz, and PID once they're implemented
    start_motor_pwm();

    StateID requested_state;
    osStatus_t status = osMessageQueueGet(g_state_change_req_queue, &requested_state, NULL, 0);
    if (status == osErrorTimeout) {
        return ManualControl;
    }
    else if (status == osOK) {
        if ( (requested_state == EMERGENCY_BRAKE) || (requested_state == SYSTEM_FAILURE) ) {
            return SevereDangerFault;
        }
        else if (requested_state == BRAKING) {
            return Idle;
        }
    }
    else {
        Error_Handler();
    }
    return ManualControl;
}
State_t StateMachineThread::NormalDangerFaultEvent() {
    // TODO: turn off SVPWM, VHz, and PID
    stop_motor_pwm();

    StateID requested_state;
    osStatus_t status = osMessageQueueGet(g_state_change_req_queue, &requested_state, NULL, 0);
    if (status == osErrorTimeout) {
        return NormalDangerFault;
    }
    else if (status == osOK) {
        if (requested_state == RESTING) {
            return Idle;
        }
    }
    else {
        Error_Handler();
    }
    return NormalDangerFault;
}
State_t StateMachineThread::SevereDangerFaultEvent() {
    // TODO: turn off SVPWM, VHz, and PID
    stop_motor_pwm();

    while (1) { asm("NOP"); }

    return SevereDangerFault;   // to avoid compiler warning
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FAULT CHECKING
uint8_t StateMachineThread::check_phase_overvolt(float threshold, uint8_t* phase_msk) {
    uint8_t has_overvolt = 0;
    if (g_mc_data.pVa > threshold) {
        has_overvolt = 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if (g_mc_data.pVb > threshold) {
        has_overvolt = 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if (g_mc_data.pVc > threshold) {
        has_overvolt = 1;
        *phase_msk |= PHASE_C_FAULT;
    }
    return has_overvolt;
}
uint8_t StateMachineThread::check_phase_undervolt(float threshold, uint8_t* phase_msk) {
    uint8_t has_undervolt = 0;
    if (g_mc_data.pVa < threshold) {
        has_undervolt = 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if (g_mc_data.pVb < threshold) {
        has_undervolt = 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if (g_mc_data.pVc < threshold) {
        has_undervolt = 1;
        *phase_msk |= PHASE_C_FAULT;
    }
    return has_undervolt;
}
uint8_t StateMachineThread::check_phase_overcurr(float threshold, uint8_t* phase_msk) {
    uint8_t has_overcurr = 0;
    if (g_mc_data.pIa > threshold) {
        has_overcurr = 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if (g_mc_data.pIb > threshold) {
        has_overcurr = 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if (g_mc_data.pIc > threshold) {
        has_overcurr = 1;
        *phase_msk |= PHASE_C_FAULT;
    }
    return has_overcurr;
}

uint8_t StateMachineThread::check_cap_overvolt(float threshold) { return (g_mc_data.dc_voltage > threshold); }
uint8_t StateMachineThread::check_cap_undervolt(float threshold) { return (g_mc_data.dc_voltage < threshold); }

uint8_t StateMachineThread::check_fet_overtemp(float threshold, uint8_t* phase_msk) {
    uint8_t has_overtemp = 0;
    if ( (g_mc_data.fet_temps[0] > threshold) || (g_mc_data.fet_temps[1] > threshold) ) {
        has_overtemp = 1;
        *phase_msk |= PHASE_A_FAULT;
    }
    if ( (g_mc_data.fet_temps[2] > threshold) || (g_mc_data.fet_temps[3] > threshold) ) {
        has_overtemp = 1;
        *phase_msk |= PHASE_B_FAULT;
    }
    if ( (g_mc_data.fet_temps[4] > threshold) || (g_mc_data.fet_temps[5] > threshold) ) {
        has_overtemp = 1;
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
            max_curr = StateMachineThread::current_limit;
            max_cap_volt = MAX_DC_VOLTAGE_NORMAL;
            min_cap_volt = MIN_DC_VOLTAGE_NORMAL;
            max_fet_temp = StateMachineThread::temperature_limit;
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
            // assigning these values to avoid a compiler warning for uninitialized vars
            max_phase_volt = 0;
            min_phase_volt = 0;
            max_curr = 0;
            max_cap_volt = 0;
            min_cap_volt = 0;
            max_fet_temp = 0;
            Error_Handler();
            break;
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



