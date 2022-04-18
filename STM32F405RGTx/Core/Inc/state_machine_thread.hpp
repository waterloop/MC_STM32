/* state_machine.hpp
 *
 *    Created on: Jul. 11, 2021
 *            Author: Kanishk Kashyap
*/
#pragma once

#include <stdint.h>
#include "can.h"
#include "cmsis_os.h"
#include "can.h"
#include "util.hpp"

// Constants - These numbers are arbritrary should be changed after testing
// Mosfet Array Length
#define NUM_MOSFETS                 6
#define NUM_PHASE_OUTPUTS           3

// Severe Fault
#define MAX_VOLTAGE_SEVERE          4.0F
#define MIN_VOLTAGE_SEVERE          1.0F
#define MAX_CURRENT_SEVERE          50.0F
#define MAX_DC_VOLTAGE_SEVERE       4.0F
#define MIN_DC_VOLTAGE_SEVERE       1.8F
#define MAX_FET_TEMP_SEVERE         70.0F

// Normal Fault
#define MAX_VOLTAGE_NORMAL          3.8F
#define MIN_VOLTAGE_NORMAL          1.8F
#define MAX_CURRENT_NORMAL          40.0F
#define MAX_DC_VOLTAGE_NORMAL       4.0F
#define MIN_DC_VOLTAGE_NORMAL       1.8F
#define MAX_FET_TEMP_NORMAL         60.0F

// DC_CAPTEMP
#define MAX_DCCAP_TEMP_NORMAL       5
#define MAX_DCCAP_TEMP_SEVERE       6

// TRACK_INFORMATION
#define TRACK_LENGTH                100
#define DISTANCE_THRESHOLD          6

#define REPORT_FAULT(severity, fault, phase_msk) {                      \
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_FAULT_REPORT);   \
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, severity);       \
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_ERROR_CODE, fault);             \
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_PHASE_MASK, phase_msk);         \
    if (CANBus_put_frame(&tx_frame) != HAL_OK) { Error_Handler(); }     \
}

typedef enum {
    Idle,
    Brake,
    AutoPilot,
    ManualControl,
    EmergencyBrake,
    NormalDangerFault,
    SevereDangerFault,
    NoFault,
} State_t;

class StateMachineThread{
    public:
        static void initialize();
        static void setState(State_t state);

    private:
        static RTOSThread thread;
        static State_t state;
        static uint8_t enable_fault_check;

    private:
        static void ack_state_change();
        static void nack_state_change();

    private:
        static uint8_t fault_checking_routine(
            MCSeverityCode severity, float max_phase_volt, float min_phase_volt, float max_curr,
            float max_cap_volt,   float min_cap_volt,   float max_fet_temp);

        static State_t NormalFaultChecking(void);
        static State_t SevereFaultChecking(void);

        static uint8_t check_phase_overvolt(float threshold, uint8_t* phase_msk);
        static uint8_t check_phase_undervolt(float threshold, uint8_t* phase_msk);
        static uint8_t check_phase_overcurr(float threshold, uint8_t* phase_msk);
        static uint8_t check_cap_overvolt(float threshold);
        static uint8_t check_cap_undervolt(float threshold);
        static uint8_t check_fet_overtemp(float threshold, uint8_t* phase_msk);


    private:
        static void runStateMachine(void* arg);

        static State_t InitializeEvent(void);
        static State_t InitializeFaultEvent(void);
        static State_t IdleEvent(void);
        static State_t MinorDangerFaultEvent(void);
        static State_t AutoPilotEvent(void);
        static State_t SevereDangerFaultEvent(void);
        static State_t NormalDangerFaultEvent(void);
        static State_t ManualControlEvent(void);
        static State_t NoFaultEvent(void);
};

