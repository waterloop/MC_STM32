/* state_machine.h
 *
 *    Created on: Jul. 11, 2021
 *            Author: Kanishk Kashyap
*/
#pragma once

#include "can.h"
#include "cmsis_os.h"
#include "can.h"
#include "util.hpp"


// Constants - These numbers are arbritrary should be changed after testing
// Mosfet Array Length
#define NUM_MOSFETS                 6
#define NUM_PHASE_OUTPUTS           3

// Severe Fault
#define MAX_MOSFET_TEMP_SEVERE      70.0F
#define MAX_DCVOLTAGE_SEVERE        4.0F
#define MIN_DCVOLTAGE_SEVERE        1.8F
#define MAX_VOLTAGE_SEVERE          4.0F
#define MIN_VOLTAGE_SEVERE          1.0F
#define MAX_TEMP_SEVERE             70.0F
#define MAX_CURRENT_SEVERE          50.0F

// Normal Fault
#define MAX_FET_TEMP_NORMAL      60.0F
#define MAX_DCVOLTAGE_NORMAL        4.0F
#define MIN_DCVOLTAGE_NORMAL        1.8F
#define MAX_VOLTAGE_NORMAL          3.8F
#define MIN_VOLTAGE_NORMAL          1.8F
#define MAX_TEMP_NORMAL             60.0F
#define MAX_CURRENT_NORMAL          40.0F

// DC_CAPTEMP
#define MAX_DCCAP_TEMP_NORMAL       5
#define MAX_DCCAP_TEMP_SEVERE       6

// TRACK_INFORMATION
#define TRACK_LENGTH                100
#define DISTANCE_THRESHOLD          6

// using namespace std;

typedef enum {
    Initialize,
    InitializeFault,
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

    private:
        static void send_CAN_hearbeat(void);

        static void report_phase_overvolt(uint8_t phase_msk);
        static void report_phase_undervolt(uint8_t phase_msk);
        static void report_phase_overcurrent(uint8_t phase_msk);
        static void report_phase_overtemp(uint8_t phase_msk);
        static void report_DC_cap_overvolt();
        static void report_DC_cap_undervolt();
        static void report_motor_stall();

        static void ack_state_change();
        static void nack_state_change();


    private:
        static void runStateMachine(void* arg);

        static State_t NormalFaultChecking(void);
        static State_t SevereFaultChecking(void);
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

