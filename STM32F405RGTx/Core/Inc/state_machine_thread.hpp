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
#define NUM_MOSFETS 6
#define NUM_PHASE_OUTPUTS 3

// Severe Fault
#define MAX_MOSFET_TEMP_SEVERE 70.0
#define MAX_DCVOLTAGE_SEVERE 4.0
#define MIN_DCVOLTAGE_SEVERE 1.8
#define MAX_VOLTAGE_SEVERE 4.0
#define MIN_VOLTAGE_SEVERE 1.0
#define MAX_TEMP_SEVERE 70.0
#define MAX_CURRENT_SEVERE 50.0

// Normal Fault
#define MAX_MOSFET_TEMP_NORMAL 60.0
#define MAX_DCVOLTAGE_NORMAL 4.0
#define MIN_DCVOLTAGE_NORMAL 1.8
#define MAX_VOLTAGE_NORMAL 3.8
#define MIN_VOLTAGE_NORMAL 1.8
#define MAX_TEMP_NORMAL 60.0
// #define MIN_CURRENT_NORMAL 5.0
#define MAX_CURRENT_NORMAL 40.0

#define MIN_OVERVOLT_FAULTS 5
#define MIN_UNDERVOLT_FAULTS 5
#define MIN_TEMP_FAULTS 5

// DC_CAPTEMP
#define MAX_DCCAP_TEMP_NORMAL 5
#define MAX_DCCAP_TEMP_SEVERE 6

// TRACK_INFORMATION
#define TRACK_LENGTH 100
#define DISTANCE_THRESHOLD 6

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
        static State_t CurrentState;
        static State_t NewState;
        static void SetLedColour(float R, float G, float B);
        static void runStateMachine(void *arg);
        static void SendCANHeartbeat(void);
        static void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
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

