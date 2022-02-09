/* state_machine.h
 *
 *    Created on: Jul. 11, 2021
 *            Author: Kanishk Kashyap
 */

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

// Constants
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
using namespace std;

typedef enum
{
    Initialize,
    InitializeFault,
    Idle,
    Brake,
    AutoPilot,
    ManualControl,
    EmergencyBrake,
    Run,
    Stop,
    Sleep,
    NormalDangerFault,
    SevereDangerFault,
    NoFault,
    Reset
} State_t;

typedef State_t (*pfEvent)(void);
class MCStateMachine
{
public:
    void SetLedColour(float R, float G, float B);
    void SendCanHeartBeat(void);
    State_t State;
    pfEvent Event;
    State_t NormalFaultChecking(void);
    State_t SevereFaultChecking(void);
    State_t InitializeEvent(void);
    State_t InitializeFaultEvent(void);
    State_t IdleEvent(void);
    State_t MinorDangerFaultEvent(void);
    State_t AutoPilotEvent(void);
    State_t SevereDangerFaultEvent(void);
    State_t ManualControlEvent(void);
    State_t AutoPilotEvent(void);
};

#endif