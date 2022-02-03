/*
 * state_machine.h
 *
 *    Created on: Jul. 11, 2021
 *            Author: Kanishk Kashyap
 */

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

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
    State_t State;
    pfEvent Event;

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
