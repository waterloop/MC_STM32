using namespace std;

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

class MCStateMachine
{

    void InitializeEvent(void);
    void InitializeFaultEvent(void);
    void IdleEvent(void);
    void MinorDangerFaultEvent(void);
    void AutoPilotEvent(void);
    void SevereDangerFaultEvent(void);
    void ManualControlEvent(void);
    void AutoPilotEvent(void);
};