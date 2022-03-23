#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "state_machine.hpp"
#include "threads.hpp"
#include "cmsis_os.h"
#include "main.h"
#include "mc.hpp"
#include "can.h"
#include "timer_utils.h"

RTOSThread StateMachineThread::thread;

State_t StateMachineThread::NewState;
State_t StateMachineThread::CurrentState;
StateMachine *StateMachineThread::SM;

uint8_t idle_state_id;
uint8_t run_state_id;
uint8_t mc_error_code;
uint8_t phase_option;
uint32_t dummy_code = 0x01;

void StateMachineThread::setState(State_t target_state) {
    NewState = target_state;
}

void StateMachineThread::SetLedColour(float R, float G, float B)
{
    set_led_intensity(RED, R);
    set_led_intensity(GREEN, G);
    set_led_intensity(BLUE, B);
}

void StateMachineThread::SendCANHeartbeat(void)
{
    float avg_MC_current = (g_mc_data.pIa + g_mc_data.pIb + g_mc_data.pIc) / 3;
    float avg_MC_voltage = (g_mc_data.pVa + g_mc_data.pVb + g_mc_data.pVc) / 3;

    CANFrame tx_frame0 = CANFrame_init(MC_POD_SPEED.id);
    CANFrame_set_field(&tx_frame0, MC_POD_SPEED, FLOAT_TO_UINT(g_mc_data.curr_speed));
    // What value to use for motor_current
    CANFrame_set_field(&tx_frame0, MOTOR_CURRENT, FLOAT_TO_UINT(avg_MC_current));

    CANFrame tx_frame1 = CANFrame_init(BATTERY_CURRENT.id);
    CANFrame_set_field(&tx_frame1, BATTERY_CURRENT, FLOAT_TO_UINT(avg_MC_current));
    CANFrame_set_field(&tx_frame1, BATTERY_VOLTAGE, FLOAT_TO_UINT(avg_MC_voltage));

    if (CANBus_put_frame(&tx_frame0) != HAL_OK) { Error_Handler(); }
    if (CANBus_put_frame(&tx_frame1) != HAL_OK) { Error_Handler(); }
}

// Returns normal fault state or no fault based on current, voltage, and temperature measurements
State_t StateMachineThread::NormalFaultChecking(void)
{
    int overvolt_faults = 0;
    int undervolt_faults = 0;
    int temp_faults = 0;
    // check for temperature faults
    for (int i = NUM_MOSFETS-1; i < NUM_MOSFETS; ++i) {
        float mosfetTemp = g_mc_data.fet_temps[i];
        if (mosfetTemp > MAX_MOSFET_TEMP_NORMAL) {
            ++temp_faults;
        }
        if (temp_faults > MIN_TEMP_FAULTS) {
            mc_error_code = PHASE_TEMPERATURE;
            return NormalDangerFault;
        }
    }
    // check undervolts and overvolts for phase outputs
    if (g_mc_data.pVc < MIN_VOLTAGE_NORMAL) {
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
        mc_error_code = PHASE_OVERVOLTAGE;
        return NormalDangerFault;
    }
    if (undervolt_faults > MIN_UNDERVOLT_FAULTS) {
        mc_error_code = PHASE_UNDERVOLTAGE;
        return NormalDangerFault;
    }

    // Check current for phase outputs
    if (g_mc_data.pIa > MAX_CURRENT_NORMAL) {
        mc_error_code = PHASE_OVERCURRENT;
        phase_option = PHASE_A;
        return NormalDangerFault;
    } else if (g_mc_data.pIb > MAX_CURRENT_NORMAL) {
        mc_error_code = PHASE_OVERCURRENT;
        phase_option = PHASE_B;
        return NormalDangerFault;
    } else if (g_mc_data.pIc > MAX_CURRENT_NORMAL)
    {
        mc_error_code = PHASE_OVERCURRENT;
        phase_option = PHASE_C;
        return NormalDangerFault;
    }

    // DC fault checking
    if (g_mc_data.dc_voltage > MAX_DCVOLTAGE_NORMAL) {
        mc_error_code = DC_CAP_OVERVOLTAGE;
        return NormalDangerFault;
    }
    else if (g_mc_data.dc_voltage < MIN_DCVOLTAGE_NORMAL) {
        mc_error_code = DC_CAP_UNDERVOLTAGE;
        return NormalDangerFault;
    }

    if (g_mc_data.dc_cap_temp > MAX_DCCAP_TEMP_NORMAL) {
        // TODO: error code
        return NormalDangerFault;
    }
    return NoFault;
}

State_t StateMachineThread::SevereFaultChecking(void)
{
    int overvolt_faults = 0;
    int undervolt_faults = 0;
    int temp_faults = 0;
    bool first_phase = true;
    // check for temperature faults
    for (int i = 0; i < NUM_MOSFETS; ++i)
    {
        float mosfetTemp = g_mc_data.fet_temps[i];
        if (mosfetTemp > MAX_MOSFET_TEMP_SEVERE)
        {
            ++temp_faults;
        }
        if (temp_faults > MIN_TEMP_FAULTS)
        {
            mc_error_code = PHASE_TEMPERATURE;
            return SevereDangerFault;
        }
    }
    // check undervolts and overvolts for phase outputs
    if (g_mc_data.pVa < MIN_VOLTAGE_SEVERE) {
        undervolt_faults++;
    }
    if (g_mc_data.pVb < MIN_VOLTAGE_SEVERE) {
        undervolt_faults++;
    }
    if (g_mc_data.pVc < MIN_VOLTAGE_SEVERE) {
        undervolt_faults++;
    }

    if (g_mc_data.pVa > MAX_VOLTAGE_SEVERE) {
        overvolt_faults++;
    }
    if (g_mc_data.pVb > MAX_VOLTAGE_SEVERE) {
        overvolt_faults++;
    }
    if (g_mc_data.pVc > MAX_VOLTAGE_SEVERE) {
        overvolt_faults++;
    }

    //  TODO: after checking for faults return state
    if (overvolt_faults > MIN_OVERVOLT_FAULTS) {
        mc_error_code = PHASE_OVERVOLTAGE;
        return NormalDangerFault;
    }
    if (undervolt_faults > MIN_UNDERVOLT_FAULTS) {
        mc_error_code = PHASE_UNDERVOLTAGE;
        return NormalDangerFault;
    }

    if (g_mc_data.pIa > MAX_CURRENT_SEVERE)
    {
        mc_error_code = PHASE_OVERCURRENT;
        phase_option = PHASE_A;
        return NormalDangerFault;
    }
    else if (g_mc_data.pIb > MAX_CURRENT_SEVERE)
    {
        mc_error_code = PHASE_OVERCURRENT;
        phase_option = PHASE_B;
        return NormalDangerFault;
    }
    else if (g_mc_data.pIc > MAX_CURRENT_SEVERE)
    {
        mc_error_code = PHASE_OVERCURRENT;
        phase_option = PHASE_C;
        return NormalDangerFault;
    }

    // DC fault checking
    if (g_mc_data.dc_voltage > MAX_DCVOLTAGE_SEVERE) {
        mc_error_code = DC_CAP_OVERVOLTAGE;
        return SevereDangerFault;
    }
    else if (g_mc_data.dc_voltage < MIN_DCVOLTAGE_SEVERE) {
        mc_error_code = DC_CAP_UNDERVOLTAGE;
        return SevereDangerFault;
    }
    if (g_mc_data.dc_cap_temp > MAX_DCCAP_TEMP_SEVERE) {
        return SevereDangerFault;
    }
    return NoFault;
}

State_t StateMachineThread::InitializeEvent(void) {
    return Idle;
}

State_t StateMachineThread::IdleEvent(void) {
    SetLedColour(0.0, 50.0, 0.0);
/*     State_t normal_fault_check = NormalFaultChecking();
    State_t severe_fault_check = SevereFaultChecking();
    if (severe_fault_check == SevereDangerFault) {
        return severe_fault_check;
    }
    else if (normal_fault_check == NormalDangerFault) {
        return normal_fault_check;
    }
 */
    // TODO: Make list of the threads that need to be turned on or turned off
        // On: Measurements Thread, CANZThread
        // Off: PIDThread, VHzThread, SVPWMThread
    MeasurementsThread::resumeMeasurements();

    if (!Queue_empty(&RX_QUEUE)) {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        idle_state_id = state_id;
        if (state_id == AUTO_PILOT) {
            return AutoPilot;
        }
        else if (state_id == MANUAL_OPERATION_WAITING) {
            return ManualControl;
        } 
    }

    return Idle;
}

State_t StateMachineThread::AutoPilotEvent(void)
{
    // Set LED colour to yellow
    SetLedColour(50.0, 50.0, 0.0);

    // Send ACK on CAN when stop complete 
    if (NewState != CurrentState) {
        CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
        CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_STATE_ID_ACK_NACK, idle_state_id);
        if (CANBus_put_frame(&tx_frame) != HAL_OK) { Error_Handler(); }
    }

    // TODO: Make list of the threads that need to be turned on or turned off
        // On: Measurements Thread, CANThread
        // Off: PIDThread, VHzThread, SVPWMThread
    MeasurementsThread::resumeMeasurements();

    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE)) {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        int distance_to_end = TRACK_LENGTH - g_mc_data.curr_pos;
        if (state_id == EMERGENCY_BRAKE || state_id == SYSTEM_FAILURE) {
            return SevereDangerFault;
            // TODO: Talk to Ryan/Dev when to enter MinorDangerFault
        }
        else if (state_id == BRAKING || distance_to_end <= DISTANCE_THRESHOLD) {
            return Idle;
        } 
    }
    return AutoPilot;
}

State_t StateMachineThread::ManualControlEvent(void)
{
    // Set LED colour to blue
    SetLedColour(0, 0, 50.0);
    // Send ACK on CAN when stop complete
    if (NewState != CurrentState) {
        CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
        CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_STATE_ID_ACK_NACK, idle_state_id);
        if (CANBus_put_frame(&tx_frame) != HAL_OK) { Error_Handler(); }

        CANFrame tx_frame1 = CANFrame_init(MANUAL_CONTROL_1);
        uint8_t target_speed = CANFrame_get_field(&tx_frame1, TARGET_SPEED);
        CANFrame_set_field(&tx_frame1, TARGET_SPEED, target_speed);
        // Not sure what to put for target frequency
    }

    // List of the threads that need to be turned on or turned off
        // On: Measurements Thread, CANZThread, PIDThread, VHzThread, SVPWMThread,  StateMachineThread

    MeasurementsThread::resumeMeasurements();

    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        run_state_id = state_id;
        if (state_id == EMERGENCY_BRAKE || state_id == SYSTEM_FAILURE)
        {
            return SevereDangerFault;
    }
        else if (state_id == BRAKING)
        {
            return Idle;
        }
    }
    return ManualControl;
}

State_t StateMachineThread::InitializeFaultEvent(void)
{
    SetLedColour(50.0, 0.0, 0.0);

    if (CurrentState != NewState) {
        CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_SEVERITY_CODE.id);
        CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, WARNING); // dummy
        CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_ERROR_CODE, mc_error_code);
    }
    /* After everything is merged
    Put all the MOSFETs into HIGH-Z mode using the drv8323 drivers
        . The DRV drivers aren't merged in right now, so just put a comment to remind yourself to put that in once it does get merged
    (Probably) read the fault codes from the DRV
    Disable the PID, SVPWM, and VHz threads
    */
    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE)) {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == RESTING) {
            return Idle;
        }
    }
    return InitializeFault;
}

State_t StateMachineThread::NormalDangerFaultEvent(void)
{
    // Set LED colour to red
    SetLedColour(50.00, 0.0, 0.0);
    
    // Report fault on CAN
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_SEVERITY_CODE.id);
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, DANGER);
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_ERROR_CODE, mc_error_code);
    if (CANBus_put_frame(&tx_frame) != HAL_OK) {
        Error_Handler();
    }
    /* After everything is merged
    Put all the MOSFETs into HIGH-Z mode using the drv8323 drivers
        . The DRV drivers aren't merged in right now, so just put a comment to remind yourself to put that in once it does get merged
    (Probably) read the fault codes from the DRV
    Disable the PID, SVPWM, and VHz threads
    */
    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE)) {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == RESTING) {
            return Idle;
        }
        else {
            return NormalDangerFault;
        }
    }
    return NormalDangerFault;
}

State_t StateMachineThread::SevereDangerFaultEvent(void)
{
    bool flash = true;
    while(1) {
        flash ? SetLedColour(50.00, 0.0,0.0) : SetLedColour(0.0,0.0,0.0);
        flash = !flash;
        osDelay(200);
    }

    // TODO: Make list of the threads that need to be turned on or turned off
        // On: Measurements Thread, CANThread
        // Off: PIDThread, VHzThread, SVPWMThread
    MeasurementsThread::resumeMeasurements();
    /* After everything is merged
    Put all the MOSFETs into HIGH-Z mode using the drv8323 drivers
        . The DRV drivers aren't merged in right now, so just put a comment to remind yourself to put that in once it does get merged
    (Probably) read the fault codes from the DRV
    Disable the PID, SVPWM, and VHz threads
    */

    // Report fault on CAN
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_SEVERITY_CODE.id);
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, SEVERE);
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_ERROR_CODE, mc_error_code);
    if (CANBus_put_frame(&tx_frame) != HAL_OK) {
        Error_Handler();
    }

    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE)) {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == RESTING) {
            return Idle;
        }
        else {
            return NormalDangerFault;
        }
    }
    return NormalDangerFault;
}

State_t StateMachineThread::NoFaultEvent() {
    return NoFault;
}

void StateMachineThread::initialize()
{
    thread = RTOSThread(
        "state_machine_thread",
        1024 * 3,
        osPriorityNormal,
        runStateMachine);

    StateMachine stateMachine[9] = {
        {Initialize, InitializeEvent},
        {InitializeFault, InitializeFaultEvent},
        {Idle, IdleEvent},
        {AutoPilot, AutoPilotEvent},
        {ManualControl, ManualControlEvent},
        {NormalDangerFault, NormalDangerFaultEvent},
        {SevereDangerFault, SevereDangerFaultEvent},
        {NoFault, NoFaultEvent},
    };
    SM = stateMachine;
}

void StateMachineThread::runStateMachine(void *argument)
{
    while (1)
    {
        CurrentState = NewState;
        switch (NewState)
        {
        case Initialize:
            NewState = InitializeEvent();
            break;

        case InitializeFault:
            NewState = InitializeFaultEvent();
            break;

        case Idle:
            NewState = IdleEvent();
            break;

        case AutoPilot:
            NewState = AutoPilotEvent();
            break;

        case ManualControl:
            NewState = ManualControlEvent();
            break;

        case NormalDangerFault:
            NewState = NormalDangerFaultEvent();
            break;

        case SevereDangerFault:
            NewState = SevereDangerFaultEvent();
            break;

        default:
            Error_Handler();
        }
        SendCANHeartbeat();
        osDelay(200);
    }
}
