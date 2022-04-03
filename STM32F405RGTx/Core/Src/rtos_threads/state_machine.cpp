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

uint8_t idle_state_id;
uint8_t run_state_id;
uint8_t mc_error_code;

// global colour variables (used by tim7 interrupt)
float red;
float green;
float blue;

// global blink variables
bool flash;
bool ledON;

void StateMachineThread::setState(State_t target_state) {
    NewState = target_state;
}

void SetLedColour(float R, float G, float B)
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
    for (int i = 0; i < NUM_MOSFETS; ++i) {
        float mosfetTemp = g_mc_data.fet_temps[i];
        if (mosfetTemp > MAX_MOSFET_TEMP_NORMAL) {
            ++temp_faults;
        }
        if (temp_faults > MIN_TEMP_FAULTS) {
            // TODO: add error code
            return NormalDangerFault;
        }
    }
    // check undervolts and overvolts for phase outputs
    if (g_mc_data.pVa < MIN_VOLTAGE_NORMAL) {
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
        return NormalDangerFault;
    }
    if (undervolt_faults > MIN_UNDERVOLT_FAULTS) {
        return NormalDangerFault;
    }

    // Check current for phase outputs
    if (g_mc_data.pIa > MAX_CURRENT_NORMAL || g_mc_data.pIb > MAX_CURRENT_NORMAL || g_mc_data.pIc > MAX_CURRENT_NORMAL) {
        //TODO: error code
        return NormalDangerFault;
    }

    // DC fault checking
    if (g_mc_data.dc_voltage > MAX_DCVOLTAGE_NORMAL) {
        // TODO: error code
        return NormalDangerFault;
    }
    else if (g_mc_data.dc_voltage < MIN_DCVOLTAGE_NORMAL) {
        // TODO: error code
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
            // TODO: add error code
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
        return NormalDangerFault;
    }
    if (undervolt_faults > MIN_UNDERVOLT_FAULTS) {
        return NormalDangerFault;
    }

    // Check current for phase outputs
    if (g_mc_data.pIa > MAX_CURRENT_SEVERE || g_mc_data.pIb > MAX_CURRENT_SEVERE || g_mc_data.pIc > MAX_CURRENT_SEVERE) {
        // error code
        return SevereDangerFault;
    }

    // DC fault checking
    if (g_mc_data.dc_voltage > MAX_DCVOLTAGE_SEVERE) {
        // error code
        return SevereDangerFault;
    }
    else if (g_mc_data.dc_voltage < MIN_DCVOLTAGE_SEVERE) {
        // error code
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
    // Send ACK on CAN when stop complete
    if (NewState != CurrentState) {
        CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
        CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_STATE_ID_ACK_NACK, idle_state_id);
        if (CANBus_put_frame(&tx_frame) != HAL_OK) { Error_Handler(); }

        // TODO: Listen to CAN bus and get TARGET_SPEED from MANUAL_CONTROL
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
    if (CurrentState != NewState) {
        CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_SEVERITY_CODE.id);
        CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, 0x01); 
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
    
    // Report fault on CAN
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_SEVERITY_CODE.id);
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, 0x01);
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
    CANFrame_set_field(&tx_frame, MOTOR_CONTROLLER_SEVERITY_CODE, 0x00);
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

void StateMachineThread::initialize() {
    thread = RTOSThread(
        "state_machine_thread",
        1024 * 3,
        osPriorityNormal,
        runStateMachine);
}

void StateMachineThread::runStateMachine(void *argument)
{
    start_timers();
    while (1)
    {
        CurrentState = NewState;
        switch (NewState)
        {
        case Initialize:
            NewState = InitializeEvent();
            break;

        case InitializeFault:
            red = 50.0;
            green = 0.0;
            blue = 0.0;
            flash = false;
            NewState = InitializeFaultEvent();
            break;

        case Idle:
            red = 0.0;
            green = 50.0;
            blue = 0.0;
            flash = false;
            NewState = IdleEvent();
            break;

        case AutoPilot:
            red = 50.0;
            green = 50.0;
            blue = 0.0;
            flash = false;
            NewState = AutoPilotEvent();
            break;

        case ManualControl:
            red = 0.0;
            green = 0.0;
            blue = 50.0;
            flash = false;
            NewState = ManualControlEvent();
            break;

        case NormalDangerFault:
            red = 50.0;
            green = 0.0;
            blue = 0.0;
            flash = false;
            NewState = NormalDangerFaultEvent();
            break;

        case SevereDangerFault:
            red = 50.0;
            green = 0.0;
            blue = 0.0;
            flash = true;
            NewState = SevereDangerFaultEvent();
            break;

        default:
            Error_Handler();
        }
        SendCANHeartbeat();
        osDelay(STATE_MACHINE_PERIODICITY);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (flash)
    {
        if (ledON)
        {
            SetLedColour(0.0, 0.0, 0.0);
        }
        else
        {
            SetLedColour(red, green, blue);
        }

        ledON = !ledON;
    }
    else
    { // non-flashing colour
        SetLedColour(red, green, blue);
    }
}
