#include <stdio.h>
#include <string.h>
#include "state_machine.h"
#include "threads.h"
#include "cmsis_os.h"
#include "main.h"
#include "can.h"

void MCStateMachine::SetLedColour(float R, float G, float B)
{
    set_led_intensity(RED, R);
    set_led_intensity(GREEN, G);
    set_led_intensity(BLUE, B);
}

void SendCANHeartbeat(void)
{
    float avg_MC_current = (MotorControllerData.pIa + MotorControllerData.pIb + MotorControllerData.pIc) / 3;

    CANFrame tx_frame0 = CANFrame_init(POD_SPEED.id);
    CANFrame_set_field(&tx_frame0, POD_SPEED, FLOAT_TO_UINT(MotorControllerData.cur_speed));
    CANFrame_set_field(&tx_frame0, MOTOR_CURRENT, FLOAT_TO_UINT(MotorControllerData.cur_speed));

    CANFrame tx_frame1 = CANFrame_init(BATTERY_CURRENT.id);
    CANFrame_set_field(&tx_frame1, BATTERY_CURRENT, FLOAT_TO_UINT(global_bms_data.battery.current));
    CANFrame_set_field(&tx_frame1, BATTERY_PACK_VOLTAGE, FLOAT_TO_UINT(global_bms_data.battery.voltage));

    if (CANBus_put_frame(&tx_frame0) != HAL_OK)
    {
        Error_Handler();
    }
    if (CANBus_put_frame(&tx_frame1) != HAL_OK)
    {
        Error_Handler();
    }
}

// Returns normal fault state or no fault based on current, voltage, and temperature measurements
State_t MCStateMachine::NormalFaultChecking(void)
{
    int overvolt_faults = 0;
    int undervolt_faults = 0;
    int temp_faults = 0;
    // check for temperature faults
    for (int i = 0; i < NUM_MOSFETS; ++i)
    {
        float mosfetTemp = MotorControllerData.FET_temps[i];
        if (mosfetTemp > MAX_MOSFET_TEMP_NORMAL)
        {
            ++temp_faults;
        }
        if (temp_faults > MIN_TEMP_FAULTS)
        {
            // TODO: add error code
            return NormalDangerFault;
        }
    }
    // check undervolts and overvolts for phase outputs
    if (MotorControllerData.pVa < MIN_VOLTAGE_NORMAL)
    {
        undervolt_faults++;
    }
    if (MotorControllerData.pVb < MIN_VOLTAGE_NORMAL)
    {
        undervolt_faults++;
    }
    if (MotorControllerData.pVc < MIN_VOLTAGE_NORMAL)
    {
        undervolt_faults++;
    }

    if (MotorControllerData.pVa > MAX_VOLTAGE_NORMAL)
    {
        overvolt_faults++;
    }
    if (MotorControllerData.pVb > MAX_VOLTAGE_NORMAL)
    {
        overvolt_faults++;
    }
    if (MotorControllerData.pVc > MAX_VOLTAGE_NORMAL)
    {
        overvolt_faults++;
    }

    // Check current for phase outputs
    if (MotorControllerData.pIa > MAX_CURRENT_NORMAL || MotorControllerData.pIb > MAX_CURRENT_NORMAL || MotorControllerData.pIc > MAX_CURRENT_NORMAL)
    {
        // error code
        return NormalDangerFault;
    }

    // DC fault checking
    if (dc_voltage > MAX_DCVOLTAGE_NORMAL)
    {
        // error code
        return NormalDangerFault;
    }
    else if (dc_voltage < MIN_DCVOLTAGE_NORMAL)
    {
        return NormalDangerFault;
    }
    // what does cap temp mean for dc_cap_temp
    return NoFault;
}

State_t MCStateMachine::SevereFaultChecking(void)
{
    int overvolt_faults = 0;
    int undervolt_faults = 0;
    int temp_faults = 0;
    // check for temperature faults
    for (int i = 0; i < NUM_MOSFETS; ++i)
    {
        float mosfetTemp = MotorControllerData.FET_temps[i];
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
    if (MotorControllerData.pVa < MIN_VOLTAGE_SEVERE)
    {
        undervolt_faults++;
    }
    if (MotorControllerData.pVb < MIN_VOLTAGE_SEVERE)
    {
        undervolt_faults++;
    }
    if (MotorControllerData.pVc < MIN_VOLTAGE_SEVERE)
    {
        undervolt_faults++;
    }

    if (MotorControllerData.pVa > MAX_VOLTAGE_SEVERE)
    {
        overvolt_faults++;
    }
    if (MotorControllerData.pVb > MAX_VOLTAGE_SEVERE)
    {
        overvolt_faults++;
    }
    if (MotorControllerData.pVc > MAX_VOLTAGE_SEVERE)
    {
        overvolt_faults++;
    }

    // Check current for phase outputs
    if (MotorControllerData.pIa > MAX_CURRENT_SEVERE || MotorControllerData.pIb > MAX_CURRENT_SEVERE || MotorControllerData.pIc > MAX_CURRENT_SEVERE)
    {
        // error code
        return SevereDangerFault;
    }

    // DC fault checking
    if (dc_voltage > MAX_DCVOLTAGE_SEVERE)
    {
        // error code
        return SevereDangerFault;
    }
    else if (dc_voltage < MIN_DCVOLTAGE_SEVERE)
    {
        return SevereDangerFault;
    }
    // what does cap temp mean for dc_cap_temp
    return NoFault;
}

State_t MCStateMachine::InitializeEvent(void)
{
    return Idle;
}

State_t MCStateMachine::IdleEvent(void)
{
    SetLEDColour(0.0, 50.0, 0.0);
    State_t normal_fault_check = NormalFaultChecking();
    State_t severe_fault_check = SevereFaultChecking();
    if (severe_fault_check == SevereDangerFault)
    {
        return severe_fault_check;
    }
    else if (normal_fault_check == NormalDangerFault)
    {
        return normal_fault_check;
    }

    // need to understand pins

    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        idle_state_id = state_id;
        if (state_id == AUTO_PILOT) // how do i get length of track
        {
            return AutoPilot;
        }
        else if (state_id == MANUAL_OPERATION_WAITING)
    }

    return Idle;
}

State_t AutoPilotEvent(void)
{
    // Set LED colour to yellow
    SetLEDColour(50.0, 50.0, 0.0);

    // Send ACK on CAN when stop complete
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK_ID, run_state_id);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK, 0x00);
    if (CANBus_put_frame(&tx_frame) != HAL_OK)
    {
        Error_Handler();
    }

    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == EMERGENCY_BRAKE || state_id == SYSTEM_FAILURE)
        {
            State_t normal_fault_check = NormalFaultChecking();
            State_t severe_fault_check = SevereFaultChecking();
            if (severe_fault_check == SevereDangerFault)
            {
                return severe_fault_check;
            }
            else if (normal_fault_check == NormalDangerFault)
            {
                return normal_fault_check;
            }
        }
        else if (state_id == BRAKING)
        {
            return Idle;
        }
    }
    return Idle;
}

State_t ManualControl(void)
{
    // Set LED colour to blue
    SetLEDColour(0, 0, 50.0);

    // Send ACK on CAN when stop complete
    CANFrame tx_frame = CANFrame_init(MOTOR_CONTROLLER_STATE_CHANGE_ACK_NACK);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK_ID, run_state_id);
    CANFrame_set_field(&tx_frame, STATE_CHANGE_ACK, 0x00);
    if (CANBus_put_frame(&tx_frame) != HAL_OK)
    {
        Error_Handler();
    }

    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == EMERGENCY_BRAKE || state_id == SYSTEM_FAILURE)
        {
            State_t normal_fault_check = NormalFaultChecking();
            State_t severe_fault_check = SevereFaultChecking();
            if (severe_fault_check == SevereDangerFault)
            {
                return severe_fault_check;
            }
            else if (normal_fault_check == NormalDangerFault)
            {
                return normal_fault_check;
            }
        }
        else if (state_id == BRAKING)
        {
            return Idle;
        }
    }
    return Idle;
}

State_t InitializeFaultEvent(void)
{

    CANFrame tx_frame = CANFrame_init(MC_SEVERITY_CODE.id);
    CANFrame_set_field(&tx_frame, MC_SEVERITY_CODE, INITIAL_FAULT); // dummy
    CANFrame_set_field(&tx_frame, ERROR_CODE, mc_error_code);
    SetLEDColour(50.0, 0.0, 0.0);
    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == RESTING)
        {
            return Idle;
        }
        else
        {
            return InitializeFault;
        }
    }
    return InitializeFault;
}

State_t NormalDangerFaultEvent(void)
{
    // Set LED colour to red
    SetLEDColour(50.00, 0.0, 0.0);

    // Report fault on CAN
    CANFrame tx_frame = CANFrame_init(BMS_SEVERITY_CODE.id);
    CANFrame_set_field(&tx_frame, MC_SEVERITY_CODE, DANGER);
    CANFrame_set_field(&tx_frame, ERROR_CODE, mc_error_code);
    if (CANBus_put_frame(&tx_frame) != HAL_OK)
    {
        Error_Handler();
    }

    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == RESTING)
        {
            return Idle;
        }
        else
        {
            return NormalDangerFault;
        }
    }
    return NormalDangerFault;
}

State_t SevereDangerFault(void)
{
    // Set LED colour to red
    SetLEDColour(50.00, 0.0, 0.0);

    // Report fault on CAN
    CANFrame tx_frame = CANFrame_init(MC_SEVERITY_CODE.id);
    CANFrame_set_field(&tx_frame, SEVERITY_CODE, DANGER);
    CANFrame_set_field(&tx_frame, ERROR_CODE, mc_error_code);
    if (CANBus_put_frame(&tx_frame) != HAL_OK)
    {
        Error_Handler();
    }

    // Receive CAN frame
    if (!Queue_empty(&RX_QUEUE))
    {
        CANFrame rx_frame = CANBus_get_frame();
        uint8_t state_id = CANFrame_get_field(&rx_frame, STATE_ID);
        if (state_id == RESTING)
        {
            return Idle;
        }
        else
        {
            return NormalDangerFault;
        }
    }
    return NormalDangerFault;
}