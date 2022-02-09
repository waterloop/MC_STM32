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
    float avg_cell_temp = 0;
    for (uint8_t i = 0; i < NUM_CELLS; i++)
    {
        avg_cell_temp += motor_controller_data.battery.cells[i].temp;
    }
    avg_cell_temp /= NUM_CELLS;

    CANFrame tx_frame0 = CANFrame_init(BATTERY_PACK_CURRENT.id);
    CANFrame_set_field(&tx_frame0, BATTERY_PACK_CURRENT, FLOAT_TO_UINT(global_bms_data.battery.current));
    CANFrame_set_field(&tx_frame0, CELL_TEMPERATURE, FLOAT_TO_UINT(avg_cell_temp));

    CANFrame tx_frame1 = CANFrame_init(BATTERY_PACK_VOLTAGE.id);
    CANFrame_set_field(&tx_frame1, BATTERY_PACK_VOLTAGE, FLOAT_TO_UINT(global_bms_data.battery.voltage));
    CANFrame_set_field(&tx_frame1, STATE_OF_CHARGE, FLOAT_TO_UINT(global_bms_data.battery.soc));

    CANFrame tx_frame2 = CANFrame_init(BUCK_TEMPERATURE.id);
    CANFrame_set_field(&tx_frame2, BUCK_TEMPERATURE, FLOAT_TO_UINT(global_bms_data.buck_temp));
    // CANFrame_set_field(&tx_frame2, BMS_CURRENT, FLOAT_TO_UINT(0));

    CANFrame tx_frame3 = CANFrame_init(MC_CAP_VOLTAGE.id);
    CANFrame_set_field(&tx_frame3, MC_CAP_VOLTAGE, FLOAT_TO_UINT(global_bms_data.mc_cap_voltage));

    if (CANBus_put_frame(&tx_frame0) != HAL_OK)
    {
        Error_Handler();
    }
    if (CANBus_put_frame(&tx_frame1) != HAL_OK)
    {
        Error_Handler();
    }
    if (CANBus_put_frame(&tx_frame2) != HAL_OK)
    {
        Error_Handler();
    }
    if (CANBus_put_frame(&tx_frame3) != HAL_OK)
    {
        Error_Handler();
    }
}

// Returns normal fault state or no fault based on current, voltage, and temperature measurements
State_t MCStateMachine::NormalFaultChecking(void)
{
    float current = global_bms_data.battery.current;
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