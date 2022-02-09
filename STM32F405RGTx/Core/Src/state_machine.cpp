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
    for (int i = 0; i < NUM_MOSFETS; ++i)
    {
        float mosfetTemp = MotorControllerData[i];
        if (mosfetTemp > MAX_MOSFET_TEMP_NORMAL)
        {
            ++temp_faults;
        }
        if (overvolt_faults > MIN_OVERVOLT_FAULTS || undervolt_faults > MIN_UNDERVOLT_FAULTS || temp_faults > MIN_TEMP_FAULTS)
        {
            if (overvolt_faults > MIN_OVERVOLT_FAULTS)
            {
                bms_error_code = BATTERY_OVERVOLTAGE_ERR;
            }
            else if (undervolt_faults > MIN_UNDERVOLT_FAULTS)
            {
                /*
                    TODO: Spelling mistake in config.h
                    BATTERY_UNDERVOLTAGE_ERR should be BATTERY_UNDERVOLTAGE_ERR
                */
                bms_error_code = BATTERY_UNDERVOLTAGE_ERR;
            }
            else
            {
                // TODO: why is there no BATTERY_TEMPERATURE_ERR error code?
            }
            return NormalDangerFault;
        }
    }
    return NoFault;
}