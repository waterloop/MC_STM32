/*
 * DEFINES.h
 *
 *  Created on: Aug. 13, 2020
 *      Author: Chawthri
 *
 *  Modified: Feb. 07, 2021
 *  	Shaheer
 */

#ifndef INC_DEFINES_H_
#define INC_DEFINES_H_

#define MODE 0 /* 0: Fixed Output 1: V/F Open Loop 2: V/F Closed LoopW*/

#define	THERMISTOR_RESISTANCE 10 // 10k thermistor
#define THERMISTOR_BETA 3936
#define ROOM_TEMP (25+273.15) // only change the 25, the +273.15 is to convert to Kelvin for calcs
#define	INPUT_VOLTAGE 3.3
#define ADC_VOLTAGE_CONVERSION (4095 / INPUT_VOLTAGE)
#define R1 50 // top of voltage divider
#define R2 2.2 // bottom of voltage divider
#define SCALING_FACTOR (R2/(R1+R2))
#define DC_RESISTOR 0.0005 //shunt resistor for DC
#define GAIN 100 //gain of INA80A3 for low-side DC current sensing
#define I_RESISTOR 0.0005 // shunt for IGBTs
#define GAIN_IGBT 40

// Predefined values for temps/power
#define MOTOR_TEMP_MAX 70
#define MOTOR_VOLTAGE 320   //45, -0.3V
#define MOTOR_CURRENT 40
#define MOTOR_PHASE1_CURRENT 6.66
#define MOTOR_PHASE2_CURRENT 6.66
#define MOTOR_PHASE3_CURRENT 6.66

//TODO: UPDATE LATER ON
#define BATTERY_TEMP
#define BATTERY_VOLTAGE
#define BATTERY_CURRENT

#define IGBT_TEMP

#endif /* INC_DEFINES_H_ */
