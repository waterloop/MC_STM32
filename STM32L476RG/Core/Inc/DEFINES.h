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
#define	INPUT_VOLTAGE 3.3
#define ADC_VOLTAGE_CONVERSION (4095 / INPUT_VOLTAGE)
#define R1 300 // 300 in actual design
#define R2 2.2 // 2.2 in actual design
#define SCALING_FACTOR (R2/(R1+R2))
#define I_RESISTOR 0.0001 //10k resistor
#endif /* INC_DEFINES_H_ */
