/*
 * SVPWM.h
 *
 *  Created on: Aug 6, 2020
 *      Author: Chawthri
 */

#ifndef INC_SVPWM_H_
#define INC_SVPWM_H_

#include "VHZ.h"
#include "main.h"

typedef struct 	{
		float  ModIndex;
		float  Freq;
        float  SwitchFreq;
        float  Alpha;
        float  NewEntry;
        float  SectorPointer;
		float  U;
        float  V;
		float  W;
		float  StepAngle;
		float  EntryOld;
		float  ta;
		float  tb;
        float  to;
} SVPWM;

#define	PI_THIRD		1.04719755119660f
#define RADIANS 		6.28318530718f
#define SIN_PI_THIRD 	0.86602540378f

void SVPWM_Update(SVPWM *svm, VHZPROFILE *vhz, TIM_HandleTypeDef *htim);
void SVPWM_Init(SVPWM *svm, float SwitchFreq);

#endif /* INC_SVPWM_H_ */
