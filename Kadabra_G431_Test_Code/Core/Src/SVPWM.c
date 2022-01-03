/*
 * SVPWM.h
 *
 *  Created on: Aug 8, 2020
 *      Author: Chawthri
 */
#include "SVPWM.h"
#include "math.h"

void SVPWM_Init(SVPWM *svm, float SwitchFreq) {
	svm->ModIndex = 0;
	svm->Freq = 0;
	svm->SwitchFreq = SwitchFreq;
	svm->Alpha = 0;
	svm->NewEntry = 0;
	svm->SectorPointer = 0;
	svm->U = 0;
	svm->V = 0;
	svm->W = 0;
	svm->StepAngle = 0;
	svm->EntryOld = 0;
	svm->ta = 0;
	svm->tb = 0;
	svm->to = 0;
}

void SVPWM_Update(SVPWM *svm, VHZPROFILE *vhz, TIM_HandleTypeDef *htim) {
//	ModIndex and Freq should ideally be the only things we will be changing
	svm->ModIndex = 0.2; /*vhz->Volt / vhz->VoltRated; how much time is spent in the off state */
	svm->Freq = 1; /*vhz->Freq;*/
	svm->StepAngle = (RADIANS * svm->Freq) / svm->SwitchFreq;
    svm->EntryOld = svm->NewEntry;
    svm->Alpha = svm->Alpha + svm->StepAngle;

	if (svm->Alpha >= PI_THIRD) {
		svm->Alpha = svm->Alpha-PI_THIRD;
	}

	svm->NewEntry = svm->Alpha;
	svm->ta = sinf(PI_THIRD - svm->NewEntry) * svm->ModIndex * svm->SwitchFreq;
	svm->tb = sinf(svm->NewEntry) * svm->ModIndex * svm->SwitchFreq;
	svm->to = (svm->SwitchFreq - svm->ta - svm->tb) / 2;

    if (svm->NewEntry - svm->EntryOld < 0) {
      	if (svm->SectorPointer == 5){
         	svm->SectorPointer = 0;
      	}
      	else {
         	svm->SectorPointer = svm->SectorPointer + 1;
      	}
    }


	if (svm->SectorPointer==0){
		svm->U = svm->ta + svm->tb + svm->to;
		svm->V = svm->tb + svm->to;
		svm->W = svm->to;
	}

	else if (svm->SectorPointer==1){
    	svm->U= svm->ta + svm->to;
    	svm->V = svm->ta + svm->tb + svm->to;
    	svm->W = svm->to;
    }

    else if (svm->SectorPointer==2){
    	svm->U = svm->to;
    	svm->V = svm->ta + svm->tb + svm->to;
    	svm->W = svm->tb + svm->to;
    }

    else if (svm->SectorPointer==3){
    	svm->U = svm->to;
    	svm->V = svm->ta + svm->to;
    	svm->W = svm->ta + svm->tb + svm->to;
    }

    else if (svm->SectorPointer==4){
    	svm->U = svm->tb + svm->to;
    	svm->V = svm->to;
    	svm->W = svm->ta + svm->tb + svm->to;
    }

    else if (svm->SectorPointer==5){
    	svm->U = svm->ta + svm->tb + svm->to;
    	svm->V = svm->to;
    	svm->W = svm->ta + svm->to;
    }

	TIM1->CCR1 = (svm->U/svm->SwitchFreq)*(TIM1->ARR+1);
	TIM1->CCR2 = (svm->V/svm->SwitchFreq)*(TIM1->ARR+1);
	TIM1->CCR3 = (svm->W/svm->SwitchFreq)*(TIM1->ARR+1);
}

