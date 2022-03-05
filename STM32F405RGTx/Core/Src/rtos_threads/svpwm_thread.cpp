#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "threads.hpp"
#include "math.h"

SVPWM::SVPWM(float SwitchFreq, VHZPROFILE *vhz, TIM_HandleTypeDef *htim){
	ModIndex = 0;
	Freq = 0;
	SwitchFreq = SwitchFreq;
	Alpha = 0;
	NewEntry = 0;
	SectorPointer = 0;
	U = 0;
	V = 0;
	W = 0;
	StepAngle = 0;
	EntryOld = 0;
	ta = 0;
	tb = 0;
	to = 0;

	vhz = vhz;
	htim = htim;
}

void SVPWM::initialize(){
	SVPWM sv(10000, vhz, htim); 

   	thread = RTOSThread(
        "svpwm_thread",
        1024,
        osPriorityAboveNormal,
		update // what is going wrong?
    );
}

void SVPWM::update(void* arg){
    ModIndex = 0.9; /*vhz->Volt / vhz->VoltRated;*/
	Freq = 1; /*vhz->Freq;*/
	StepAngle = (RADIANS * Freq) / SwitchFreq;
    EntryOld = NewEntry;
    Alpha = Alpha + StepAngle;

	if (Alpha >= PI_THIRD) {
		Alpha = Alpha-PI_THIRD;
	}

	NewEntry = Alpha;
	ta = sinf(PI_THIRD - NewEntry) * ModIndex * SwitchFreq;
	tb = sinf(NewEntry) * ModIndex * SwitchFreq;
	to = (SwitchFreq - ta - tb) / 2;

    if (NewEntry - EntryOld < 0) {
      	if (SectorPointer == 5){
         	SectorPointer = 0;
      	}
      	else {
         	SectorPointer = SectorPointer + 1;
      	}
    }

	if (SectorPointer==0){
		U = ta + tb + to;
		V = tb + to;
		W = to;
	}

	else if (SectorPointer==1){
    	U= ta + to;
    	V = ta + tb + to;
    	W = to;
    }

    else if (SectorPointer==2){
    	U = to;
    	V = ta + tb + to;
    	W = tb + to;
    }

    else if (SectorPointer==3){
    	U = to;
    	V = ta + to;
    	W = ta + tb + to;
    }

    else if (SectorPointer==4){
    	U = tb + to;
    	V = to;
    	W = ta + tb + to;
    }

    else if (SectorPointer==5){
    	U = ta + tb + to;
    	V = to;
    	W = ta + to;
    }

	TIM1->CCR1 = (U/SwitchFreq)*(TIM1->ARR+1);
	TIM1->CCR2 = (V/SwitchFreq)*(TIM1->ARR+1);
	TIM1->CCR3 = (W/SwitchFreq)*(TIM1->ARR+1);
}