#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "threads.hpp"

SVPWMThread svpwm;
int OldSector;
RTOSThread SVPWMThread::thread;

void SVPWMThread::initialize(){
   	thread = RTOSThread(
        "svpwm_thread",
        1024,
        osPriorityAboveNormal,
		runPWM
    );
}

void SVPWMThread::runPWM(void* arg) {

	while(1) {
		osThreadFlagsWait(0x00000001U, osFlagsWaitAll, 0U);
		OldSector = svpwm.SectorPointer;
		VHZ_Update(&vhz);
		svpwm.SVPWM_Update(&svpwm, &vhz, &htim1);
	}

}

void SVPWMThread::SVPWM_Update(SVPWMThread *svm, VHZPROFILE *vhz, TIM_HandleTypeDef *htim){

	svpwm.ModIndex = MODINDEX; /*vhz->Volt / vhz->VoltRated;*/
	svpwm.FundamentalFreq = FUNDAMENTAL_FREQ; /*vhz->Freq;*/
	svpwm.StepAngle = (RADIANS * svpwm.FundamentalFreq) / svpwm.SwitchFreq;
	svpwm.EntryOld = svpwm.NewEntry;
	svpwm.Alpha = svpwm.Alpha + svpwm.StepAngle;

	if (svpwm.Alpha >= PI_THIRD) {
		svpwm.Alpha = svpwm.Alpha-PI_THIRD;
	}

	svpwm.NewEntry = svpwm.Alpha;
	svpwm.ta = sinf(PI_THIRD - svpwm.NewEntry) * svpwm.ModIndex * svpwm.SwitchFreq;
	svpwm.tb = sinf(svpwm.NewEntry) * svpwm.ModIndex * svpwm.SwitchFreq;
	svpwm.to = (svpwm.SwitchFreq - svpwm.ta - svpwm.tb) / 2;

    if (svpwm.NewEntry - svpwm.EntryOld < 0) {
      	if (svpwm.SectorPointer == 5){
      		svpwm.SectorPointer = 0;
      	}
      	else {
      		svpwm.SectorPointer = svpwm.SectorPointer + 1;
      	}
    }

	if (svpwm.SectorPointer==0){
		svpwm.U = svpwm.ta + svpwm.tb + svpwm.to;
		svpwm.V = svpwm.tb + svpwm.to;
		svpwm.W = svpwm.to;
	}

	else if (svpwm.SectorPointer==1){
		svpwm.U = svpwm.ta + svpwm.to;
		svpwm.V = svpwm.ta + svpwm.tb + svpwm.to;
		svpwm.W = svpwm.to;
    }

    else if (svpwm.SectorPointer==2){
    	svpwm.U = svpwm.to;
    	svpwm.V = svpwm.ta + svpwm.tb + svpwm.to;
    	svpwm.W = svpwm.tb + svpwm.to;
    }

    else if (svpwm.SectorPointer==3){
    	svpwm.U = svpwm.to;
    	svpwm.V = svpwm.ta + svpwm.to;
    	svpwm.W = svpwm.ta + svpwm.tb + svpwm.to;
    }

    else if (svpwm.SectorPointer==4){
    	svpwm.U = svpwm.tb + svpwm.to;
    	svpwm.V = svpwm.to;
    	svpwm.W = svpwm.ta + svpwm.tb + svpwm.to;
    }

    else if (svpwm.SectorPointer==5){
    	svpwm.U = svpwm.ta + svpwm.tb + svpwm.to;
    	svpwm.V = svpwm.to;
    	svpwm.W = svpwm.ta + svpwm.to;
    }

	TIM1->CCR1 = (svpwm.U/svpwm.SwitchFreq)*(TIM1->ARR+1);
	TIM1->CCR2 = (svpwm.V/svpwm.SwitchFreq)*(TIM1->ARR+1);
	TIM1->CCR3 = (svpwm.W/svpwm.SwitchFreq)*(TIM1->ARR+1);
}

osThreadId_t SVPWMThread::getThreadId() {
    return thread.getThreadId();
} 
