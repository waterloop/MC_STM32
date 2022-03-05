#pragma once

#include "VHZ.h"
#include "main.h"
#include "util.hpp"

#ifndef INC_SVPWM_H_
#define INC_SVPWM_H_

class SVPWM{
    public:
        static float  ModIndex;
		static float  Freq;
        static float  SwitchFreq;
        static float  Alpha;
        static float  NewEntry;
        static float  SectorPointer;
		static float  U;
        static float  V;
		static float  W;
		static float  StepAngle;
		static float  EntryOld;
		static float  ta;
		static float  tb;
        static float  to;

        static VHZPROFILE *vhz;
        static TIM_HandleTypeDef *htim;

        SVPWM(float SwitchFreq, VHZPROFILE *vhz, TIM_HandleTypeDef *htim);
        static void initialize();
        static void update(void* arg);
    private:
        static RTOSThread thread;
};


#define	PI_THIRD		1.04719755119660f
#define RADIANS 		6.28318530718f
#define SIN_PI_THIRD 	0.86602540378f

#endif