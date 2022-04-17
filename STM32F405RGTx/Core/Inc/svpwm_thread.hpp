#pragma once

#include "VHZ.h"
#include "main.h"
#include "util.hpp"

#define MODINDEX               0.9                  // Set ModIndex here        
#define FUNDAMENTAL_FREQ       1                    // Set Frequency here (kHz)
#define	PI_THIRD		       1.04719755119660f
#define RADIANS 		       6.28318530718f
#define SIN_PI_THIRD 	       0.86602540378f

class SVPWMThread{
    public:
        static void initialize();

        static VHZPROFILE vhz;
        static TIM_HandleTypeDef *htim;

        static void runPWM(void * args);
        void SVPWM_Update(SVPWMThread *svm, VHZPROFILE *vhz, TIM_HandleTypeDef *htim);
        static osThreadId_t getThreadId();

        static float  ModIndex;
		static float  FundamentalFreq;
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

    private:
        static RTOSThread thread;
};

extern SVPWMThread svpwm;
