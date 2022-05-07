#pragma once

#include "vhz_thread.hpp"
#include "main.h"
#include "util.hpp"

#define MODINDEX               0.5                  // Set ModIndex here        
#define FUNDAMENTAL_FREQ       1000                    // Set Frequency here (kHz)]
#define SWITCH_FREQ            10000
#define	PI_THIRD		       1.04719755119660f
#define RADIANS 		       6.28318530718f
#define SIN_PI_THIRD 	       0.86602540378f

class SVPWM{
    public:
        float  ModIndex = 0;
		 float  FundamentalFreq = 0;
         float  SwitchFreq = SWITCH_FREQ;
         float  Alpha = 0;
         float  NewEntry = 0;
         float  SectorPointer = 0;
		 float  U = 0;
         float  V = 0;
		 float  W = 0;
		 float  StepAngle = 0;
		 float  EntryOld = 0;
		 float  ta = 0;
		 float  tb = 0;
         float  to = 0;

};

class SVPWMThread{
    public:
        static void initialize();

        static VHZPROFILE vhz;
        static TIM_HandleTypeDef *htim;
        static SVPWM svpwm;

        static void runPWM(void * args);
        static void SVPWM_Update(SVPWM *svm, VHZPROFILE *vhz, TIM_HandleTypeDef *htim);
        static osThreadId_t getThreadId();

    private:
        static RTOSThread thread;
};

