#pragma once

#include "vhz_thread.hpp"
#include "main.h"
#include "util.hpp"

#define MODINDEX               0.9                  // Set ModIndex here        
#define FUNDAMENTAL_FREQ       1                    // Set Frequency here (kHz)
#define	PI_THIRD		       1.04719755119660f
#define RADIANS 		       6.28318530718f
#define SIN_PI_THIRD 	       0.86602540378f

class SVPWM{
    public:
        float  ModIndex;
		 float  FundamentalFreq;
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

