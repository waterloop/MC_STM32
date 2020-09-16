/*
 * SVPWM.h
 *
 *  Created on: Aug 6, 2020
 *      Author: Chawthri
 */

#ifndef INC_SVPWM_FOC_H_
#define INC_SVPWM_FOC_H_

typedef struct 	{ float  ModIndex; 				// Input: reference gain volUge (pu)
				  float  Freq;				// Input: reference frequency (pu)
                  float  SwitchFreq;
                  float  Alpha;     			// History: Sector angle (pu)
                  float  NewEntry;    		// History: Sine (angular) look-up pointer (pu)
                  uint32_t  SectorPointer;   	// History: Sector number (Q0) - independently with global Q
				  float  U;					// Output: reference phase-a swiWhing function (pu)
				  float  V;					// Output: reference phase-b swiWhing function (pu)
				  float  W;					// Output: reference phase-c swiWhing function (pu)
				  float  StepAngle;			// Variable
				  float  EntryOld;				// Variable
				  float  ta;					// Variable
				  float  tb;					// Variable
                  float  to;  			// Parameter: Maximum step angle = 6*base_freq*T (pu)

				} SVPWM;

#define SVPWM_DEFAULTS { 0,0,0,0,0,0,0,0,0,0,0,0,0 }

#define	PI_THIRD		1.04719755119660
#define RADIANS 		6.28318530718
#define SIN_PI_THIRD 	0.86602540378

#define SVPWM_MACRO(v)																			\
	/* Normalise the freq input to appropriate step angle	*/										\
    	/* Here, 1 pu. = 60 degree			*/														\
    v.StepAngle = (RADIANS*v.Freq)/v.SwitchFreq;														\
	/* Calculate new angle alpha			*/														\
    v.EntryOld = v.NewEntry;																	\
    v.Alpha = v.Alpha + v.StepAngle;															\
	if (v.Alpha >= PI_THIRD) {																		\
		v.Alpha = v.Alpha-PI_THIRD; 																\
		}																							\
		v.NewEntry = v.Alpha;																		\
		v.ta = sin(PI_THIRD-v.NewEntry)*v.ModIndex*v.SwitchFreq*SIN_PI_THIRD;     		\
	 	v.tb = sin(v.NewEntry)*v.ModIndex*v.SwitchFreq*SIN_PI_THIRD;              			\
	 	v.to = v.SwitchFreq - v.ta - v.tb;                                                                   \
  	/* Determine which sector	 		*/															\
    	if (v.NewEntry-v.EntryOld<0)																\
    	{																							\
      		if (v.SectorPointer==5)																	\
         		v.SectorPointer = 0;																\
      		else																					\
         		v.SectorPointer = v.SectorPointer + 1; 												\
    	}																							\
	 	if (v.SectorPointer==0)  /* Sector 1 calculations - a,b,c -. a,b,c*/							\
	    	{																							\
			v.U = (v.ta+v.tb)/2+v.ta;																\
			v.V = (v.ta+v.tb)/2+v.tb;																		\
			v.W = (v.ta+v.tb)/2;																\
	    	}																							\
	    else if (v.SectorPointer==1)  /* Sector 2 calculations - a,b,c -. b,a,c  &  v.ta <-. v.tb*/	\
	    	{																							\
	    	v.U = (v.ta+v.tb)/2+v.tb;																\
	    	v.V = (v.ta+v.tb)/2+v.ta;																		\
	    	v.W = (v.ta+v.tb)/2;																	\
	    	}																							\
	    else if (v.SectorPointer==2)  /* Sector 3 calculations - a,b,c -. b,c,a		*/				\
	    	{																							\
	    	v.U = (v.ta+v.tb)/2;																\
	    	v.V = (v.ta+v.tb)/2+v.ta;																		\
	    	v.W = (v.ta+v.tb)/2+v.tb;																\
	    	}																							\
	    else if (v.SectorPointer==3)  /* Sector 4 calculations - a,b,c -. c,b,a  &  v.ta <-. v.tb*/	\
	    	{																							\
	    	v.U = (v.ta+v.tb)/2;																\
	    	v.V = (v.ta+v.tb)/2+v.tb;																		\
	    	v.W = (v.ta+v.tb)/2+v.ta;	 																	\
	    	}																							\
	    else if (v.SectorPointer==4)  /* Sector 5 calculations - a,b,c -. c,a,b		*/				\
	    {																							\
	    	v.U = (v.ta+v.tb)/2+v.tb;																\
	    	v.V = (v.ta+v.tb)/2;																		\
	    	v.W = (v.ta+v.tb)/2+v.ta;	 																	\
	    }																							\
	    else if (v.SectorPointer==5)  /* Sector 6 calculations - a,b,c -. a,c,b  &  v.ta <-. v.tb*/	\
	    {																							\
	    	v.U = (v.ta+v.tb)/2+v.ta;																\
	    	v.V = (v.ta+v.tb)/2;																			\
	    	v.W = (v.ta+v.tb)/2+v.tb;	 																	\
	    }
#endif /* INC_SVPWM_FOC_H_ */
