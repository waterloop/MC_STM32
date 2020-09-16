/* =================================================================================
File name:       SVGEN_MF.H
===================================================================================*/

#ifndef __SVGEN_MF_H__
#define __SVGEN_MF_H__

typedef struct 	{ float  Gain; 				// Input: reference gain voltage (pu)
				  float  Offset;				// Input: reference offset voltage (pu)
				  float  Freq;				// Input: reference frequency (pu)
                  float  SwitchFreq;  			// Parameter: Maximum step angle = 6*base_freq*T (pu)
                  float  Alpha;     			// History: Sector angle (pu)
                  float  NewEntry;    		// History: Sine (angular) look-up pointer (pu)
                  uint32_t  SectorPointer;   	// History: Sector number (Q0) - independently with global Q
				  float  Ta;					// Output: reference phase-a switching function (pu)
				  float  Tb;					// Output: reference phase-b switching function (pu)
				  float  Tc;					// Output: reference phase-c switching function (pu)
				  float  StepAngle;			// Variable
				  float  EntryOld;				// Variable
				  float  dx;					// Variable
				  float  dy;					// Variable

				} SVGENMF;

/*-----------------------------------------------------------------------------
Default initalizer for the SVGENMF object.
-----------------------------------------------------------------------------*/
#define SVGENMF_DEFAULTS { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 }

/*------------------------------------------------------------------------------
	 SVGENMF Macro Definitions
------------------------------------------------------------------------------*/

#define	PI_THIRD	1.04719755119660    /* This is 60 degree */
#define PI          3.14159265358979

#define SVGENMF_MACRO(v)																			\
	/* Normalise the freq input to appropriate step angle	*/										\
    	/* Here, 1 pu. = 60 degree			*/														\
    	v.StepAngle = (2/(v.SwitchFreq/v.Freq))*PI;														\
	/* Calculate new angle alpha			*/														\
    	v.EntryOld = v.NewEntry;																	\
    	v.Alpha = v.Alpha + v.StepAngle;															\
	if (v.Alpha >= PI_THIRD) {																		\
		v.Alpha = v.Alpha-PI_THIRD; 																\
		}																							\
		v.NewEntry = v.Alpha;																		\
	 	v.dy = sin(v.NewEntry)*6.283185307;              			\
		v.dx = sin(PI_THIRD-v.NewEntry)*6.283185307;     		\
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
		v.Ta = (1.0-v.dx-v.dy)/2;																\
		v.Tb = v.Ta + v.dx;																			\
		v.Tc = 1.0 - v.Ta; 																	\
    	}																							\
    	else if (v.SectorPointer==1)  /* Sector 2 calculations - a,b,c -. b,a,c  &  v.dx <-. v.dy*/	\
    	{																							\
		v.Tb = (1.0-v.dx-v.dy)/2;																\
		v.Ta = v.Tb + v.dy;																			\
		v.Tc = 1.0 - v.Tb; 																	\
    	}																							\
    	else if (v.SectorPointer==2)  /* Sector 3 calculations - a,b,c -. b,c,a		*/				\
    	{																							\
		v.Tb = (1.0-v.dx-v.dy)/2;																\
		v.Tc = v.Tb + v.dx;																			\
	    	v.Ta = 1.0 - v.Tb; 																\
    	}																							\
    	else if (v.SectorPointer==3)  /* Sector 4 calculations - a,b,c -. c,b,a  &  v.dx <-. v.dy*/	\
    	{																							\
		v.Tc = (1.0-v.dx-v.dy)/2;																\
		v.Tb = v.Tc + v.dy;																			\
		v.Ta = 1.0 - v.Tc; 																	\
    	}																							\
    	else if (v.SectorPointer==4)  /* Sector 5 calculations - a,b,c -. c,a,b		*/				\
    	{																							\
		v.Tc = (1.0-v.dx-v.dy)/2;																\
		v.Ta = v.Tc + v.dx;																			\
		v.Tb = 1.0 - v.Tc; 																	\
    	}																							\
    	else if (v.SectorPointer==5)  /* Sector 6 calculations - a,b,c -. a,c,b  &  v.dx <-. v.dy*/	\
    	{																							\
		v.Ta = (1.0-v.dx-v.dy)/2;																\
		v.Tc = v.Ta + v.dy;																			\
		v.Tb = 1.0 - v.Ta; 																	\
    	}																							\
/* Convert the unsigned GLOBAL_Q format (ranged (0,1)) . signed GLOBAL_Q format (ranged (-1,1))	*/	\
/* Then, multiply with a gain and add an offset.						*/							\
    	v.Ta = (v.Ta-0.5)*2;																	\
    	v.Ta = v.Gain*v.Ta/6.283185307 + v.Offset;														\
																									\
 	    v.Tb = (v.Tb-0.5)*2;																		\
    	v.Tb = v.Gain*v.Tb/6.283185307 + v.Offset;														\
																									\
    	v.Tc = (v.Tc-0.5)*2; 																	\
    	v.Tc = v.Gain*v.Tc/6.283185307 + v.Offset;														\

#endif // __SVGEN_MF_H__
