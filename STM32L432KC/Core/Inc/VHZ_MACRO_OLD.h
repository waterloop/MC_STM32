/* =================================================================================
File name:       VHZ_PROF.H  
===================================================================================*/


#ifndef __VHZ_PROF_H__
#define __VHZ_PROF_H__

#include "math.h"

typedef struct 	{ float  Freq; 		    // Input: Input Frequency (pu)
				float  VoltOut;			// Output: Output voltage (pu)
				float  LowFreq;			// Parameter: Low Frequency (pu)
				float  HighFreq;			// Parameter: High Frequency at rated voltage (pu)
				float  FreqMax; 			// Parameter: Maximum Frequency (pu)
				float  VoltMax;			// Parameter: Rated voltage (pu)
			    float  VoltMin;	 		// Parameter: Voltage at low Frequency range (pu)
			    float  VfSlope;			// Variable
			    float  AbsFreq;			// Variable
		  	  	  
				} VHZPROFILE;


/*-----------------------------------------------------------------------------
Default initalizer for the VHZPROF object.
-----------------------------------------------------------------------------*/                     
#define VHZPROF_DEFAULTS { 0,0, 		\
                           0,0,0,0,0, 	\
                  		 }

/*------------------------------------------------------------------------------
	 VHZ_PROF Macro Definitions
------------------------------------------------------------------------------*/
void VHZ_Update(VHZPROFILE *vhz);

//#define VHZ_PROF_MACRO(v)															\
/* Take absolute frequency to allow the operation of both rotational directions	*/	\
    v.AbsFreq = fabsf(v.Freq);														\
	if (v.AbsFreq <= v.LowFreq)   													\
	        /* Compute output voltage in profile #1	*/								\
        	v.VoltOut = v.VoltMin;													\
	else if ((v.AbsFreq > v.LowFreq)&&(v.AbsFreq <= v.HighFreq))      				\
       {																			\
        	/* Compute slope of V/f profile	*/										\
        	v.VfSlope = (v.VoltMax - v.VoltMin)/(v.HighFreq - v.LowFreq);	\
        	/* Compute output voltage in profile #2	*/								\
        	v.VoltOut = v.VoltMin + v.VfSlope*(v.AbsFreq-v.LowFreq);		\
       }																			\
    else if ((v.AbsFreq > v.HighFreq)&&(v.AbsFreq < v.FreqMax))      				\
        	/* Compute output voltage in profile #3	*/								\
        	v.VoltOut = v.VoltMax;


#endif  // __VHZ_PROF_H__
