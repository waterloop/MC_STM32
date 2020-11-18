/* =================================================================================
File name:       VHZ_PROF.H  
===================================================================================*/


#ifndef __VHZ_PROF_H__
#define __VHZ_PROF_H__


typedef struct 	{
		float  FreqMin;
		float  FreqRated;
		float  VoltMin;
		float  VoltRated;
		float  VfSlope;
		float  Volt;
		float  Freq;
} VHZPROFILE;



void VHZ_Update(VHZPROFILE *vhz);
void VHZ_Init(VHZPROFILE *vhz, float FreqMin, float FreqRated, float VoltMin, float VoltRated);

#endif  // __VHZ_PROF_H__
