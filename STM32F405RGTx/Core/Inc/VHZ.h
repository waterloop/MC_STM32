/* =================================================================================
File name:       VHZ_PROF.H  
===================================================================================*/

#pragma once


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
