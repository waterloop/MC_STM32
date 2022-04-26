#pragma once

#include "main.h"
#include "util.hpp"


class VHZPROFILE{
	public:
		float  FreqMin;
		float  FreqRated;
		float  VoltMin;
		float  VoltRated;
		float  VfSlope;
		float  Volt;
		float  Freq;	

		void VHZ_Update(VHZPROFILE *vhz);
		void VHZ_Init(VHZPROFILE *vhz, float FreqMin, float FreqRated, float VoltMin, float VoltRated);
};

