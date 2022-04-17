#include "vhz_thread.hpp"
#include "math.h"

void VHZPROFILE::VHZ_Init(VHZPROFILE *vhz, float FreqMin, float FreqRated, float VoltMin, float VoltRated) {
	vhz->FreqMin = FreqMin;
	vhz->FreqRated = FreqRated;
	vhz->VoltMin = VoltMin;
	vhz->VoltRated = VoltRated;
	vhz->VfSlope = (VoltRated - VoltMin) / (FreqRated - FreqMin);
	vhz->Volt = 0;
	vhz->Freq = FreqMin;
}

void VHZPROFILE::VHZ_Update(VHZPROFILE *vhz){
	if (vhz->Freq <= vhz->FreqMin){
		vhz->Volt = vhz->VoltMin;
	}

	else if ((vhz->Freq > vhz->FreqMin) && (vhz->Freq <= vhz->FreqRated)){
		vhz->Volt = vhz->VoltMin + vhz->VfSlope * (vhz->Freq - vhz->FreqMin);
    }

	else if ((vhz->Freq > vhz->FreqRated)){
       	vhz->Volt = vhz->VoltRated;
    }
}
