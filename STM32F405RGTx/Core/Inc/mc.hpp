#pragma once

#include "drv8323.h"

extern Drv8323 drv8323;

class MC {
    public:
        // Phase voltages
        float pVa;
        float pVb;
        float pVc;
        
        // DC battery voltage
        float dc_voltage;
        
        // Phase currents
        float pIa;
        float pIb;
        float pIc;
        
        // MOSFET
        float fet_temps[3];
        // Note: dc_cap_temp dne for g5.5 but does for g6 logic board
        float dc_cap_temp;
        
        // Current position, speed, and acceleration
        float curr_pos;
        float curr_speed;
        float curr_accel;
};


int mc_entry();

extern MC g_mc_data;
