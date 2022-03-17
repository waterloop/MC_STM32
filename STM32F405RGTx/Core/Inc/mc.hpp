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
        
        // MOSFET and DC Link Capacitor temperatures
        float fet_temps[3];
        // Note: dc_cap_temp dne for Powerboard rev 2 but will for the next rev
        float dc_cap_temp;
        
        // Current position, speed, and acceleration
        float curr_pos;
        float curr_speed;
        float curr_accel;
};


int mc_entry();

extern MC g_mc_data;
<<<<<<< HEAD
=======

>>>>>>> 04bfb9fc38670c61e96b283967aee63f0705ca84
