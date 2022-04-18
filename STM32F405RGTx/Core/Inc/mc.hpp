#pragma once

#include "drv8323.h"

extern Drv8323 drv8323;

class MC {
    public:
        // Phase voltages
        float pVa;
        float pVb;
        float pVc;

        // Phase currents
        float pIa;
        float pIb;
        float pIc;
 
        // DC battery voltage
        float dc_voltage;
       
        // MOSFET and DC Link Capacitor temperatures
        float fet_temps[3];
        
        // Current position, speed, and acceleration
        float curr_pos;
        float curr_speed;
        float curr_accel;

        float target_speed;
        float track_length;

        float ring_encoder_reported_speed;

    private:
        // Note: dc_cap_temp dne for Powerboard rev 2 but will for the next rev
        float dc_cap_temp;
};


int mc_entry();

extern MC g_mc_data;
