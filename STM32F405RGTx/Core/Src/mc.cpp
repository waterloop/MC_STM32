#include "main.h"
#include "drv8323.h"
#include "mc.hpp"

Drv8323 drv8323;

int mc_entry() {
    drv8323 = Drv8323_init();

    while (1) {
        asm("NOP");
    }

    return 0;
}
