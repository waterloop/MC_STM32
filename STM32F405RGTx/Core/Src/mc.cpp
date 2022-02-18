#include "main.h"
#include "drv8323.h"
#include "mc.hpp"

int mc_entry() {
    while (1) {
        asm("NOP");
    }

    return 0;
}