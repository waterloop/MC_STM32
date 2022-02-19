#include "main.h"
#include "drv8323.h"

Drv8323 Drv8323_init() {
    Drv8323 ret = {};
    Drv8323_read_all(&ret);

    return ret;
}

void Drv8323_read_reg(Drv8323* self, uint8_t addr) {
    uint16_t* reg;
    switch (addr) {
        case FAULT_STATUS_1_ADDR:
            reg = &(self->FAULT_STATUS_1);

        case FAULT_STATUS_2_ADDR:
            reg = &(self->FAULT_STATUS_2);

        case DRIVER_CONTROL_ADDR:
            reg = &(self->OCP_CONTROL);

        case GATE_DRIVE_HS_ADDR:
            reg = &(self->GATE_DRIVE_HS);

        case GATE_DRIVE_LS_ADDR:
            reg = &(self->GATE_DRIVE_LS);

        case OCP_CONTROL_ADDR:
            reg = &(self->OCP_CONTROL);

        case CSA_CONTROL_ADDR:
            reg = &(self->CSA_CONTROL);

        default:
            reg = NULL;
    }
    if (reg == NULL) { return; }

    uint16_t tx = (1U << 15) | (addr << 11);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)(&tx), (uint8_t*)reg, 2, HAL_MAX_DELAY);

    *reg &= 0x7ff;   // mask of 11 1s
}

void Drv8323_commit(Drv8323* self) {
    uint16_t* reg_table[7] = {
        &(self->FAULT_STATUS_1),
        &(self->FAULT_STATUS_2),
        &(self->DRIVER_CONTROL),
        &(self->GATE_DRIVE_HS),
        &(self->GATE_DRIVE_LS),
        &(self->OCP_CONTROL),
        &(self->CSA_CONTROL)
    };

    uint16_t tx = 0;
    for (uint8_t i = 0; i < 7; i++) {
        tx = (i << 11) | (*reg_table[i] & 0x7FF);
        HAL_SPI_Transmit(
            &hspi1, (uint8_t*)(tx), 2, HAL_MAX_DELAY);
    }
}

void Drv8323_read_all(Drv8323* self) {
    uint16_t* reg_table[7] = {
        &(self->FAULT_STATUS_1),
        &(self->FAULT_STATUS_2),
        &(self->DRIVER_CONTROL),
        &(self->GATE_DRIVE_HS),
        &(self->GATE_DRIVE_LS),
        &(self->OCP_CONTROL),
        &(self->CSA_CONTROL)
    };

    uint16_t tx = 0;
    for (uint8_t i = 0; i < 7; i++) {
        tx = (1 << 15) | (i << 11);
        HAL_SPI_TransmitReceive(
            &hspi1, (uint8_t*)(&tx), (uint8_t*)reg_table[i], 2, HAL_MAX_DELAY);

        *reg_table[i] &= 0x7ff;     // mask of 11 1s
    }
}

