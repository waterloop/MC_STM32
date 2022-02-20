#include "main.h"
#include "drv8323.h"

Drv8323_Status _Drv8323_TransmitRecieve(
    SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {

    Drv8323_cs_low();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, Size, Timeout);
    Drv8323_cs_high();

    if (status != HAL_OK) { return DRV8323_SPI_ERR; }
    else { return DRV8323_OK; }
}
Drv8323_Status _Drv8323_Transmit(
    SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {

    Drv8323_cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, pData, Size, Timeout);
    Drv8323_cs_high();

    if (status != HAL_OK) { return DRV8323_SPI_ERR; }
    else { return DRV8323_OK; }
}

Drv8323 Drv8323_init() {
    Drv8323 ret = {};

    Drv8323_cs_high();
    Drv8323_enable();

    Drv8323_read_all(&ret);

    return ret;
}

Drv8323_Status Drv8323_commit(Drv8323* self) {
    uint16_t* reg_table[7] = {
        &(self->DRIVER_CONTROL),
        &(self->GATE_DRIVE_HS),
        &(self->GATE_DRIVE_LS),
        &(self->OCP_CONTROL),
        &(self->CSA_CONTROL)
    };

    Drv8323_Status status = DRV8323_OK;
    uint16_t tx = 0;
    for (uint8_t i = 2; i < 7; i++) {
        tx = (i << 11) | (*reg_table[i] & 0x7FF);
        status |= _Drv8323_Transmit(&hspi1, (uint8_t*)(&tx), 2, HAL_MAX_DELAY);
    }

    return status;
}

Drv8323_Status Drv8323_read_reg(Drv8323* self, uint8_t addr) {
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
    if (reg == NULL) { return DRV8323_INVALID_ARG_ERR; }

    uint16_t tx = (1U << 15) | (addr << 11);
    Drv8323_Status status = _Drv8323_TransmitRecieve(
        &hspi1, (uint8_t*)(&tx), (uint8_t*)reg, 2, HAL_MAX_DELAY);

    *reg &= 0x7ff;   // mask of 11 1s

    return status;
}

Drv8323_Status Drv8323_read_all(Drv8323* self) {
    uint16_t* reg_table[7] = {
        &(self->FAULT_STATUS_1),
        &(self->FAULT_STATUS_2),
        &(self->DRIVER_CONTROL),
        &(self->GATE_DRIVE_HS),
        &(self->GATE_DRIVE_LS),
        &(self->OCP_CONTROL),
        &(self->CSA_CONTROL)
    };

    Drv8323_Status status = DRV8323_OK;
    uint16_t tx = 0;
    for (uint8_t i = 0; i < 7; i++) {
        tx = (1 << 15) | (i << 11);
        status |= _Drv8323_TransmitRecieve(
            &hspi1, (uint8_t*)(&tx), (uint8_t*)reg_table[i], 2, HAL_MAX_DELAY);

        *reg_table[i] &= 0x7ff;     // mask of 11 1s
    }

    return status;
}
