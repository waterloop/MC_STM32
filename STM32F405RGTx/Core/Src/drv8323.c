#include "main.h"
#include "drv8323.h"

Drv8323_Status _Drv8323_TransmitRecieve(uint16_t* tx, uint16_t* rx) {
    uint8_t rx_buff[2];
    uint8_t tx_buff[2];
    tx_buff[0] = (*tx >> 8) & 0xff;
    tx_buff[1] = *tx & 0xff;

    Drv8323_cs_low();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_buff, rx_buff, 2, HAL_MAX_DELAY);
    Drv8323_cs_high();

    *rx = (rx_buff[0] << 8) | rx_buff[1];
    *rx &= 0x7FF;

    if (status != HAL_OK) { return DRV8323_SPI_ERR; }
    else { return DRV8323_OK; }
}
Drv8323_Status _Drv8323_Transmit(uint16_t* tx) {
    uint8_t tx_buff[2];
    tx_buff[0] = (*tx_buff >> 8) & 0xff;
    tx_buff[1] = *tx_buff & 0xff;

    Drv8323_cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx_buff, 2, HAL_MAX_DELAY);
    Drv8323_cs_high();

    if (status != HAL_OK) { return DRV8323_SPI_ERR; }
    else { return DRV8323_OK; }
}

Drv8323 Drv8323_init() {
    Drv8323 ret = {};

    Drv8323_cs_high();
    Drv8323_enable();

    Drv8323_read_all(&ret);

    // set the gate source and sink currents to the lowest setting...
    ret.GATE_DRIVE_HS &= ~(0b1111U << GATE_DRIVE_HS_IDRIVEP_HS);
    ret.GATE_DRIVE_HS &= ~(0b1111U << GATE_DRIVE_HS_IDRIVEN_HS);
    ret.GATE_DRIVE_LS &= ~(0b1111U << GATE_DRIVE_HS_IDRIVEP_HS);
    ret.GATE_DRIVE_LS &= ~(0b1111U << GATE_DRIVE_HS_IDRIVEN_HS);

    // set deadtime to max...
    ret.OCP_CONTROL |= (0b11U << OCP_CONTROL_DEAD_TIME);

    Drv8323_commit(&ret);

    return ret;
}

Drv8323_Status Drv8323_fault_status(Drv8323* self, uint8_t* has_fault) {
    if (HAL_GPIO_ReadPin(NFAULT_GPIO_Port, NFAULT_Pin) != 0) {
        *has_fault = 0U;
        return DRV8323_OK;
    }
    else {
        Drv8323_Status status = DRV8323_INTERNAL_FAULT;
        status |= Drv8323_read_all(self);
        *has_fault = 1U;
        return status;
    }
}

Drv8323_Status Drv8323_commit(Drv8323* self) {
    uint16_t* reg_table[7] = {
        NULL,
        NULL,
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
        status |= _Drv8323_Transmit(&tx);
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
    Drv8323_Status status = _Drv8323_TransmitRecieve(&tx, reg);

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
        status |= _Drv8323_TransmitRecieve(&tx, reg_table[i]);
    }

    return status;
}
