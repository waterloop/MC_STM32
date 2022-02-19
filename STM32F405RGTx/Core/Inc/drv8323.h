#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// SIDENOTE: fuck this chip

#define FAULT_STATUS_1_ADDR         0x00
#define FAULT_STATUS_2_ADDR         0x01
#define DRIVER_CONTROL_ADDR         0x02
#define GATE_DRIVE_HS_ADDR          0x03
#define GATE_DRIVE_LS_ADDR          0x04
#define OCP_CONTROL_ADDR            0x05
#define CSA_CONTROL_ADDR            0x06

#define FAULT_STATUS_1_FAULT        10
#define FAULT_STATUS_1_VDS_OCP      9
#define FAULT_STATUS_1_GDF          8
#define FAULT_STATUS_1_UVLO         7
#define FAULT_STATUS_1_OTSD         6
#define FAULT_STATUS_1_VDS_HA       5
#define FAULT_STATUS_1_VDS_LA       4
#define FAULT_STATUS_1_VDS_HB       3
#define FAULT_STATUS_1_VDS_LB       2
#define FAULT_STATUS_1_VDS_HC       1
#define FAULT_STATUS_1_VDS_LC       0

#define FAULT_STATUS_2_SA_OC        10
#define FAULT_STATUS_2_SB_OC        9
#define FAULT_STATUS_2_SC_OC        8
#define FAULT_STATUS_2_OTW          7
#define FAULT_STATUS_2_CPUV         6
#define FAULT_STATUS_2_VGS_HA       5
#define FAULT_STATUS_2_VGS_LA       4
#define FAULT_STATUS_2_VGS_HB       3
#define FAULT_STATUS_2_VGS_LB       2
#define FAULT_STATUS_2_VGS_HC       1
#define FAULT_STATUS_2_VGS_LC       0

#define DRIVER_CONTROL_DIS_CPUV     9
#define DRIVER_CONTROL_DIS_GDF      8
#define DRIVER_CONTROL_OTW_REP      7
#define DRIVER_CONTROL_PWM_MODE     5
#define DRIVER_CONTROL_PWM_COM1     4
#define DRIVER_CONTROL_PWM_DIR1     3
#define DRIVER_CONTROL_COAST        2
#define DRIVER_CONTROL_BRAKE        1
#define DRIVER_CONTROL_CLR_FLT      0

#define GATE_DRIVE_HS_LOCK          8
#define GATE_DRIVE_HS_IDRIVEP_HS    4
#define GATE_DRIVE_HS_IDRIVEN_HS    0

#define GATE_DRIVE_LS_CBC           10
#define GATE_DRIVE_LS_TDRIVE        8
#define GATE_DRIVE_LS_IDRIVEP_LS    4
#define GATE_DRIVE_LS_IDRIVEN_LS    0

#define OCP_CONTROL_TRETRY          10
#define OCP_CONTROL_DEAD_TIME       8
#define OCP_CONTROL_OCP_MODE        6
#define OCP_CONTROL_OCP_DEG         4
#define OCP_CONTROL_VDS_LVL         0

#define CSA_CONTROL_CSA_FET         10
#define CSA_CONTROL_VREF_DIV        9
#define CSA_CONTROL_LS_REF          8
#define CSA_CONTROL_CSA_GAIN        6
#define CSA_CONTROL_DIS_SEN         5
#define CSA_CONTROL_CSA_CAL_A       4
#define CSA_CONTROL_CSA_CAL_B       3
#define CSA_CONTROL_CSA_CAL_C       2
#define CSA_CONTROL_SEN_LVL         0

typedef struct {
    uint16_t FAULT_STATUS_1;
    uint16_t FAULT_STATUS_2;
    uint16_t DRIVER_CONTROL;
    uint16_t GATE_DRIVE_HS;
    uint16_t GATE_DRIVE_LS;
    uint16_t OCP_CONTROL;
    uint16_t CSA_CONTROL;
} Drv8323;

Drv8323 Drv8323_init();

void Drv8323_read_reg(Drv8323* self, uint8_t addr);
void Drv8323_read_all(Drv8323* self);
void Drv8323_commit(Drv8323* self);

#ifdef __cplusplus
}
#endif
