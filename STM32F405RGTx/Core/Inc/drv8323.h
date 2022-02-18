#pragma once

#include <stdint.h>

// SIDENOTE: fuck this chip

typedef enum {
    FAULT       = 0b1U << 10,       // Logical OR of FAULT status registers, NOT of nFAULT pin
    VDS_OCP     = 0b1U << 9,        // VDS monitor overcurrent fault
    GDF         = 0b1U << 8,        // Gate drive fault
    UVLO        = 0b1U << 7,        // Undervoltage lockout fault
    OTSD        = 0b1U << 6,        // Overtemperature shutdown
    VDS_HA      = 0b1U << 5,        // VDS overcurrent fault on A high-side FET
    VDS_LA      = 0b1U << 4,        // VDS overcurrent fault on A low-side FET
    VDS_HB      = 0b1U << 3,        // VDS overcurrent fault on B high-side FET
    VDS_LB      = 0b1U << 2,        // VDS overcurrent fault on B low-side FET
    VDS_HC      = 0b1U << 1,        // VDS overcurrent fault on C high-side FET
    VDS_LC      = 0b1U << 0         // VDS overcurrent fault on C low-side FET
} FAULT_STATUS_1_FIELD;

typedef enum {
    SA_OC       = 0b1U << 10,       // Overcurrent on phase A differential amplifier
    SB_OC       = 0b1U << 9,        // Overcurrent on phase B differential amplifier
    SC_OC       = 0b1U << 8,        // Overcurrent on phase C differential amplifier
    OTW         = 0b1U << 7,        // Overtemperature warning
    CPUV        = 0b1U << 6,        // Charge pump undervoltage
    VGS_HA      = 0b1U << 5,        // Gate drive fault on A high-side FET
    VGS_LA      = 0b1U << 4,        // Gate drive fault on A low-side FET
    VGS_HB      = 0b1U << 3,        // Gate drive fault on B high-side FET
    VGS_LB      = 0b1U << 2,        // Gate drive fault on B low-side FET
    VGS_HC      = 0b1U << 1,        // Gate drive fault on C high-side FET
    VGS_LC      = 0b1U << 0         // Gate drive fault on C low-side FET
} FAULT_STATUS_2_FIELD;

typedef enum {
    DIS_CPUV    = 0b1U << 9,        // 0b1 => charge pump UVLO disabled
    DIS_GDF     = 0b1U << 8,        // 0b1 => gate drive fault disabled
    OTW_REP     = 0b1U << 7,        // 0b1 => OTW reported on nFAULT
    PWM_MODE    = 0b11U << 5,       // 0b00 => 6x, 0b01 => 3x, 0b10 => 1x, 0b11 => independent
    PWM_COM1    = 0b1U << 4,        // 0b0 => 1x mode uses synchronous recification
    PWM_DIR1    = 0b1U << 3,        // ORed with INHC (DIR) input in 1x PWM mode
    COAST       = 0b1U << 2,        // Write 1 to put all FETs in Hi-Z
    BRAKE       = 0b1U << 1,        // Write 1 to turn on all low-side FETs in 1x mode, ORed with INLC (BRAKE) input
    CLR_FLT     = 0b1U << 0,        // Write 1 to clear latched fault bits, resets after being written
} DRIVER_CONTROL_FIELD;

typedef enum {
    LOCK        = 0b111U << 8,      // 0b110 => lock settings and ignore further writes, 0b011 => unlock
    IDRIVEP_HS  = 
} GATE_DRIVE_HS_FIELD;


typedef struct {
    uint16_t FAULT_STATUS_1;
    uint16_t FAULT_STATUS_2;
    uint16_t DRIVER_CONTROL;
    uint16_t GATE_DRIVE_HS;
    uint16_t GATE_DRIVE_LS;
    uint16_t OCP_CONTROL;
    uint16_t CSA_CONTROL;
} Drv8323;
