#pragma once

#include "cmsis_os.h"
#include "measurements_thread.hpp"
#include "state_machine_thread.hpp"
#include "CAN_thread.hpp"
#include "LED_thread.hpp"

// Thread Periodicities (ms)
#define MEASUREMENTS_THREAD_PERIODICITY     100
#define MEASUREMENTS_THREAD_PRIORITY        osPriorityRealtime7

#define STATE_MACHINE_THREAD_PERIODICITY    150
#define STATE_MACHINE_THREAD_PRIORITY       osPriorityRealtime6

#define CAN_THREAD_PERIODICITY              200
#define CAN_THREAD_PRIORITY                 osPriorityRealtime5

#define LED_THREAD_PERIODICITY              350
#define LED_THREAD_PRIORITY                 osPriorityIdle

