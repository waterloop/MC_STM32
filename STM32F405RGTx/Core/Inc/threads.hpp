#pragma once

#include "cmsis_os.h"
#include "measurements_thread.hpp"
#include "state_machine_thread.hpp"
#include "CAN_thread.hpp"
#include "LED_thread.hpp"

// Thread Periodicities (ms)
#define MEASUREMENTS_THREAD_PERIODICITY     10
#define MEASUREMENTS_THREAD_PRIORITY        osPriorityAboveNormal

#define STATE_MACHINE_THREAD_PERIODICITY    20
#define STATE_MACHINE_THREAD_PRIORITY       osPriorityNormal

#define CAN_THREAD_PERIODICITY              200
#define CAN_THREAD_PRIORITY                 osPriorityBelowNormal

#define LED_THREAD_PERIODICITY              350
#define LED_THREAD_PRIORITY                 osPriorityIdle

