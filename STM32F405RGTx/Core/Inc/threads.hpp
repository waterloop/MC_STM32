#pragma once

#include "cmsis_os.h"
#include "measurements_thread.hpp"
#include "state_machine_thread.hpp"
#include "led_thread.hpp"

// Thread Periodicities (ms)
#define MEASUREMENTS_THREAD_PERIODICITY      10
#define STATE_MACHINE_THREAD_PERIODICITY     20
#define LED_THREAD_PERIODICITY               200

