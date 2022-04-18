#pragma once

#include "util.hpp"

#define BUS_TEST_REQ_TIMEOUT        10000

extern osMessageQueueId_t g_state_change_req_queue;
extern osMutexId_t g_bus_mutex;

// thread safe wrapper for CANBus_put_frame
void send_frame(CANFrame* frame);

class CANThread {
    public:
        static void initialize();
        static osThreadId_t getThreadId();

    private:
        static RTOSThread thread_;
        static void runCANThread(void* args);

        static void send_heartbeat();
};


