#include "can_thread.hpp"

#define BUS_TEST_FLAG       0b0010U
#define STATE_CHANGE_FLAG   0b0001U

CANFrame* CANThread::currentRxFrame;

void CANThread::initialize() {
    thread = RTOSThread(
        "can_thread",
        1024,
        osPriorityHigh,
        runCANThread
    );
    osThreadFlagsSet(thread.getThreadId(), STATE_CHANGE_FLAG);

    // stateIdChangeFlag = -1;
}

uint8_t CANThread::hasStateReq() {
    return stateChangeReq > 0;
}

StateID CANThread::getStateChange() {
    osThreadFlagsSet(thread.getThreadId(), STATE_CHANGE_FLAG);
    return (StateID)(stateChangeReq - 1);
} 

void CANThread::runCANThread(void *args) {
    while (1) {
        // Load next queue message if our current message has been dealt with
        if (!Queue_empty(&RX_QUEUE) && !currentRxFrame) {
            CANFrame returnFrame = CANBus_get_frame();
            currentRxFrame = &returnFrame;
        }

        if (currentRxFrame) {
            // If we have a message, let's handle it
            if (currentRxFrame->id == STATE_CHANGE_REQ) {
                osThreadFlagsWait(STATE_CHANGE_FLAG, osFlagsWaitAll, osWaitForever);
                stateChangeReq = (StateID)(CANFrame_get_field(currentRxFrame, STATE_ID) + 1);
                currentRxFrame = NULL;
            }  
            // else if (currentRxFrame->id >= BUS_TEST_REQ_BASE && currentRxFrame->id <= BUS_TEST_REQ_BASE | 0xF && busTestReqFlag < 0) {
            //     busTestReqFlag = currentRxFrame->id - BUS_TEST_REQ_BASE;
            //     currentRxFrame = NULL;
            // }
        }

        osDelay(20);
    }
}