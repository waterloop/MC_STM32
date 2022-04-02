#include "can.h"
#include "util.hpp"

/*
Checking if a message has been sent in the RX_QUEUE

If it has been received, it will pop it from the queue and store 
    the message data as appropriate (E.g. if the message received 
    is a state change request, it should store the requested state 
    inside of a field of the CAN thread class)

If a message is received for a field that already has a value (E.g. 
    if a state change request message is received while there is already 
    a requested state change stored in the class field), it will wait 
    until a rtos thread flag is set (Thread Flags (keil.com))

It will have getters for these fields. If a getter for a field is called, 
    it will clear the field, return its value and set the corresponding 
    rtos thread flag
*/

/*
TODO: 
    - Store other types of message data (only state change requests have been done so far)
    - Replace all interaction with the CAN library in the MC repo with the CAN thread
*/

class CANThread {
    public:
        static void initialize();

        static uint8_t hasStateReq();

        static StateID getStateChange();

    private:
        static RTOSThread thread;
        static void runCANThread(void *args);

        static CANFrame* currentRxFrame;

        // Requested state change
        static StateID stateChangeReq;
};