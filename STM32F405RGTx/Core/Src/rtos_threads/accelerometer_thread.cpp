#include "main.h"
#include "threads.hpp"

void AccelerometerThread::initialize(){
	accelerometer = Accelerometer(I2CConnection);
   	thread = RTOSThread(
        "accelerometer_thread",
        1024*5,
		ACCELEROMETER_THREAD_PRIORITY,
		update
    );
}

void AccelerometerThread::update(void* args) {
    while (1) {
    	accelerometer.update();
        osThreadFlagsWait(0x00000001U, osFlagsWaitAll, 0U);
        osDelay(MEASUREMENTS_THREAD_PERIODICITY);
    }
}

void AccelerometerThread::stopMeasurements() {
    osThreadSuspend(thread.getThreadId());
}

void AccelerometerThread::resumeMeasurements() {
    osThreadResume(thread.getThreadId());
}

osThreadId_t AccelerometerThread::getThreadId() {
    return thread.getThreadId();
}
