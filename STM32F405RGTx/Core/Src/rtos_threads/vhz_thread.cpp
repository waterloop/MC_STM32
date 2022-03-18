#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "vhz_thread.hpp"
#include "mc.hpp"
#include "cmsis_os.h"
#include "threads.hpp"

RTOSThread VHzThread::thread;

void VHzThread::initialize(){
    thread = RTOSThread(
        "vhz_thread",
        1024,
        osPriorityAboveNormal,
        VHzUpdate
    );

}

void VHzThread::VHzUpdate(void* args){
    int i = 0;
}