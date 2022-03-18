#pragma once
#include "util.hpp"

class VHzThread {
    public:
        static void initialize();
        static void VHzUpdate(void* args);

    private:
        static RTOSThread thread;

};