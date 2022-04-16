#pragma once

#include "util.hpp"

typedef struct {
    float R;
    float G;
    float B;
    float blink;
} LEDStatus;

class LEDThread {
    public:
        static void initialize();
        static void setLED(LEDStatus status);

    private:
        static RTOSThread thread_;
        static float R;
        static float G;
        static float B;
        static uint8_t blink;
        static uint8_t on;

    private:
        static void runLEDThread(void* args);
};

