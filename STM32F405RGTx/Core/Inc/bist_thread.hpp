#pragma once

#include "util.hpp"

// milli-seconds
#define BUTTON_DEBOUNCE_TIME        500

class BistThread {
    public:
        static void initialize();
        static void toggleBist();

    protected:
        static RTOSThread thread_;
        static void runBist(void* args);

        static void (*callback)(void);
        static void enabled_callback();
        static void disabled_callback();

        /*
            prompt  -> string to prompt user for input
            buff    -> buffer to receive input
            len     -> length of buffer, this value will be changed to
                       the length of the user inputted string on return
        */
        static void _sinput(const char* prompt, uint8_t* buff, uint32_t* len);
        static void _print(uint8_t* str);

        // wrapper for standard library strcmp so we can pass a uint8_t
        static uint8_t _strcmp(uint8_t* a, const char* b);

        // help
        static void _help();
        // print measurements
        static void _p_measurements();

        // change RGB LED color
        static void _rgb();

        // toggle fault checking in state machine
        static void _toggle_fc();

        // clears the command interface
        static void _clear();
};

