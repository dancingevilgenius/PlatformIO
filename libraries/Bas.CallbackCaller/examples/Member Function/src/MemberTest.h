#ifndef TEST_h
#define TEST_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <CallbackCaller.h>

class MemberTest
{
    Bas::CallbackCaller callbackCaller;
    int value;  // We'll initialize each MemberTest instance with a unique number to demonstrate that the callback is called on separate instances.

    public:
    void memberCallback();  // The callback to be called
    void begin(int value);  // begin() initializes the MemberTest instance
    void raiseCallback();   // raise the callback
};

#endif