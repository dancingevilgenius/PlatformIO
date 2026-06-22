#ifndef CALLBACKCALLER_h
#define CALLBACKCALLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

namespace Bas
{
    class CallbackCaller
    {
        struct BaseCallback
        {
            virtual ~BaseCallback() {}
            virtual void invoke() = 0;
        };

        template<typename Function>
        struct Callback : BaseCallback
        {
            Function callbackFunction;

            Callback(Function callbackFunction) : callbackFunction(callbackFunction)
            {
            }

            void invoke() override
            {
                callbackFunction();
            }
        };

        BaseCallback *callback = nullptr;

        public:
        CallbackCaller() {}
        CallbackCaller(const CallbackCaller&) = delete;
        CallbackCaller& operator=(const CallbackCaller&) = delete;
        ~CallbackCaller();

        template<typename Function>
        void begin(Function callbackFunction)
        {
			delete callback; // Just to be sure, in case begin() is called multiple times.
            callback = new Callback<Function>{ callbackFunction };
        }
        void call();
    };
}

#endif