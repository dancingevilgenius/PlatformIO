#include "CallbackCaller.h"

Bas::CallbackCaller::~CallbackCaller()
{
    delete callback;
}

void Bas::CallbackCaller::call()
{
    if (callback)
    {
        callback->invoke();
    }
}
