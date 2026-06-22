#include <CallbackCaller.h>

void setup()
{
    Serial.begin(9600);
    while (!Serial);  // wait for serial port to connect. Needed for native USB port only

    // Instantiate a CallbackCaller and attach a lambda function.
    Bas::CallbackCaller caller;
    caller.begin([]()
    {
        Serial.println("Lambda callback called!");
    });

    // Call the attached lambda function.
    caller.call();
}

void loop()
{
}