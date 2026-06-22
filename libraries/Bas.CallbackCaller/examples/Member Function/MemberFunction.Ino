#include "src/MemberTest.h"

void setup()
{
    Serial.begin(9600);
    while (!Serial);  // wait for serial port to connect. Needed for native USB port only

    // Instantiate an instance of MemberTest and set the value member to a unique number.
    MemberTest test;
    test.begin(1138);

    // Instantiate a second instance of MemberTest and set the value to a different unique number,
    // so we can differentiate between the two instances.
    MemberTest otherTest;
    otherTest.begin(8008);

    // Have both MemberTest instances raise their callbacks.
    // The serial output will show the different values set for each instance.
    test.raiseCallback();
    otherTest.raiseCallback();
}

void loop()
{
}