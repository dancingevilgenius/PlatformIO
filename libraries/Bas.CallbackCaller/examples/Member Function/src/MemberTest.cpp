#include "MemberTest.h"

void MemberTest::memberCallback()
{
    // Print the value passed to the begin() function.
    Serial.print("Member callback called, value is ");
    Serial.println(this->value);
}

void MemberTest::begin(int value)
{
    // Store the passed value argument.
    // The callback will later print it to show that it is being called on the correct instance of this class.
    this->value = value;

    // Attach a callback by means of a lambda which calls the memberCallback() member function.
    Serial.println("Attaching callback.");
    this->callbackCaller.begin([this]{ memberCallback(); });
}

void MemberTest::raiseCallback()
{
    // Raise the callback
    Serial.println("Raising callback.");
    this->callbackCaller.call();
}
