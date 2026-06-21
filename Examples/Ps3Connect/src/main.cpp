#include <Arduino.h>

#include <Ps3Controller.h>

#define PS3_BLACK_BLACK "00:19:c1:c2:d8:9b"

void setup() {
    Serial.begin(115200);
    Ps3.begin(PS3_BLACK_BLACK);
    Serial.println("Ready.");
}

void loop()
{
  if (Ps3.isConnected()){
    Serial.println("Connected!");
  }else{
    Serial.println("Not Connected");
  }

  delay(3000);
}

