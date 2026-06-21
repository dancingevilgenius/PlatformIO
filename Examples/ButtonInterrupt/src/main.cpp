#include <Arduino.h>

// --- Forward Declarations ---
void blink();

const byte ledPin = 14;
const byte interruptPin = 25;  // input pin that the interruption will be attached to
volatile byte state = LOW;  // variable that will be updated in the ISR

void setup() {

  Serial.begin(9600);
  Serial.println("Button via interrupt test!");

  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() {
  digitalWrite(ledPin, state);  
}

void blink() {
  state = !state;
  
  
  // if(state){
  // } else {
  //   Serial.println("button: ");
  // } 
   
}
