#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
}
int counter = 0;
int x = 0;
int y = 0;

void loop()
{
  // put your main code here, to run repeatedly:
  int val = myFunction(x, y);
  x = x + 1;
  y = y + 2;
  Serial.print("val:");
  Serial.println(val);
  delay(1000);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}