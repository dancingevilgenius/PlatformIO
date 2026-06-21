#include <Arduino.h>

// --- Forward Declarations ---
void setupQwiicMotorDriver();
void setupAlphanumeric();
void setup8x8();
void loopAlphanumeric();
void loop8x8();

/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <SparkFun_Alphanumeric_Display.h> //Click here to get the library: http://librarymanager/All#SparkFun_Qwiic_Alphanumeric_Display by SparkFun
#include "MedianFilterLib2.h"
#include "SCMD.h"
#include "SCMD_config.h" //Contains #defines for common SCMD register names and values

// Alphanumeric Segmented display
HT16K33 display;

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output
#define NUM_ROWS 8
#define NUM_COLS 8
int dist8x8[NUM_ROWS][NUM_COLS] = {
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1,
                                  -1,-1,-1,-1,-1,-1,-1,-1
                                  };

// Median Filter - to remove abnormal data readings.
// Define number of sensors and filter window size
//const int NUM_SENSORS = 3; //
const int WINDOW_SIZE = 5;

// Array of MedianFilter objects
MedianFilter2<int> filter[NUM_ROWS][NUM_COLS] =

{
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)},
  {MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE),MedianFilter2<int>(WINDOW_SIZE)}
};

// For Serical Controlled Motor Driver (SCMD) also known as th Sparkfun QWIIC Motor Driver
SCMD myMotorDriver; //This creates the main object of one motor driver and connected peripherals.
#define DIR_FW  0
#define DIR_RV  1
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1


void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Mini Sumo: SparkFun 8x8 Imager, Alphanumeric Display, QWIIC Motor Driver");

  Wire1.begin(); //This resets to 100kHz I2C

  setupAlphanumeric();

  setup8x8();

  setupQwiicMotorDriver();
}


void setupQwiicMotorDriver(){
  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face is I2C_MODE 
  myMotorDriver.settings.commInterface = I2C_MODE;

  //  set address if I2C configuration selected with the config jumpers
  myMotorDriver.settings.I2CAddress = 0x5D; //config pattern is "1000" (default) on board for address 0x5D

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;

  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.print("begin() return word ");
    Serial.print(myMotorDriver.begin(), HEX);
    Serial.print(" : ");
    Serial.println( "ID mismatch, trying again" );    
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for peripherals before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  //Uncomment code for motor 0 inversion
  //while( myMotorDriver.busy() );
  //myMotorDriver.inversionMode(0, 1); //invert motor 0

  //Uncomment code for motor 1 inversion
  while ( myMotorDriver.busy() ); //Waits until the SCMD is available.
  myMotorDriver.inversionMode(1, 1); //invert motor 1

  while ( myMotorDriver.busy() )
    ; // Do nothing until driver is available

  myMotorDriver.enable(); //Enables the output driver hardware
  Serial.println("QWIIC Motor Driver initialized.");
  display.print("MOTO");
  delay(3000);
}


void setupAlphanumeric(){
  bool displaySuccess = display.begin(
          DEFAULT_ADDRESS,
          DEFAULT_NOTHING_ATTACHED,
          DEFAULT_NOTHING_ATTACHED,
          DEFAULT_NOTHING_ATTACHED,
          Wire1); 
  if (!displaySuccess)
  {
    Serial.println("Alphanumeric display did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Alphanumeric display initialized.");

  display.print("ALPH");
  delay(2000);
}

void setup8x8(){
  Serial.println("Initializing Sparkfun 8x8 board. This can take up to 10s. Please wait.");
  //Wire1.begin();
  Wire1.setClock(400000); //Sensor has max I2C freq of 400kHz 

  if (myImager.begin((DEFAULT_I2C_ADDR >> 1), Wire1) == false)
  {
    Serial.println(F("Sparkfun TOF 8x8 Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }

  myImager.setResolution(NUM_ROWS*NUM_COLS); //Enable all 64 pads
  
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width

  myImager.startRanging();

  Serial.print(F("Sparkfun TOF 8x8 Sensor initialized  with imageWidth:"));
  Serial.println(imageWidth);

  // Put this on display.
  display.print("8x8");
  delay(2000);
}


void loop()
{
  loop8x8();

  //loopAlphanumeric();

  delay(70); //Small delay between polling
}

void loopAlphanumeric(){
  // display value of last/bottom row, one of the middle columns
  int d = dist8x8[7][4]; 


  char buffer[5];
  sprintf(buffer, "%d", d);
  String distStr = buffer;

  display.print(distStr.c_str());
}

void loop8x8(){
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      int raw_d, filtered_d;
      int row, col;
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
      {
          if(y > 0){
            row = y/8;
          }else {
            row = y;
          }
          //Serial.print("row:"); Serial.print(row);
        
        for (int x = imageWidth - 1 ; x >= 0 ; x--)
        //for(int x=0 ; i< imageWidth  ; i++)
        {
          if(true){
            raw_d = measurementData.distance_mm[x + y];

            //Serial.print("y:"); Serial.print(row);
            //Serial.print(" x:"); Serial.print(x);
            // Save for later
            col = x;
            filtered_d = filter[row][col].GetFiltered();

            if(abs(raw_d - filtered_d) < 100){
              filter[row][col].AddValue(raw_d);
              dist8x8[row][col] = filtered_d;
            } else {
              Serial.print("Anomoly!");
              Serial.print("raw_d:");
              Serial.println(raw_d);
            }
            //Serial.print("\t");
            //Serial.print(filtered_d);

          }
        }
        if(true){
          //Serial.println();
        }
      }
      //Serial.println();
    }
  }

}
