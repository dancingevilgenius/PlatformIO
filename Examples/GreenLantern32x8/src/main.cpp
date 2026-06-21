#include <Arduino.h>

// --- Forward Declarations ---
void writeText(String msg);


#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)

int matrixW = 32;
int matrixH = 8;
#define PIN 13 // OUTPUT PIN FROM ARDUINO TO MATRIX D-In

Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(matrixW, matrixH, PIN,
                            NEO_MATRIX_TOP     + NEO_MATRIX_LEFT +
                            NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG, //NEO_MATRIX_COLUMNS + NEO_MATRIX_ZIGZAG
                            NEO_GRB            + NEO_KHZ800);

const uint16_t colors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(255, 255, 0), matrix.Color(0, 0, 255), matrix.Color(255, 0, 255), matrix.Color(0, 255, 255), matrix.Color(255, 255, 255)
};

#define arr_len( x )  ( sizeof( x ) / sizeof( *x ) ) // Calculation of Array Size;

int pixelPerChar = 6; // Width of Standard Font Characters is 8X6 Pixels
int x = matrix.width(); // Width of the Display
int pass = 0; // Counter
int i = 0; // Counter
int clr = 0; // Counter for Indexing Array of Colors

char msg[] = "-------------------"; // BLANK Message of Your Choice;

void setup() {
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(5);
  matrix.setTextColor(colors[1]);
}

void loop() {
  int arrSize = arr_len( colors );
  //char exampleText[] = "Superman could not make it. You got an upgrade"; // SCROLLING Message of Your Choice;
  char exampleText[] = "SUPERMAN COULD NOT MAKE IT. YOU GOT AN UPGRADE"; // SCROLLING Message of Your Choice;
  writeText(exampleText);
  delay(1000);

  matrix.print(msg); //Print the Message String;
  matrix.show();
  delay(1000);

  /*  Commands for Debugging  */
  char msgText[] = "Let's Start Over";
  matrix.print(msgText); // Scrolling Print the Message String;
  matrix.show();
  delay(1000);
  
  writeText("End of Loop"); //Print the Message String;
  delay(1000);
  /*  Commands for Debugging  */
  
  // LATHER - RINSE - REPEAT Is Why coders never leave the Shower;
}

/* --------------------  FUNCTIONS BELOW  --------------------  */
void writeText(String msg) {
  int arrSize = arr_len( colors ); // Array of Text Colors;
  int msgSize = (msg.length() * pixelPerChar) + (2 * pixelPerChar); // CACULATE message length;
  int scrollingMax = (msgSize) + matrix.width(); // ADJUST Displacement for message length;

  x = matrix.width(); // RESET Cursor Position and Start Text String at New Position on the Far Right;


  while (clr <= arrSize) {
    /* Change Color with Each Pass of Complete Message */
    matrix.setTextColor(colors[1]);

    matrix.fillScreen(0); // BLANK the Entire Screen;
    matrix.setCursor(x, 0); // Set Starting Point for Text String;
    matrix.print(msg); // Set the Message String;

    /* SCROLL TEXT FROM RIGHT TO LEFT BY MOVING THE CURSOR POSITION */
    if (--x < -scrollingMax ) {
      /*  ADJUST FOR MESSAGE LENGTH  */
      // Decrement x by One AND Compare New Value of x to -scrollingMax;
	  // This Animates (moves) the text by one pixel to the Left;

      x = matrix.width(); // After Scrolling by scrollingMax pixels, RESET Cursor Position and Start String at New Position on the Far Right;
   
    }
    matrix.show(); // DISPLAY the Text/Image
    delay(100); // SPEED OF SCROLLING or FRAME RATE;
  }


/* LATHER - RINSE - REPEAT - Why coders have such nice hair */
}

