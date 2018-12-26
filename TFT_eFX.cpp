// Created by Bodmer 15/11/18 as an example for the TFT_eFX graphics extension library to TFT_eSPI

// Use as a template when you add your graphics functions to the libraries files:
// TFT_eFX.h     - add function prototype, see myGraphicsFunction() line
// TFT_eFX.cpp   - add function with code, see myGraphicsFunction() lines
// keywords.txt  - add function name to highlight in IDE, see myGraphicsFunction line


// https://github.com/Bodmer/TFT_eSPI
#include <TFT_eSPI.h>            // Include the graphics library (this includes the sprite functions)

// https://github.com/Bodmer/TFT_eFX
#include <TFT_eFX.h>             // Include the extension graphics functions library

TFT_eSPI tft = TFT_eSPI();       // Create object "tft"

TFT_eFX  efx = TFT_eFX(&tft);    // Create TFT_eFX object "efx" with pointer to "tft" object

// =========================================================================
void setup(void) {

  Serial.begin(115200);

  tft.init();

  tft.setRotation(0);

  tft.fillScreen(TFT_BLACK);  
}

// =========================================================================
void loop() {

  // This example draws parrot.bmp to the tft
 /* !!NOT WORKING EXAMPLE!! */
  efx.drawBMP("/parrot.bmp", 100, 100);

  
  delay(200);
}
