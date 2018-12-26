/***************************************************************************************/
// The following class class adds additional graphics functions to the TFT_eSPI library.
// The class inherits the generic drawing graphics functions from the TFT_eSPI class.

// Created by Bodmer 15/11/2018
// See license.txt in root folder of library
/***************************************************************************************/

#include <TFT_eSPI.h>

#include "TFT_eFX.h"

/***************************************************************************************
** Function name:           TFT_eFX
** Description:             Class constructor
***************************************************************************************/
TFT_eFX::TFT_eFX(TFT_eSPI *tft)
{
  _tft = tft;  // Pointer to tft class so we can call member functions

  _efx    = 0; // Initialise dummy variable

}

/***************************************************************************************
** Function name:           myGraphicsFunction
** Description:             Does whatever you want with the graphics library
***************************************************************************************/
void TFT_eFX::myGraphicsFunction(int32_t x, int32_t y, uint16_t color)
{
  // This is just an example that draws a 3x3 pixel block centered on x,y
  _tft->fillRect(x-1, y-1, 3, 3, color);
}

/***************************************************************************************
** Function name:           drawQuadraticBezier
** Description:             Draw a bezier curve between points
***************************************************************************************/
// Plot any quadratic Bezier curve, no restrictions on point positions
// See source code http://members.chello.at/~easyfilter/bresenham.c
void TFT_eFX::drawBezier(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color)
{
  int32_t x = x0 - x1, y = y0 - y1;
  double t = x0 - 2 * x1 + x2, r;

  if (x * (x2 - x1) > 0) {
    if (y * (y2 - y1) > 0)
      if (fabs((y0 - 2 * y1 + y2) / t * x) > abs(y)) {
        x0 = x2; x2 = x + x1; y0 = y2; y2 = y + y1;
      }
    t = (x0 - x1) / t;
    r = (1 - t) * ((1 - t) * y0 + 2.0 * t * y1) + t * t * y2;
    t = (x0 * x2 - x1 * x1) * t / (x0 - x1);
    x = floor(t + 0.5); y = floor(r + 0.5);
    r = (y1 - y0) * (t - x0) / (x1 - x0) + y0;
    drawBezierSegment(x0, y0, x, floor(r + 0.5), x, y, color);
    r = (y1 - y2) * (t - x2) / (x1 - x2) + y2;
    x0 = x1 = x; y0 = y; y1 = floor(r + 0.5);
  }
  if ((y0 - y1) * (y2 - y1) > 0) {
    t = y0 - 2 * y1 + y2; t = (y0 - y1) / t;
    r = (1 - t) * ((1 - t) * x0 + 2.0 * t * x1) + t * t * x2;
    t = (y0 * y2 - y1 * y1) * t / (y0 - y1);
    x = floor(r + 0.5); y = floor(t + 0.5);
    r = (x1 - x0) * (t - y0) / (y1 - y0) + x0;
    drawBezierSegment(x0, y0, floor(r + 0.5), y, x, y, color);
    r = (x1 - x2) * (t - y2) / (y1 - y2) + x2;
    x0 = x; x1 = floor(r + 0.5); y0 = y1 = y;
  }
  drawBezierSegment(x0, y0, x1, y1, x2, y2, color);
}

/***************************************************************************************
** Function name:           drawBezierSegment
** Description:             Draw a bezier segment curve between points
***************************************************************************************/

//  x0, y0 defines p0 etc.
//  coordinates for p0-p3 must be sequentially increasing or decreasing so
//  n0 <= n1 <= n2 or n0 >= n1 >= n2 where n is x or y, e.g.
//
//         p1 x           .x p2      p2 x.
//                   .                       .     x p1
//               .                               .
//            .                                     .
//          .                                         .
//        .                                             .
//      .                                                 .
//  p0 x                                                   x p0
//
void TFT_eFX::drawBezierSegment(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color)
{
  // Check if coordinates are sequential (replaces assert)
  if (((x2 >= x1 && x1 >= x0) || (x2 <= x1 && x1 <= x0))
      && ((y2 >= y1 && y1 >= y0) || (y2 <= y1 && y1 <= y0)))
  {
    // Coordinates are sequential
    int32_t sx = x2 - x1, sy = y2 - y1;
    int32_t xx = x0 - x1, yy = y0 - y1, xy;
    float dx, dy, err, cur = xx * sy - yy * sx;

    if (sx * (int32_t)sx + sy * (int32_t)sy > xx * xx + yy * yy) {
      x2 = x0; x0 = sx + x1; y2 = y0; y0 = sy + y1; cur = -cur;
    }
    if (cur != 0) {
      xx += sx; xx *= sx = x0 < x2 ? 1 : -1;
      yy += sy; yy *= sy = y0 < y2 ? 1 : -1;
      xy = 2 * xx * yy; xx *= xx; yy *= yy;
      if (cur * sx * sy < 0) {
        xx = -xx; yy = -yy; xy = -xy; cur = -cur;
      }
      dx = 4.0 * sy * cur * (x1 - x0) + xx - xy;
      dy = 4.0 * sx * cur * (y0 - y1) + yy - xy;
      xx += xx; yy += yy; err = dx + dy + xy;
      do {
        _tft->drawPixel(x0, y0, color);
        if (x0 == x2 && y0 == y2) return;
        y1 = 2 * err < dx;
        if (2 * err > dy) {
          x0 += sx;
          dx -= xy;
          err += dy += yy;
        }
        if (    y1    ) {
          y0 += sy;
          dy -= xy;
          err += dx += xx;
        }
        yield();
      } while (dy < dx );
    }
    _tft->drawLine(x0, y0, x2, y2, color);
  }
  else Serial.println("Bad coordinate set - non-sequential!");
}

/***************************************************************************************
** Function name:           drawBMP
** Description:             Draw a bitmap image from SPIFFS
***************************************************************************************/

// Bodmers BMP image rendering function


// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(fs::File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(fs::File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}




void drawBMP(String filename, int16_t x, int16_t y) {

  if ((x >= _tft->width()) || (y >= _tft->height())) return;

  fs::File bmpFS;

  // Open requested file on SD card
  bmpFS = SPIFFS.open(filename, "r");

  if (!bmpFS)
  {
    Serial.print("File not found");
    return;
  }

  uint32_t seekOffset;
  uint16_t w, h, row, col;
  uint8_t  r, g, b;

  uint32_t startTime = millis();

  if (read16(bmpFS) == 0x4D42)
  {
    read32(bmpFS);
    read32(bmpFS);
    seekOffset = read32(bmpFS);
    read32(bmpFS);
    w = read32(bmpFS);
    h = read32(bmpFS);

    if ((read16(bmpFS) == 1) && (read16(bmpFS) == 24) && (read32(bmpFS) == 0))
    {
      y += h - 1;

      _tft->setSwapBytes(true);
      bmpFS.seek(seekOffset);

      uint16_t padding = (4 - ((w * 3) & 3)) & 3;
      uint8_t lineBuffer[w * 3 + padding];

      for (row = 0; row < h; row++) {
        
        bmpFS.read(lineBuffer, sizeof(lineBuffer));
        uint8_t*  bptr = lineBuffer;
        uint16_t* tptr = (uint16_t*)lineBuffer;
        // Convert 24 to 16 bit colours
        for (uint16_t col = 0; col < w; col++)
        {
          b = *bptr++;
          g = *bptr++;
          r = *bptr++;
          *tptr++ = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        }

        // Push the pixel row to screen, pushImage will crop the line if needed
        // y is decremented as the BMP image is drawn bottom up
        _tft->pushImage(x, y--, w, 1, (uint16_t*)lineBuffer);
      }
      Serial.print("Loaded in "); Serial.print(millis() - startTime);
      Serial.println(" ms");
    }
    else Serial.println("BMP format not recognized.");
  }
  bmpFS.close();
}

