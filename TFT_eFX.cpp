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


// This drawBMP function contains code from:
// https://github.com/adafruit/Adafruit_ILI9341/blob/master/examples/spitftbitmap/spitftbitmap.ino
// Here is Bodmer's version: this uses the ILI9341 CGRAM coordinate rotation features inside the display and
// buffers both file and TFT pixel blocks, it typically runs about 2x faster for bottom up encoded BMP images

void TFT_eFX::drawBMP(String filename, uint8_t x, uint16_t y) {
  // Flips the TFT internal SGRAM coords to draw bottom up BMP images faster, in this application it can be fixed
  boolean flip = 1;

  if ((x >= _tft->width()) || (y >= _tft->height())) return;

  fs::File bmpFile;
  int16_t  bmpWidth, bmpHeight;   // Image W+H in pixels
  uint32_t bmpImageoffset;        // Start address of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3 * BUFFPIXEL];    // file read pixel buffer (8 bits each R+G+B per pixel)
  uint16_t tftbuffer[BUFFPIXEL];       // TFT pixel out buffer (16-bit per pixel)
  uint8_t  rgb_ptr = sizeof(sdbuffer); // read 24 bit RGB pixel data buffer pointer (8 bit so BUFF_SIZE must be less than 86)
  boolean  goodBmp = false;            // Flag set to true on valid header parse
  int16_t  w, h, row, col;             // to store width, height, row and column
  uint8_t rotation;      // to restore rotation
  uint8_t  tft_ptr = 0;  // TFT 16 bit 565 format pixel data buffer pointer

  // Check file exists and open it
  Serial.println(filename);
  if ( !(bmpFile = SPIFFS.open(filename, "r")) ) {
    Serial.println(F(" File not found")); // Can comment out if not needed
    return;
  }

  // Parse BMP header to get the information we need
  if (read16(bmpFile) == 0x4D42) { // BMP file start signature check
    read32(bmpFile);       // Dummy read to throw away and move on
    read32(bmpFile);       // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    read32(bmpFile);       // Dummy read to throw away and move on
    bmpWidth  = read32(bmpFile);  // Image width
    bmpHeight = read32(bmpFile);  // Image height

    // Only proceed if we pass a bitmap file check
    // Number of image planes -- must be '1', depth 24 and 0 (uncompressed format)
    if ((read16(bmpFile) == 1) && (read16(bmpFile) == 24) && (read32(bmpFile) == 0)) {
      goodBmp = true; // Supported BMP format
      // BMP rows are padded (if needed) to 4-byte boundary
      rowSize = (bmpWidth * 3 + 3) & ~3;
      // Crop area to be loaded
      w = bmpWidth;
      h = bmpHeight;

      // We might need to alter rotation to avoid tedious file pointer manipulation
      // Save the current value so we can restore it later
      rotation = _tft->getRotation();
      // Use TFT SGRAM coord rotation if flip is set for 25% faster rendering (new rotations 4-7 supported by library)
      if (flip) _tft->setRotation((rotation + (flip<<2)) % 8); // Value 0-3 mapped to 4-7

      // Calculate new y plot coordinate if we are flipping
      switch (rotation) {
        case 0:
          if (flip) y = _tft->height() - y - h; break;
        case 1:
          y = _tft->height() - y - h; break;
          break;
        case 2:
          if (flip) y = _tft->height() - y - h; break;
          break;
        case 3:
          y = _tft->height() - y - h; break;
          break;
      }

      // Set TFT address window to image bounds
      // Currently, image will not draw or will be corrputed if it does not fit
      // TODO -> efficient clipping, but I don't need it to be idiot proof ;-)
      _tft->setAddrWindow(x, y, x + w - 1, y + h - 1);

      // Finally we are ready to send rows of pixels, writing like this avoids slow 32 bit multiply in 8 bit processors
      for (uint32_t pos = bmpImageoffset; pos < bmpImageoffset + h * rowSize ; pos += rowSize) {
        // Seek if we need to on boundaries and arrange to dump buffer and start again
        if (bmpFile.position() != pos) {
          bmpFile.seek(pos, fs::SeekSet);
          rgb_ptr = sizeof(sdbuffer);
          //Serial.println("Seeking in file >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        }

        // Fill the pixel buffer and plot
        for (col = 0; col < w; col++) { // For each column...
          // Time to read more pixel data?
          if (rgb_ptr >= sizeof(sdbuffer)) {
            // Push tft buffer to the display
            if (tft_ptr) {
              // Here we are sending a uint16_t array to the function
              _tft->pushColors(tftbuffer, tft_ptr);
              tft_ptr = 0; // tft_ptr and rgb_ptr are not always in sync...
            }
            // Finally reading bytes from SD Card
            bmpFile.read(sdbuffer, sizeof(sdbuffer));
            rgb_ptr = 0; // Set buffer index to start
          }
          // Convert pixel from BMP 8+8+8 format to TFT compatible 16 bit word
          // Blue 5 bits, green 6 bits and red 5 bits (16 bits total)
          // Is is a long line but it is faster than calling a library fn for this
          tftbuffer[tft_ptr] = (sdbuffer[rgb_ptr++] >> 3) ;
          tftbuffer[tft_ptr] |= ((sdbuffer[rgb_ptr++] & 0xFC) << 3);
          tftbuffer[tft_ptr] |= ((sdbuffer[rgb_ptr++] & 0xF8) << 8);
          tft_ptr++;
        } // Next row
      }   // All rows done

      // Write any partially full buffer to TFT
      if (tft_ptr) _tft->pushColors(tftbuffer, tft_ptr);

    } // End of bitmap access
  }   // End of bitmap file check

  bmpFile.close();
 if(!goodBmp) Serial.println(F("BMP format not recognised."));
 // tft_->setRotation(rotation); // Put back original rotation, map gets mirrorred if format is not recognized -- not good :-)
}


