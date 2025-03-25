#ifndef LEDUTIL_H
#define LEDUTIL_H

#include <Adafruit_NeoPixel.h>
#include "../constants.h"
#include "../enums/color.h"

extern Color currentColor;

// Function Declarations
void initLED();
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
void setLEDColor(Color color);
void colorWipe(Color color, int wait);

#endif // LEDUTIL_H