#ifndef HARDWARE_H
#define HARDWARE_H

#include <Adafruit_AS7341.h>
#include <Adafruit_NeoPixel.h>
#include "constants.h"

// Spectrometer sensor
extern Adafruit_AS7341 as7341;

// NeoPixel LED strip
extern Adafruit_NeoPixel strip;

#endif // HARDWARE_H