#include "LEDUtil.h"
#include "../hardware.h"

Color currentColor = Color::BLACK;

// Function Implementations

/*
 * Initialize the LED strip.
 */
void initLED() {
    strip.begin();
    strip.show();

    // Set all LEDs to off
    setLEDColor(Color::BLACK);

    // Store LED Color
    currentColor = Color::BLACK;
}

/*
 * Set the color of the LED strip to a custom RGB value.
 *
 * @param r: The red value of the color (0-255)
 * @param g: The green value of the color (0-255)
 * @param b: The blue value of the color (0-255)
 */
void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
    // Interate through each LED and set the color
    for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(i, r, g, b);
    }

    // Set current color as custom
    currentColor = Color::CUSTOM;
}

/*
 * Set the color of the LED strip to a predefined color.
 *
 * @param color: The color to set the LED strip to
 * @see Color
 */
void setLEDColor(Color color) {
    // If the color is the same as the current color, do nothing
    if (color == currentColor) {
        return;
    }

    // Set all LEDs to the specified color
    for (int i = 0; i < LED_COUNT; i++) {
        strip.setPixelColor(
            i, 
            colorValues[static_cast<int>(color)].r, 
            colorValues[static_cast<int>(color)].g, 
            colorValues[static_cast<int>(color)].b
        );
    }

    // Store LED Color
    currentColor = color;

    // Show the LED strip
    strip.show();
}

/*
 * Fill the LED strip with a single color.
 *
 * @param color: The color to fill the LED strip with
 * @param wait: The time to wait between each LED
 */
void colorWipe(Color color, int wait) {
    // If the color is the same as the current color, do nothing
    if (color == currentColor) {
        return;
    }

    // Fill the LED strip with the specified color
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(
            i, 
            colorValues[static_cast<int>(color)].r, 
            colorValues[static_cast<int>(color)].g, 
            colorValues[static_cast<int>(color)].b
        );
        strip.show();
        delay(wait);
    }

    // Store LED Color
    currentColor = color;
}