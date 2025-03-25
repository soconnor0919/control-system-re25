#ifndef COLOR_H
#define COLOR_H

enum class Color {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    CYAN,
    MAGENTA,
    ORANGE,
    WHITE,
    BLACK,
    CUSTOM,
};

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

const RGB colorValues[] = {
    {255, 0, 0},    // Red
    {0, 255, 0},    // Green
    {0, 0, 255},    // Blue
    {255, 255, 0},  // Yellow
    {0, 255, 255},  // Cyan
    {255, 0, 255},  // Magenta
    {255, 50, 0},  // Orange
    {255, 255, 255},// White
    {0, 0, 0},      // Black
    {0, 0, 0}       // Custom
};

#endif // COLOR_H