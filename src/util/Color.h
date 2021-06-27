/*
 * Color.h
 *
 *  Created on: 24.11.2020
 *      Author: AndreR
 */

#pragma once

#include <cstdint>

class ColorYUV
{
public:
    ColorYUV();
    ColorYUV(uint8_t y, uint8_t u, uint8_t v);

    uint8_t y_;
    uint8_t u_;
    uint8_t v_;
};

class ColorRGB
{
public:
    ColorRGB(uint8_t r, uint8_t g, uint8_t b);
    ColorRGB(float r, float g, float b);
    ColorRGB(uint32_t rgbHex);

    operator ColorYUV() const;

    uint8_t r_;
    uint8_t g_;
    uint8_t b_;
};
