/*
 * Color.cpp
 *
 *  Created on: 24.11.2020
 *      Author: AndreR
 */

#include "Color.h"

ColorYUV::ColorYUV()
:y_(0), u_(0), v_(0)
{
}

ColorYUV::ColorYUV(uint8_t y, uint8_t u, uint8_t v)
:y_(y), u_(u), v_(v)
{
}

ColorRGB::ColorRGB(uint8_t r, uint8_t g, uint8_t b)
:r_(r), g_(g), b_(b)
{
}

ColorRGB::ColorRGB(float r, float g, float b)
:r_(255*r), g_(255*g), b_(255*b)
{
}

ColorRGB::ColorRGB(uint32_t rgbHex)
:r_(rgbHex >> 16), g_((rgbHex & 0x00FF00) >> 8), b_(rgbHex & 0x0000FF)
{
}

ColorRGB::operator ColorYUV() const
{
    float Y =  0.257f * r_ + 0.504f * g_ + 0.098f * b_ +  16.0f;
    float U = -0.148f * r_ - 0.291f * g_ + 0.439f * b_ + 128.0f;
    float V =  0.439f * r_ - 0.368f * g_ - 0.071f * b_ + 128.0f;

    return ColorYUV(static_cast<uint8_t>(Y), static_cast<uint8_t>(U), static_cast<uint8_t>(V));
}
