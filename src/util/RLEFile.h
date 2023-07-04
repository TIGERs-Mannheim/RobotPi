#pragma once

#include <fstream>
#include "interface/RLEFrame.h"

/* Wraps multiple RLE frames in one file,
 * each prepended by a 4 byte (little-endian) integer describing the length of the following frame.
 */

class RLEFile
{
public:
    static RLEFile openNext(const std::string& mediapath = "/media");

    explicit RLEFile(const std::string& path);

    bool readNext(RLEFrame& frame);
    void append(const RLEFrame& frame);

private:
    std::fstream file_;
};
