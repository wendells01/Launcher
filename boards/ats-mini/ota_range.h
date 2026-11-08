#pragma once

#include <stdint.h>

inline bool atsMiniValidOtaRange(uint32_t imageSize, uint32_t appOffset, uint32_t appSize) {
    return imageSize > 0 && imageSize <= 0x1000000 && appSize > 0 && appOffset < imageSize &&
           appSize <= imageSize - appOffset;
}
