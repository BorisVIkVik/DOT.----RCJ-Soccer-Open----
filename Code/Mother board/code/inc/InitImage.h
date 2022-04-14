#ifndef INIT_IMAGE_BUFFER
#define INIT_IMAGE_BUFFER

#include <stdlib.h> 

uint8_t initImageBuffer[128*64] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 240, 0, 0, 32, 16, 16, 16, 16, 224, 0, 0, 32, 16, 16, 16, 16, 224, 0, 0, 32, 16, 16, 16, 16, 224, 0, 0, 224, 16, 16, 16, 16, 224, 0, 0, 192, 32, 16, 16, 16, 16, 32, 96, 0, 0, 0, 0, 0, 0, 224, 16, 16, 16, 32, 254, 0, 0, 240, 32, 16, 16, 16, 224, 0, 0, 64, 32, 16, 16, 16, 16, 32, 96, 128, 0, 0, 224, 16, 16, 16, 16, 224, 0, 0, 224, 16, 16, 16, 32, 240, 0, 0, 224, 16, 16, 16, 16, 224, 0, 0, 16, 32, 192, 0, 0, 0, 0, 0, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 4, 15, 0, 0, 7, 9, 9, 9, 9, 7, 0, 0, 4, 8, 8, 8, 8, 7, 0, 0, 4, 8, 8, 8, 8, 7, 0, 0, 7, 8, 8, 8, 8, 7, 0, 0, 17, 34, 66, 66, 68, 68, 36, 28, 0, 0, 0, 0, 0, 0, 7, 8, 8, 8, 4, 15, 0, 0, 15, 0, 0, 0, 0, 15, 0, 0, 16, 32, 64, 64, 64, 64, 32, 48, 15, 0, 0, 7, 8, 8, 8, 8, 7, 0, 0, 7, 8, 8, 8, 4, 127, 0, 0, 7, 8, 8, 8, 8, 7, 0, 0, 60, 66, 66, 67, 66, 66, 66, 66, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 192, 192, 32, 16, 8, 8, 132, 4, 4, 8, 8, 240, 252, 4, 252, 216, 136, 144, 16, 16, 32, 32, 32, 64, 64, 64, 128, 128, 128, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 252, 3, 128, 64, 32, 47, 24, 16, 16, 16, 136, 136, 136, 140, 143, 139, 136, 200, 63, 63, 47, 63, 63, 95, 68, 72, 190, 190, 252, 124, 127, 249, 128, 255, 248, 0, 0, 0, 0, 0, 0, 0, 0, 248, 248, 248, 248, 248, 0, 0, 0, 0, 0, 0, 248, 248, 248, 248, 248, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 192, 224, 224, 240, 240, 248, 120, 120, 120, 120, 120, 120, 248, 240, 240, 224, 224, 192, 128, 0, 0, 0, 0, 0, 0, 128, 224, 224, 240, 240, 248, 120, 120, 120, 120, 120, 120, 120, 120, 248, 248, 248, 248, 248, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 225, 16, 8, 4, 2, 2, 1, 1, 193, 32, 144, 72, 248, 36, 36, 36, 37, 37, 37, 38, 38, 74, 76, 148, 36, 234, 10, 18, 17, 17, 8, 14, 241, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 248, 254, 255, 255, 255, 15, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 15, 255, 255, 255, 254, 248, 0, 0, 248, 255, 255, 255, 255, 7, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 4, 4, 8, 8, 255, 33, 16, 8, 255, 8, 8, 8, 8, 8, 8, 8, 8, 16, 16, 32, 195, 255, 8, 4, 4, 2, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 63, 255, 255, 255, 248, 224, 192, 128, 0, 0, 0, 0, 0, 0, 128, 192, 224, 248, 255, 255, 255, 63, 15, 0, 0, 15, 127, 255, 255, 255, 240, 192, 128, 128, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 129, 66, 124, 127, 42, 41, 47, 41, 47, 41, 42, 36, 68, 66, 129, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 0, 1, 3, 3, 7, 7, 15, 15, 15, 15, 15, 15, 15, 15, 7, 7, 3, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 7, 7, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 4, 8, 8, 16, 16, 16, 16, 16, 16, 16, 16, 16, 8, 8, 4, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#endif
