#ifndef INIT_IMAGE_BUFFER
#define INIT_IMAGE_BUFFER

#include <stdlib.h> 

uint8_t initImageBuffer[128*64] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 240, 0, 0, 32, 16, 16, 16, 16, 224, 0, 0, 32, 16, 16, 16, 16, 224, 0, 0, 32, 16, 16, 16, 16, 224, 0, 0, 224, 16, 16, 16, 16, 224, 0, 0, 192, 32, 16, 16, 16, 16, 32, 96, 0, 0, 0, 0, 0, 0, 224, 16, 16, 16, 32, 254, 0, 0, 240, 32, 16, 16, 16, 224, 0, 0, 64, 32, 16, 16, 16, 16, 32, 96, 128, 0, 0, 224, 16, 16, 16, 16, 224, 0, 0, 224, 16, 16, 16, 32, 240, 0, 0, 224, 16, 16, 16, 16, 224, 0, 0, 16, 32, 192, 0, 0, 0, 0, 0, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 4, 15, 0, 0, 7, 9, 9, 9, 9, 7, 0, 0, 4, 8, 8, 8, 8, 7, 0, 0, 4, 8, 8, 8, 8, 7, 0, 0, 7, 8, 8, 8, 8, 7, 0, 0, 17, 34, 66, 66, 68, 68, 36, 28, 0, 0, 0, 0, 0, 0, 7, 8, 8, 8, 4, 15, 0, 0, 15, 0, 0, 0, 0, 15, 0, 0, 16, 32, 64, 64, 64, 64, 32, 48, 15, 0, 0, 7, 8, 8, 8, 8, 7, 0, 0, 7, 8, 8, 8, 4, 127, 0, 0, 7, 8, 8, 8, 8, 7, 0, 0, 60, 66, 66, 67, 66, 66, 66, 66, 127, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 240, 240, 240, 240, 240, 240, 240, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 240, 240, 240, 240, 240, 240, 240, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 192, 224, 224, 240, 240, 240, 240, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 240, 240, 240, 224, 224, 224, 192, 128, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 128, 192, 192, 224, 224, 224, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 128, 224, 248, 252, 255, 255, 255, 255, 255, 127, 31, 15, 15, 7, 7, 3, 3, 3, 3, 3, 3, 3, 3, 7, 7, 15, 31, 31, 127, 255, 255, 255, 255, 255, 252, 248, 224, 128, 0, 0, 0, 0, 0, 0, 0, 192, 240, 248, 254, 255, 255, 255, 255, 127, 63, 31, 15, 15, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 252, 255, 255, 255, 255, 255, 255, 255, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 255, 255, 255, 255, 255, 255, 255, 252, 0, 0, 0, 0, 0, 252, 255, 255, 255, 255, 255, 255, 255, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 255, 255, 255, 255, 255, 255, 255, 224, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 224, 255, 255, 255, 255, 255, 255, 255, 63, 0, 0, 0, 0, 0, 63, 255, 255, 255, 255, 255, 255, 255, 192, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 255, 255, 255, 255, 255, 255, 255, 255, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 0, 0, 0, 0, 0, 1, 7, 31, 63, 127, 255, 255, 255, 255, 252, 248, 240, 240, 224, 224, 192, 192, 192, 192, 192, 192, 192, 192, 224, 224, 240, 240, 248, 252, 255, 255, 255, 255, 127, 63, 31, 7, 1, 0, 0, 0, 0, 0, 0, 0, 3, 15, 31, 127, 255, 255, 255, 255, 255, 252, 248, 240, 240, 240, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 3, 7, 7, 7, 15, 15, 15, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 15, 15, 15, 15, 7, 7, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 3, 7, 7, 7, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 0, 0, 0, 0};

#endif
