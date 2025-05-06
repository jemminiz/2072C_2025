// screen.hpp
#pragma once

#include "display/lv_core/lv_obj.h"
#include "display/lv_misc/lv_color.h"
#include <string>

struct AutonFunction {
    const char* name;
    void (*function)();
};

extern AutonFunction autonFunctions[];

extern std::string calibrationGifs[];
extern int calibrationGifsSize;

extern std::string matchGifs[];
extern int matchGifsSize;

// Declare other functions
void calibrationScreen();
void particlesjs();
void calibrationScreenInit();
void autonSelectorScreenInit(AutonFunction autonFunctions[], size_t autonCount, lv_color_t color_scheme);
void runSelectedAuton(AutonFunction autonFunctions[], size_t autonCount);

lv_obj_t* imageContainer(lv_obj_t *scr, const char* imagePath);