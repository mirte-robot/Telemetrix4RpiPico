#pragma once
#include "ssd1306.h"
#define ENABLE_MIRTE_MASTER 1

extern bool is_mm;

void mm_detect();

void mm_loop();

// in mirte_logo.cpp
extern const uint8_t mirte_logo[1024];