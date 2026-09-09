#pragma once
#include "ssd1306.h"
#define ENABLE_MIRTE_MASTER 1

extern bool is_mm;

extern pico_ssd1306::SSD1306 *mirte_master_display;

void mm_detect();

void mm_loop();

extern uint8_t mirte_logo[1024];