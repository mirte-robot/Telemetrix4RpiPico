#pragma once
#include <stdint.h>

extern bool internal_led_pin; // true if the board has an internal led pin,
                              // false if it does not
extern bool initialized_led;  // true if the led has been initialized, false if
                              // it has not
void init_led();
void set_led_pin(bool value);

void led_debug(int blinks, uint16_t delay);