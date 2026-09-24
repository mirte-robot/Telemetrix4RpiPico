#pragma once
#include <stdint.h>
typedef unsigned int uint;

void enable_watchdog();
void disable_watchdog();
extern bool watchdog_triggered;

extern bool watchdog_enable_shutdown; // if false, then don't do anything with
                                      // the watchdog and just wait for shutdown