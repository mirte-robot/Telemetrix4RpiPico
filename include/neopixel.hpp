#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Telemetrix4RpiPico.pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/sem.h"
#include "pico/stdlib.h"
auto const max_pixels = 100;

static void dma_init(PIO pio, unsigned int sm, uint32_t NEOC_NOF_LEDS_IN_LANE);

class TMX_NeoPixel {
public:
  void init(int LED_count, int LED_pin, int RED_fill, int GREEN_fill,
            int BLUE_fill);
  void setPixelColor(int pixel, int RED, int GREEN, int BLUE, bool show = true);
  void show();
  void clear();
  void fill(int RED, int GREEN, int BLUE, bool show);

private:
  int LED_count;
  int LED_pin;
  uint32_t pixels[max_pixels];
};
