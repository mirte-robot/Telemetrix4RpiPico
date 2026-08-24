#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <led_pin.hpp>
#include <stdio.h>
const uint LED_PIN = 25; // board LED

#ifdef CYW43_WL_GPIO_LED_PIN
#include <pico/cyw43_arch.h>
#define TEST_LED_PIN 1
#else
// no need to check if no wifi support compiled into.
#define TEST_LED_PIN 0
#endif

#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

bool internal_led_pin =
    true; // true if the board has an internal led pin, false if it does not
bool initialized_led =
    false; // true if the led has been initialized, false if it has not
void init_led() {
  // check if wifi or normal board:
//   https://pip-assets.raspberrypi.com/categories/686-raspberry-pi-pico-w/documents/RP-008257-DS-2-connecting-to-the-internet-with-pico-w.pdf
// page 12, section 2.4
// when pin 25 is low, pico w adc3 will be 0, pico: will be ~0x2cd.
#if TEST_LED_PIN
  adc_init();
  adc_gpio_init(29);
  adc_select_input(3);
  const float conversion_factor = 3.3f / (1 << 12);
  uint16_t result = adc_read();
  //   printf("ADC3 value: 0x%03x, voltage: %f V\n", result,
  //          result * conversion_factor);
  gpio_init(25);
  gpio_set_dir(25, GPIO_IN);
  uint value = gpio_get(25);
  //   printf("GP25 value: %i", value);
  if (result > 0x100) {
    internal_led_pin = true;
  } else {
    internal_led_pin = false;
  }

  if (!internal_led_pin) {
    // apparently we're pico-w board, which has no internal led pin
    cyw43_arch_init();
  } else {
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
  }
#else
  //   no support for wifi, so just assume it's a normal pico board
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
  initialized_led = true;

  led_debug(3, 100);
}

void set_led_pin(bool value) {
  // value = 1;
  // return;
  if (!initialized_led) {
    init_led();
  }
  if (internal_led_pin) {
    gpio_put(PICO_DEFAULT_LED_PIN, value);
  } else {
#if PICO_CYW43_SUPPORTED
    // apparently we're pico-w board, which has no internal led pin
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, value);
#endif
  }
}

/************************************************************
 * Blink the board led
 * @param blinks - number of blinks
 * @param delay - delay in milliseconds
 */
void led_debug(int blinks, uint16_t delay) {
  for (int i = 0; i < blinks; i++) {
    set_led_pin(1);
    // gpio_put(LED_PIN, 1);
    sleep_ms(delay);
    set_led_pin(0);
    // gpio_put(LED_PIN, 0);
    sleep_ms(delay);
  }
}