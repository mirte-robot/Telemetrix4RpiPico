#include <module/Shutdown_Module.hpp>
// #include "Telemetrix4RpiPico.hpp"
#include "hardware/watchdog.h"
#include "led_pin.hpp"
bool check_usb_connection();
void Shutdown_Relay::readModule() {
  if (this->enabled) {
    if (time_us_32() - this->start_time > (this->wait_time * 1'000'000) ||
        !check_usb_connection()) {
      gpio_put(this->pin, this->enable_on);
      // relay will be turned off and power will be cut

      // enable watchdog and wait for reset when the relay is not connected
      //   or
      // not working. Don't want the pico to be stuck
      enable_watchdog();
      while (true) {
        led_debug(100, 100);
      }
    }
  }
}
Shutdown_Relay::Shutdown_Relay(std::vector<uint8_t> &data) {
  this->pin = data[0];
  this->enable_on = data[1];
  this->wait_time = data[2]; // seconds
  gpio_init(this->pin);
  gpio_set_dir(this->pin, GPIO_OUT);
  gpio_put(this->pin, !this->enable_on);
  this->start_time = time_us_32();
  this->enabled = false;
}

void Shutdown_Relay::writeModule(std::vector<uint8_t> &data) {
  if (data[0] == 1) // trigger to start the countdown
  {
    this->start_time = time_us_32();
    this->enabled = true;
    disable_watchdog();
    watchdog_enable_shutdown = false; // dont let ping pet the watchdog
  } else {
    this->enabled = false;
    enable_watchdog();
    watchdog_enable_shutdown = true;
  }
  // set_led_pin(this->enabled);
  // gpio_put(LED_PIN, this->enabled);
}
