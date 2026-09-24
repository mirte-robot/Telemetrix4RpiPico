#include "mirte_master.hpp"
#include "led_pin.hpp"
bool is_mm = false; // whether the board is a mirte master pcb, determined by
                    // checking if the uart pins are tied together
#if !ENABLE_MIRTE_MASTER

void mm_detect() { is_mm = false; }
void mm_loop() {
  // do nothing, as mm is disabled
}

#else
#include "Telemetrix4RpiPico.hpp"
#include "uart.hpp"

bool check_usb_connection() {
  // Read in VBUS pin
  // NOTE: this does not work with a pico W, as the VBUS pin is connected to the
  // Wifi chip
  auto const USB_VBUS_PIN = 24;
  return gpio_get(USB_VBUS_PIN);
}

void shutdown_robot_power() {
  const auto relay_pin = 28;
  gpio_init(relay_pin);
  gpio_set_dir(relay_pin, GPIO_OUT);
  gpio_put(relay_pin, 0);
  //   enable_watchdog();
  while (1) {
    led_debug(10, 200);
  }
}

void check_mirte_master() {
  // #if DISABLE_USB_CHECK
  //   return;
  // #endif
  if (uart_enabled) {
    // Not a mirte master pcb (with tied uart pins)
    return;
  }
  auto usb = check_usb_connection();
  // gpio_put(LED_PIN, usb);
  // Assume the pico is put on a mirte-master pcb
  // when the pc is shut down, but did not inform the pico for the relay, then
  // the power will stay on check usb connection, if not connected, then turn
  // off the relay
  static auto start_time = 0;
  if (!usb) {
    if (start_time == 0) {
      start_time = time_us_32();
    }
    if (time_us_32() - start_time >
        100'000'000) { // Wait 100s for a usb connection
      shutdown_robot_power();
    }
  } else {
    start_time = 0;
  }
}

const auto button_pin = 27;
void init_mm_button_hold() {
  gpio_init(button_pin);
  gpio_set_dir(button_pin, GPIO_IN);
  gpio_pull_up(button_pin);
}

const auto button_hold_value =
    1; // The value read from the button pin when the button is held

void detect_mm_button_hold() {
  const auto button_pin = 27;
  static decltype(time_us_32()) button_hold_start_time = 0;
  static bool button_released_start = false;

  bool button_held = gpio_get(button_pin) == button_hold_value;
  if (button_held && button_released_start) {
    if (button_hold_start_time == 0) {
      button_hold_start_time = time_us_32();
    }
    if (time_us_32() - button_hold_start_time > 10'000'000) { // 10s
      shutdown_robot_power();
    }
  } else {
    button_hold_start_time = 0;
    if (button_held) {
      // The button is currently held, but we haven't detected a hold yet, so
      // do nothing until it's released and held again
    } else {
      // The button is currently not held, so we can start detecting for a hold
      // after it's pressed again
      button_released_start = true;
    }
  }
  // set_led_pin(check_usb_connection());
  // gpio_put(25, check_usb_connection());
}

void mm_detect() {
  if (uart_enabled) {
    is_mm = false;
  } else {
    is_mm = true;
    init_mm_button_hold();
  }
}

void mm_loop() {
  if (is_mm) {
    check_mirte_master();
  }
  detect_mm_button_hold();
}
#endif