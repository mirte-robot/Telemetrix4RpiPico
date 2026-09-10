#include "mirte_master.hpp"
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
  gpio_put(25, check_usb_connection());
}
void show_boot_screen(i2c_inst *i2c) {
  auto mirte_master_display =
      pico_ssd1306::SSD1306(i2c, 0x3C, pico_ssd1306::Size::W128xH64);
  mirte_master_display.setPostWrite(false);
  mirte_master_display.setOrientation(0);

  mirte_master_display.clear();
  mirte_master_display.addBitmapImage(0, 0, 128, 64, mirte_logo);
  mirte_master_display.sendBuffer();
};

// i2c pins on mirte pioneer pcb
std::array<std::array<int, 3>, 2> i2c_pcb_pins = {
    {{11, 10, 1}, {5, 4, 0}}}; // scl, sda, port

void show_boot_screen(bool mm_pcb) {
#if ENABLE_BOOT_SCREEN
  if (!mm_pcb) {
    for (auto pins : i2c_pcb_pins) {
      reset_i2c(pins[0], pins[1], pins[2]);
      if (check_addr(pins[2], 0x3C)) {
        show_boot_screen(pins[2] == 0 ? i2c0 : i2c1);
      }
      // reset pins to default state, then config can set them later
      gpio_set_function(pins[0], GPIO_FUNC_NULL);
      gpio_set_function(pins[1], GPIO_FUNC_NULL);
    }
  } else {
    // Mirte master pcb
    reset_i2c(3, 2, 1);
    if (!check_addr(1, 0x3C)) {
      return;
    }
    show_boot_screen(i2c1);
  }
#endif
}

void mm_detect() {
  if (uart_enabled) {
    is_mm = false;
    show_boot_screen(false);
  } else {
    is_mm = true;
    init_mm_button_hold();
    show_boot_screen(true);
  }
}

void mm_loop() {
  if (is_mm) {
    check_mirte_master();
    detect_mm_button_hold();
  }
}
#endif