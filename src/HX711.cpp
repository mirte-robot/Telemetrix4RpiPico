/**
 *
 * HX711 library for Arduino, modified for the rp2040
 * https://github.com/bogde/HX711
 *
 * MIT License
 * (c) 2018 Bogdan Necula
 *
 **/
#include "HX711.hpp"

#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/sync.h>
#include <hardware/timer.h>

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU 1

const int LSBFIRST = 0;
const int MSBFIRST = 1;
#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// -
// https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t shiftInSlow(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
  uint8_t value = 0;
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    gpio_put(clockPin, 1);
    busy_wait_us(1);
    if (bitOrder == LSBFIRST)
      value |= gpio_get(dataPin) << i;
    else
      value |= gpio_get(dataPin) << (7 - i);
    gpio_put(clockPin, 0);
    busy_wait_us(1);
  }
  return value;
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data, clock, order)                         \
  shiftInSlow(data, clock, order)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(data, clock, order)                         \
  shiftIn(data, clock, order)
#endif

#if ARCH_ESPRESSIF
// ESP8266 doesn't read values between 0x20000 and 0x30000 when DOUT is pulled
// up.
#define DOUT_MODE INPUT
#else
#define DOUT_MODE INPUT_PULLUP
#endif

HX711::HX711() {}

HX711::~HX711() {}

void HX711::begin(byte dout, byte pd_sck, byte gain) {
  PD_SCK = pd_sck;
  DOUT = dout;
  gpio_init(PD_SCK);
  gpio_set_dir(PD_SCK, GPIO_OUT);
  gpio_init(DOUT);
  gpio_set_dir(DOUT, GPIO_IN);

  // pinMode(PD_SCK, OUTPUT);
  // pinMode(DOUT, DOUT_MODE);

  set_gain(gain);
}

bool HX711::is_ready() { return gpio_get(DOUT) == 0; }

void HX711::set_gain(byte gain) {
  switch (gain) {
  case 128: // channel A, gain factor 128
    GAIN = 1;
    break;
  case 64: // channel A, gain factor 64
    GAIN = 3;
    break;
  case 32: // channel B, gain factor 32
    GAIN = 2;
    break;
  }
}

long HX711::read() {

  // Wait for the chip to become ready.
  if (!wait_ready_timeout(100, 10)) {
    return 0;
  }

  // Define structures for reading data into.
  unsigned long value = 0;
  uint8_t data[3] = {0};
  uint8_t filler = 0x00;

  // Protect the read sequence from system interrupts.  If an interrupt occurs
  // during the time the PD_SCK signal is high it will stretch the length of the
  // clock pulse. If the total pulse time exceeds 60 uSec this will cause the
  // HX711 to enter power down mode during the middle of the read sequence.
  // While the device will wake up when PD_SCK goes low again, the reset starts
  // a new conversion cycle which forces DOUT high until that cycle is
  // completed.
  //
  // The result is that all subsequent bits read by shiftIn() will read back as
  // 1, corrupting the value returned by read().  The ATOMIC_BLOCK macro
  // disables interrupts during the sequence and then restores the interrupt
  // mask to its previous state after the sequence completes, insuring that the
  // entire read-and-gain-set sequence is not interrupted.  The macro has a few
  // minor advantages over bracketing the sequence between `noInterrupts()` and
  // `interrupts()` calls.
  auto status = save_and_disable_interrupts();

  // Pulse the clock pin 24 times to read the data.
  data[2] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
  data[1] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
  data[0] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);

  // Set the channel and the gain factor for the next reading using the clock
  // pin.
  for (unsigned int i = 0; i < GAIN; i++) {
    gpio_put(PD_SCK, 1);

    busy_wait_us(1);

    gpio_put(PD_SCK, 0);

    busy_wait_us(1);
  }

  // Enable interrupts again.
  restore_interrupts(status);
  // Replicate the most significant bit to pad out a 32-bit signed integer
  if (data[2] & 0x80) {
    filler = 0xFF;
  } else {
    filler = 0x00;
  }

  // Construct a 32-bit signed integer
  value = (static_cast<unsigned long>(filler) << 24 |
           static_cast<unsigned long>(data[2]) << 16 |
           static_cast<unsigned long>(data[1]) << 8 |
           static_cast<unsigned long>(data[0]));

  return static_cast<long>(value);
}

void HX711::wait_ready(unsigned long delay_ms) {
  // Wait for the chip to become ready.
  // This is a blocking implementation and will
  // halt the sketch until a load cell is connected.
  while (!is_ready()) {
    // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on
    // ESP. https://github.com/bogde/HX711/issues/73
    sleep_ms(delay_ms);
  }
}

bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
  // Wait for the chip to become ready by
  // retrying for a specified amount of attempts.
  // https://github.com/bogde/HX711/issues/76
  int count = 0;
  while (count < retries) {
    if (is_ready()) {
      return true;
    }
    sleep_ms(delay_ms);
    count++;
  }
  return false;
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
  // Wait for the chip to become ready until timeout.
  // https://github.com/bogde/HX711/pull/96

  timeout *= 1000;
  auto microsStarted = time_us_32();
  while (time_us_32() - microsStarted < timeout) {
    if (is_ready()) {
      return true;
    }
    sleep_ms(delay_ms);
  }
  return false;
}

long HX711::read_average(byte times) {
  long sum = 0;
  for (byte i = 0; i < times; i++) {
    sum += read();
    // Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on
    // ESP. https://github.com/bogde/HX711/issues/73
    sleep_ms(0);
  }
  return sum / times;
}

double HX711::get_value(byte times) { return read_average(times) - OFFSET; }

float HX711::get_units(byte times) { return get_value(times) / SCALE; }

void HX711::tare(byte times) {
  double sum = read_average(times);
  set_offset(sum);
}

void HX711::set_scale(float scale) { SCALE = scale; }

float HX711::get_scale() { return SCALE; }

void HX711::set_offset(long offset) { OFFSET = offset; }

long HX711::get_offset() { return OFFSET; }

void HX711::power_down() {
  gpio_put(PD_SCK, 0);
  gpio_put(PD_SCK, 1);
}

void HX711::power_up() { gpio_put(PD_SCK, 0); }