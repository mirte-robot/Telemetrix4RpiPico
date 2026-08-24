
/********************************************************
 * Copyright (c) 2021 Alan Yorinks All rights reserved.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU AFFERO GENERAL PUBLIC LICENSE
 Version 3 as published by the Free Software Foundation; either
 or (at your option) any later version.
 This library is distributed in the hope that it will be useful,f
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU AFFERO GENERAL PUBLIC LICENSE
 along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

/******************** Attributions ***********************************
 * This file contains modifications of the work of others to support some
 * of the project's features.
 *
 * Neopixel support:
 *https://github.com/raspberrypi/pico-examples/tree/master/pio/ws2812
 *
 * DHT sensor support:
 *https://github.com/raspberrypi/pico-examples/tree/master/gpio/dht_sensor
 *
 *************************************************************************/
#include "module/Hiwonder_Servo.hpp"
#include "module/PCA9685_Module.hpp"
#include "module/Shutdown_Module.hpp"
#include "module/tmx_ssd1306_Module.hpp"

#include "drivers/neopixel.hpp"
#include "sensors/MPU6050_sensor.hpp"
#include "sensors/adxl345_sensor.hpp"
#include "sensors/as5600_sensor.hpp"
#include "sensors/bno055_sensor.hpp"
#include "sensors/gps_sensor.hpp"
#include "sensors/hmc5883l_sensor.hpp"
#include "sensors/hx711_sensor.hpp"
#include "sensors/ina226_sensor.hpp"
#include "sensors/mpu9250_sensor.hpp"
#include "sensors/veml6040_sensor.hpp"
#include "sensors/vl53l0x_sensor.hpp"

#include "Telemetrix4RpiPico.hpp"
#include "led_pin.hpp"
#include "mirte_master.hpp"
#include "sensors/sonar.hpp"
#include "serialization.hpp"
#include <functional>
/*******************************************************************
 *              GLOBAL VARIABLES, AND STORAGE
 ******************************************************************/

const auto ANALOG_OFFSET = 26;

// buffer to hold incoming command data
uint8_t command_buffer[MAX_COMMAND_LENGTH];

bool stop_reports = false; // a flag to stop sending all report messages

// an array of digital_pin_descriptors
pin_descriptor the_digital_pins[MAX_DIGITAL_PINS_SUPPORTED];

// an array of analog_pin_descriptors
analog_pin_descriptor the_analog_pins[MAX_ANALOG_PINS_SUPPORTED];

// number of active dht devices
int dht_count = -1;

// dht device descriptors
dht_data the_dhts = {.next_dht_index = 0, .dhts = {}};

// encoder device descriptors
encoder_data encoders = {
    .next_encoder_index = 0, .trigger_timer = {}, .encoders = {}, .mutex = {}};

// // pio for neopixel values
// PIO np_pio = pio0;
// uint np_sm = 0;

// // neopixel storage for up to 150 pixel string
// // Each entry contains an RGG array.

// uint8_t pixel_buffer[MAXIMUM_NUM_NEOPIXELS][3];

// uint actual_number_of_pixels;

// scan delay
uint32_t scan_delay = 100000;

// // TODO: remove this slow mess, use latest example with dma
// static inline void put_pixel(uint32_t pixel_grb) {
//   pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
// }

// static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
//   return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
// }

// PWM values
uint32_t top;

// for dht repeating read timer
struct repeating_timer timer;
volatile bool timer_fired = false;

/******************* REPORT BUFFERS *******************/
// NOTE First value in the array is the number of reporting
// data elements. It does not include itself in this count.

// buffer to hold data for the loop_back command
// The last element will be filled in by the loopback command
int loop_back_report_message[] = {2, (int)SERIAL_LOOP_BACK, 0};

// buffer to hold data for send_debug_info command
uint debug_info_report_message[] = {4, DEBUG_PRINT, 0, 0, 0};

// buffer to hold firmware version info
int firmware_report_message[] = {3, FIRMWARE_REPORT, FIRMWARE_MAJOR,
                                 FIRMWARE_MINOR};

// buffer to hold i2c report data
int i2c_report_message[64];

// buffer to hold spi report data
int spi_report_message[64];

// get_pico_unique_id report buffer
int unique_id_report_report_message[] = {
    9, REPORT_PICO_UNIQUE_ID, 0, 0, 0, 0, 0, 0, 0, 0};
// digital input report buffer
int digital_input_report_message[] = {3, DIGITAL_REPORT, 0, 0};

// analog input report message
int analog_input_report_message[] = {4, ANALOG_REPORT, 0, 0, 0};

// dht report message
int dht_report_message[] = {
    6, DHT_REPORT, 0, 0, 0, 0, 0,
};

/*****************************************************************
 *                   THE COMMAND TABLE
 When adding a new command update the command_table.
 The command length is the number of bytes that follow
 the command byte itself, and does not include the command
 byte in its length.
 The command_func is a pointer the command's function.
 ****************************************************************/
// An array of pointers to the command functions
template <typename V, typename... T>
constexpr auto array_of(T &&...t) -> std::array<V, sizeof...(T)> {
  return {{std::forward<T>(t)...}};
}

constexpr auto command_table = array_of<command_descriptor>(
    &serial_loopback, &set_pin_mode, &digital_write, &pwm_write,
    &modify_reporting, &get_firmware_version, &get_pico_unique_id,
    &servo_attach, &servo_write, &servo_detach, &i2c_begin, &i2c_read,
    &i2c_write, &sonar_new, &dht_new, &stop_all_reports, &enable_all_reports,
    &reset_data, &reset_board, &init_neo_pixels, &show_neo_pixels,
    &set_neo_pixel, &clear_all_neo_pixels, &fill_neo_pixels, &init_spi,
    &write_blocking_spi, &read_blocking_spi, &set_format_spi, &spi_cs_control,
    &set_scan_delay, &encoder_new, &sensor_new, &ping, &module_new,
    &module_data, &get_id, &set_id, &feature_detect, &reset_to_bootloader);

/***************************************************************************
 *                   DEBUGGING FUNCTIONS
 **************************************************************************/

/************************************************************
 * Loop back the received character
 */
void serial_loopback() {
  loop_back_report_message[LOOP_BACK_DATA] = command_buffer[DATA_TO_LOOP_BACK];
  serial_write(loop_back_report_message,
               sizeof(loop_back_report_message) / sizeof(int));
}

/******************************************************************
 * Send debug info report
 * @param id: 8 bit value
 * @param value: 16 bit value
 */
// A method to send debug data across the serial link
void send_debug_info(uint id, uint value) {
  auto msg = std::span(debug_info_report_message);
  debug_info_report_message[DEBUG_ID] = id;
  std::copy_n(encode_u16(value).cbegin(), sizeof(uint16_t),
              msg.subspan<DEBUG_VALUE_HIGH_BYTE, sizeof(uint16_t)>().begin());

  // Assert are OK; Removed for speed;
  // assert(debug_info_report_message[DEBUG_VALUE_HIGH_BYTE] ==
  //        (value & 0xFF00) >> 8);
  // assert(debug_info_report_message[DEBUG_VALUE_LOW_BYTE] ==
  //        (value & 0x00FF));

  serial_write((int *)debug_info_report_message,
               sizeof(debug_info_report_message) / sizeof(int));
}

/*******************************************************************************
 *                  COMMAND FUNCTIONS
 ******************************************************************************/

/************************************************************************
 * Set a Pins mode
 */
void set_pin_mode() {
  uint pin;
  PIN_MODES mode;
  pin = command_buffer[SET_PIN_MODE_GPIO_PIN];
  mode = (PIN_MODES)command_buffer[SET_PIN_MODE_MODE_TYPE];

  switch (mode) {
  case PIN_MODES::INPUT:
  case PIN_MODES::INPUT_PULL_UP:
  case PIN_MODES::INPUT_PULL_DOWN:
    the_digital_pins[pin].pin_mode = mode;
    the_digital_pins[pin].reporting_enabled =
        command_buffer[SET_PIN_MODE_DIGITAL_IN_REPORTING_STATE];
    the_digital_pins[pin].last_value = 0xFF;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (mode == INPUT_PULL_UP) {
      gpio_pull_up(pin);
    }
    if (mode == INPUT_PULL_DOWN) {
      gpio_pull_down(pin);
    }
    break;
  case PIN_MODES::OUTPUT:
    the_digital_pins[pin].pin_mode = mode;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    break;
  case PIN_MODES::PWM: {
    /* Here we will set the operating frequency to be 50 hz to
       simplify support PWM as well as servo support.
    */
    the_digital_pins[pin].pin_mode = mode;

    const uint32_t f_hz = 50; // frequency in hz.

    uint slice_num = pwm_gpio_to_slice_num(pin); // get PWM slice for the pin

    // set frequency
    // determine top given Hz using the free-running clock
    uint32_t f_sys = clock_get_hz(clk_sys);
    float divider = (float)(f_sys / 1'000'000UL); // run the pwm clock at 1MHz
    pwm_set_clkdiv(slice_num,
                   divider); // pwm clock should now be running at 1MHz
    const auto top_v = 1'000'000UL / f_hz - 1; // calculate the TOP value
    top = top_v;
    pwm_set_wrap(slice_num, (uint16_t)top);
    // set the current level to 0
    pwm_set_gpio_level(pin, 0);

    pwm_set_enabled(slice_num, true); // let's go!
    gpio_set_function(pin, GPIO_FUNC_PWM);
    static_assert(top_v == 19999, "Top is not 19999");
    break;
  }
  case PIN_MODES::ANALOG_INPUT: {
    auto analog_pin = pin - ANALOG_OFFSET; // convert pin number to analog pin
    // if the temp sensor was selected, then turn it on
    if (analog_pin == ADC_TEMPERATURE_REGISTER) {
      adc_set_temp_sensor_enabled(true);
    }
    the_analog_pins[analog_pin].reporting_enabled =
        command_buffer[SET_PIN_MODE_ANALOG_IN_REPORTING_STATE];
    // save the differential value
    the_analog_pins[analog_pin].differential = std::max<int>(
        1,
        decode_u16(
            std::span(command_buffer)
                .subspan<SET_PIN_MODE_ANALOG_DIFF_HIGH, sizeof(uint16_t)>()));

  } break;
  default:
    break;
  }
}

/**********************************************************
 * Set a digital output pin's value
 */
void digital_write() {
  uint pin;
  uint value;
  pin = command_buffer[DIGITAL_WRITE_GPIO_PIN];
  value = command_buffer[DIGITAL_WRITE_VALUE];
  // special case for led pin:
  if (pin == 200) {
    set_led_pin(value);
    return;
  }
  gpio_put(pin, (bool)value);
}

/**********************************************
 * Set A PWM Pin's value
 */
void pwm_write() {
  auto data = std::span(command_buffer);
  uint pin;
  uint16_t value;
  auto msg_count = (packet_size - 1) / 3;
  for (int i = 0; i < msg_count; i++) {
    auto offset = i * 3;
    pin = command_buffer[offset + PWM_WRITE_GPIO_PIN];

    value = decode_u16(std::span<uint8_t, sizeof(uint16_t)>(
        data.data() + offset + SET_PIN_MODE_PWM_HIGH_VALUE, sizeof(uint16_t)));
    if (value == 0 || value >= top) {
      // for the common case of fully on or off, just set the pin level to avoid
      // any issues with the wrap around
      if (the_digital_pins[pin].pin_mode != PIN_MODES::PWM) {
        // if pwm, then still fall thru to normal pwm mode, otherwise (digital),
        // then write like digital.
        gpio_put(pin, value >= top);
        continue;
      }
    }
    pwm_set_gpio_level(pin, value);
  }
}

/***************************************************
 *  Control reporting
 */
void modify_reporting() {
  int pin = command_buffer[MODIFY_REPORTING_PIN];

  switch (command_buffer[MODIFY_REPORTING_TYPE]) {
  case REPORTING_DISABLE_ALL:
    for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
      the_digital_pins[i].reporting_enabled = false;
    }
    for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
      the_analog_pins[i].reporting_enabled = false;
    }
    break;
  case REPORTING_ANALOG_ENABLE:
    the_analog_pins[pin - ANALOG_OFFSET].reporting_enabled = true;
    break;
  case REPORTING_ANALOG_DISABLE:
    the_analog_pins[pin - ANALOG_OFFSET].reporting_enabled = false;
    break;
  case REPORTING_DIGITAL_ENABLE:
    if (the_digital_pins[pin - ANALOG_OFFSET].pin_mode != PIN_MODE_NOT_SET) {
      the_digital_pins[pin - ANALOG_OFFSET].reporting_enabled = true;
    }
    break;
  case REPORTING_DIGITAL_DISABLE:
    if (the_digital_pins[pin - ANALOG_OFFSET].pin_mode != PIN_MODE_NOT_SET) {
      the_digital_pins[pin - ANALOG_OFFSET].reporting_enabled = false;
    }
    break;
  default:
    break;
  }
}

/***********************************************************************
 * Retrieve the current firmware version
 */
void get_firmware_version() {
  serial_write(firmware_report_message,
               sizeof(firmware_report_message) / sizeof(int));
}

/**************************************************************
 * Retrieve the Pico's Unique ID
 */
void get_pico_unique_id() {
  // get the unique id
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);

  unique_id_report_report_message[2] = (board_id.id[0]);
  unique_id_report_report_message[3] = (board_id.id[1]);
  unique_id_report_report_message[4] = (board_id.id[2]);
  unique_id_report_report_message[5] = (board_id.id[3]);
  unique_id_report_report_message[6] = (board_id.id[4]);
  unique_id_report_report_message[7] = (board_id.id[5]);

  serial_write(unique_id_report_report_message, 10);
}

/********************************************
 * Stop reporting for all input pins
 */
void stop_all_reports() {
  stop_reports = true;
  sleep_ms(20);
  stdio_flush();
}

/**********************************************
 * Enable reporting for all input pins
 */
void enable_all_reports() {
  stdio_flush();
  stop_reports = false;
  sleep_ms(20);
}

/******************************************
 * Use the watchdog time to reset the board.
 */
void reset_board() {
  reset_hardware();
  watchdog_reboot(0, 0, 0);
  watchdog_enable(10, 1);
}

void i2c_begin() {
  // get the GPIO pins associated with this i2c instance
  uint sda_gpio = command_buffer[I2C_SDA_GPIO_PIN];
  uint scl_gpio = command_buffer[I2C_SCL_GPIO_PIN];
  uint8_t i2c_port = command_buffer[I2C_PORT];
  if (i2c_port == sda_gpio && i2c_port == scl_gpio) {
    // if the port is the same as the GPIO pins, then we are targeted as a raw
    // device, so just use the default i2c pins
    if (i2c_port == 0) {
      sda_gpio = 4;
      scl_gpio = 5;
    } else if (i2c_port == 1) {
      sda_gpio = 10;
      scl_gpio = 11;
    } else {
      // invalid port, so return
      return;
    }
  }
  reset_i2c(scl_gpio, sda_gpio, command_buffer[I2C_PORT]);
  // // set the i2c instance - 0 or 1
  // if (command_buffer[I2C_PORT] == 0) {
  //   i2c_init(i2c0, 100 * 1000);
  // } else {
  //   i2c_init(i2c1, 100 * 1000);
  // }
  // gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
  // gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
  // gpio_pull_up(sda_gpio);
  // gpio_pull_up(scl_gpio);
}

void i2c_read() {

  // The report_message offsets:
  // 0 = packet length - this must be calculated
  // 1 = I2C_READ_REPORT
  // 2 = The i2c port - 0 or 1
  // 3 = i2c device address
  // 4 = message_id
  // 5 = i2c read register
  // 6 = number of bytes read
  // 7... = bytes read

  // length of i2c report packet
  int num_of_bytes_to_send =
      I2C_READ_START_OF_DATA + command_buffer[I2C_READ_LENGTH];

  // We have a separate buffer ot store the data read from the device
  // and combine that data back into the i2c report buffer.
  // This gets around casting.
  // uint8_t data_from_device[command_buffer[I2C_READ_LENGTH]];
  std::vector<uint8_t> data_from_device(command_buffer[I2C_READ_LENGTH], 0);
  uint8_t message_id = command_buffer[I2C_READ_MESSAGE_ID];
  // return value from write and read i2c sdk commands
  int i2c_sdk_call_return_value;

  // selector for i2c0 or i2c1
  i2c_inst_t *i2c;

  // Determine the i2c port to use.
  if (command_buffer[I2C_PORT]) {
    i2c = i2c1;
  } else {
    i2c = i2c0;
  }

  // If there is an i2c register specified, set the register pointer
  if (command_buffer[I2C_READ_NO_STOP_FLAG] != I2C_NO_REGISTER_SPECIFIED) {
    i2c_sdk_call_return_value =
        i2c_write_blocking(i2c, (uint8_t)command_buffer[I2C_DEVICE_ADDRESS],
                           (const uint8_t *)&command_buffer[I2C_READ_REGISTER],
                           1, (bool)command_buffer[I2C_READ_NO_STOP_FLAG]);
    if (i2c_sdk_call_return_value == PICO_ERROR_GENERIC) {
      return;
    }
  }

  // now do the read request
  i2c_sdk_call_return_value = i2c_read_blocking(
      i2c, (uint8_t)command_buffer[I2C_DEVICE_ADDRESS], data_from_device.data(),
      (size_t)(command_buffer[I2C_READ_LENGTH]),
      (bool)command_buffer[I2C_READ_NO_STOP_FLAG]);
  if (i2c_sdk_call_return_value == PICO_ERROR_GENERIC) {
    i2c_report_message[I2C_PACKET_LENGTH] =
        I2C_ERROR_REPORT_LENGTH;                         // length of the packet
    i2c_report_message[I2C_REPORT_ID] = I2C_READ_FAILED; // report ID
    i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];
    i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] =
        command_buffer[I2C_DEVICE_ADDRESS];

    serial_write(i2c_report_message, I2C_ERROR_REPORT_NUM_OF_BYTE_TO_SEND);
    return;
  }

  // copy the data returned from i2c device into the report message buffer
  for (int i = 0; i < i2c_sdk_call_return_value; i++) {
    i2c_report_message[i + I2C_READ_START_OF_DATA] = data_from_device[i];
  }
  // length of the packet
  i2c_report_message[I2C_PACKET_LENGTH] =
      (uint8_t)(i2c_sdk_call_return_value + I2C_READ_DATA_BASE_BYTES);

  i2c_report_message[I2C_REPORT_ID] = I2C_READ_REPORT;

  // i2c_port
  i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];

  // i2c_address
  i2c_report_message[I2C_REPORT_DEVICE_ADDRESS] =
      command_buffer[I2C_DEVICE_ADDRESS];

  // i2c register
  i2c_report_message[I2C_REPORT_READ_REGISTER] =
      command_buffer[I2C_READ_REGISTER];

  // number of bytes read from i2c device
  i2c_report_message[I2C_REPORT_READ_NUMBER_DATA_BYTES] =
      (uint8_t)i2c_sdk_call_return_value;

  i2c_report_message[I2C_READ_MESSAGE_ID] = message_id;

  serial_write((int *)i2c_report_message, num_of_bytes_to_send);
}

void i2c_write() {
  // i2c instance pointer
  i2c_inst_t *i2c;

  // Determine the i2c port to use.
  if (command_buffer[I2C_PORT]) {
    i2c = i2c1;
  } else {
    i2c = i2c0;
  }

  int i2c_sdk_call_return_value = i2c_write_blocking_until(
      i2c, (uint8_t)command_buffer[I2C_DEVICE_ADDRESS],
      &(command_buffer[I2C_WRITE_BYTES_TO_WRITE]),
      command_buffer[I2C_WRITE_NUMBER_OF_BYTES],
      (bool)command_buffer[I2C_WRITE_NO_STOP_FLAG], make_timeout_time_ms(50));

  i2c_report_message[I2C_PACKET_LENGTH] = 4; // length of the packet
  i2c_report_message[I2C_REPORT_ID] =
      I2C_WRITE_REPORT; // bytes written or error
  i2c_report_message[I2C_REPORT_PORT] = command_buffer[I2C_PORT];
  i2c_report_message[I2C_WRITE_MESSAGE_ID] =
      command_buffer[I2C_WRITE_MESSAGE_ID];

  i2c_report_message[4] = i2c_sdk_call_return_value;

  serial_write(i2c_report_message, I2C_ERROR_REPORT_NUM_OF_BYTE_TO_SEND);
}
TMX_NeoPixel np;
bool np_initialized = false;
void init_neo_pixels() {
  np.init(command_buffer[NP_NUMBER_OF_PIXELS], command_buffer[NP_PIN_NUMBER],
          command_buffer[NP_RED_FILL], command_buffer[NP_GREEN_FILL],
          command_buffer[NP_BLUE_FILL]);
  np_initialized = true;
}

void set_neo_pixel() {
  if (!np_initialized) {
    return;
  }
  np.setPixelColor(command_buffer[NP_PIXEL_NUMBER], command_buffer[NP_SET_RED],
                   command_buffer[NP_SET_GREEN], command_buffer[NP_SET_BLUE],
                   command_buffer[NP_SET_AUTO_SHOW]);
}

void show_neo_pixels() {
  if (!np_initialized) {
    return;
  }
  np.show();
}

void clear_all_neo_pixels() {
  if (!np_initialized) {
    return;
  }
  // set all the neopixels in the buffer to all zeroes
  np.clear();
}

void fill_neo_pixels() {
  if (!np_initialized) {
    return;
  }
  np.fill(command_buffer[NP_FILL_RED], command_buffer[NP_FILL_GREEN],
          command_buffer[NP_FILL_BLUE], command_buffer[NP_FILL_AUTO_SHOW]);
}

void reset_neo_pixels() {
  if (!np_initialized) {
    return;
  }
  np.clear();
}

void init_quadrature_encoder(int A, int B, encoder_t *enc, bool pullup = false,
                             bool pulldown = false) {
  gpio_init(A);
  gpio_set_dir(A, GPIO_IN);
  gpio_set_pulls(A, pullup, pulldown);
  gpio_init(B);
  gpio_set_dir(B, GPIO_IN);
  gpio_set_pulls(B, pullup, pulldown);

  enc->A = A;
  enc->B = B;
  enc->type = QUADRATURE;
}
void init_single_encoder(int A, encoder_t *enc, bool pullup = false,
                         bool pulldown = false) {
  gpio_init(A);
  gpio_set_dir(A, GPIO_IN);
  gpio_set_pulls(A, pullup, pulldown);

  enc->A = A;
  enc->type = SINGLE;
}

bool encoder_callback(repeating_timer_t *timer) {
  (void)timer;
  if (!mutex_try_enter(&encoders.mutex, NULL)) {
    return true;
  }

  if (encoders.next_encoder_index == 0) {
    mutex_exit(&encoders.mutex);
    return true;
  }
  for (int i = 0; i < encoders.next_encoder_index; i++) {
    encoder_t *enc = &encoders.encoders[i];
    if (enc->type == QUADRATURE) {
      bool a = gpio_get(enc->A);

      bool b = gpio_get(enc->B);

      int new_gray_code_step = a << 1 | b;
      int new_bin_step = new_gray_code_step >> 1 ^ new_gray_code_step;
      int diff = new_bin_step - enc->last_state;
      if (diff > -2 && diff < 2) {
        enc->step += diff;
      } else {
        enc->step += (diff < 0) ? 1 : -1;
      }
      enc->last_state = new_bin_step;
    } else {
      bool a = gpio_get(enc->A);
      if (a != enc->last_state) {
        if (time_us_32() - enc->last_time > 1000) { // debounce time 1ms
          enc->step++;
          enc->last_time = time_us_32();
          enc->last_state = a;
        }
      }
    }
  }
  mutex_exit(&encoders.mutex);
  return true;
}

bool create_encoder_timer() {
  int hz = 10'000;
  /* blue encoder motor:
  - 110 rpm = ~2 rot/s
  - 540 steps/rot
  - >1000 steps/s
  - requires at least 1 scan per step

  Mirte-master:
  107 rpm, 1320 ticks/rot
  -> ~1.8 rot/s * 1320 = 2354 ticks/s
*/
  if (!add_repeating_timer_us(1'000'000 / hz, encoder_callback, NULL,
                              &encoders.trigger_timer)) {
    // printf("Failed to add timer\n");
    return false;
  }
  return true;
}

void encoder_new() {

  ENCODER_TYPES type = (ENCODER_TYPES)command_buffer[ENCODER_TYPE];
  uint pin_a = command_buffer[ENCODER_PIN_A];
  uint pin_b = command_buffer[ENCODER_PIN_B]; // both cases will have a pin B
  // set to 1 if pullup, 2 if pulldown, 0 if nothing.
  auto pulls = command_buffer[ENCODER_PULLS];

  if (encoders.next_encoder_index == 0) {
    mutex_init(&encoders.mutex);
    bool timer = create_encoder_timer();
    if (!timer) {
      return;
    }
  } else if (encoders.next_encoder_index > MAX_ENCODERS) {
    return;
  }
  mutex_enter_blocking(&encoders.mutex);

  encoder_t *new_encoder = &encoders.encoders[encoders.next_encoder_index];
  if (type == SINGLE) {
    init_single_encoder(pin_a, new_encoder, pulls & 1, pulls & 2);
  } else {
    init_quadrature_encoder(pin_a, pin_b, new_encoder, pulls & 1, pulls & 2);
  }
  encoders.next_encoder_index++;
  mutex_exit(&encoders.mutex);
}

int encoder_report_message[] = {3, ENCODER_REPORT, 0, 0};

void scan_encoders() {
  if (encoders.next_encoder_index < 1) {
    return;
  }
  if (!mutex_try_enter(&encoders.mutex, NULL)) {
    return;
  }
  for (int i = 0; i < encoders.next_encoder_index; i++) {
    encoder_t *enc = &encoders.encoders[i];
    if (enc->step != 0) {
      encoder_report_message[ENCODER_REPORT_PIN_A] = (uint8_t)enc->A;
      encoder_report_message[ENCODER_REPORT_STEP] = enc->step;
      enc->step = 0;
      serial_write(encoder_report_message, 4);
    }
  }
  mutex_exit(&encoders.mutex);
}

bool repeating_timer_callback(struct repeating_timer *t) {
  (void)t;
  // printf("Repeat at %lld\n", time_us_64());
  timer_fired = true;
  return true;
}

void dht_new() {
  if (dht_count > MAX_DHTS) {
    return;
  }
  if (dht_count == -1) {
    // first time through start repeating timer
    add_repeating_timer_ms(2000, repeating_timer_callback, NULL, &timer);
  }
  dht_count++;

  uint dht_pin = command_buffer[DHT_DATA_PIN];
  the_dhts.dhts[dht_count].data_pin = dht_pin;
  the_dhts.dhts[dht_count].previous_time = get_absolute_time();
  gpio_init(dht_pin);
}

void init_spi() {
  spi_inst_t *spi_port;
  uint spi_baud_rate;
  uint cs_pin;

  // initialize the spi port
  if (command_buffer[SPI_PORT] == 0) {
    spi_port = spi0;
  } else {
    spi_port = spi1;
  }

  spi_baud_rate = decode_u32(
      std::span(command_buffer).subspan<SPI_FREQ_MSB, sizeof(uint32_t)>());

  spi_init(spi_port, spi_baud_rate);

  // set gpio pins for miso, mosi and clock
  gpio_set_function(command_buffer[SPI_MISO], GPIO_FUNC_SPI);
  gpio_set_function(command_buffer[SPI_MOSI], GPIO_FUNC_SPI);
  gpio_set_function(command_buffer[SPI_CLK_PIN], GPIO_FUNC_SPI);

  // initialize chip select GPIO pins
  for (int i = 0; i < command_buffer[SPI_CS_LIST_LENGTH]; i++) {
    cs_pin = command_buffer[SPI_CS_LIST + i];
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
  }
}

void spi_cs_control() {
  uint8_t cs_pin;
  uint8_t cs_state;

  cs_pin = command_buffer[SPI_SELECT_PIN];
  cs_state = command_buffer[SPI_SELECT_STATE];
  asm volatile("nop \n nop \n nop");
  gpio_put(cs_pin, cs_state);
  asm volatile("nop \n nop \n nop");
}

void set_scan_delay() {
  scan_delay = ((uint32_t)command_buffer[SCAN_DELAY]) * 1000;
}

void read_blocking_spi() {
  // The report_message offsets:
  // 0 = packet length - this must be calculated
  // 1 = SPI_READ_REPORT
  // 2 = The i2c port - 0 or 1
  // 3 = number of bytes read
  // 4... = bytes read

  spi_inst_t *spi_port;
  size_t data_length;
  uint8_t repeated_transmit_byte;
  std::vector<uint8_t> data;
  data.resize(command_buffer[SPI_READ_LEN]);
  std::fill(data.begin(), data.end(), 0);
  // uint8_t data[command_buffer[SPI_READ_LEN]];

  if (command_buffer[SPI_PORT] == 0) {
    spi_port = spi0;
  } else {
    spi_port = spi1;
  }

  data_length = command_buffer[SPI_READ_LEN];
  // memset(data, 0, data_length);
  // memset(data, 0, sizeof(data));

  repeated_transmit_byte = command_buffer[SPI_REPEATED_DATA];

  // read data
  spi_read_blocking(spi_port, repeated_transmit_byte, data.data(), data_length);
  sleep_ms(100);

  // build a report from the data returned
  spi_report_message[SPI_PACKET_LENGTH] =
      SPI_REPORT_NUMBER_OF_DATA_BYTES + data_length;
  spi_report_message[SPI_REPORT_ID] = SPI_REPORT;
  spi_report_message[SPI_REPORT_PORT] = command_buffer[SPI_PORT];
  spi_report_message[SPI_REPORT_NUMBER_OF_DATA_BYTES] = data_length;
  for (size_t i = 0; i < data_length; i++) {
    spi_report_message[SPI_DATA + i] = data[i];
  }
  serial_write((int *)spi_report_message, SPI_DATA + data_length);
}

void write_blocking_spi() {
  spi_inst_t *spi_port;
  // uint cs_pin;
  size_t data_length;

  if (command_buffer[SPI_PORT] == 0) {
    spi_port = spi0;
  } else {
    spi_port = spi1;
  }

  data_length = command_buffer[SPI_WRITE_LEN];
  // write data
  spi_write_blocking(spi_port, &command_buffer[SPI_WRITE_DATA], data_length);
}

void set_format_spi() {
  spi_inst_t *spi_port;
  uint data_bits = command_buffer[SPI_NUMBER_OF_BITS];
  spi_cpol_t cpol = (spi_cpol_t)command_buffer[SPI_CLOCK_PHASE];
  spi_cpha_t cpha = (spi_cpha_t)command_buffer[SPI_CLOCK_POLARITY];

  if (command_buffer[SPI_PORT] == 0) {
    spi_port = spi0;
  } else {
    spi_port = spi1;
  }
  spi_set_format(spi_port, data_bits, cpol, cpha, (spi_order_t)1);
}

/******************* FOR FUTURE RELEASES **********************/

void reset_data() {}

/***************** Currently Unused ***************************/
void servo_attach() {

  auto pin = command_buffer[1];
  // auto min_pulse = command_buffer[
  command_buffer[SET_PIN_MODE_GPIO_PIN] = pin;
  command_buffer[SET_PIN_MODE_MODE_TYPE] = PIN_MODES::PWM;
  set_pin_mode();
}

void servo_write() {
  // get microticks value
  const uint32_t f_hz = 50; // frequency in hz.

  auto pin = command_buffer[1];
  uint16_t ticks_ms =
      decode_u16(std::span(command_buffer).subspan<2, sizeof(uint16_t)>());

  const uint32_t top = 1'000'000UL / f_hz - 1; // calculate the TOP value

  uint16_t value = (ticks_ms * top) / 20'000UL;
  pwm_set_gpio_level(pin, value);
}

void servo_detach() {
  // TODO: implement
}

/******************************************************
 *             INTERNALLY USED FUNCTIONS
 *****************************************************/

/***************************************************
 * Retrieve the next command and process it
 */

// rewrite of get_next_command without any waiting states, use command buffer
// and index to read into buffer until packet size bytes are received

int curr_packet_index = -1;
int packet_size; // to get the size of the packets in module_new
auto last_cmd_byte = time_us_32();
void get_next_command() {
  auto byte = stdio_getchar_timeout_us(0);
  if (byte == PICO_ERROR_TIMEOUT || byte > 255) {
    if (curr_packet_index != -1 && time_us_32() - last_cmd_byte > 100000) {
      // if it's been more than 100ms since the last byte, then reset the packet
      // index to start fresh
      curr_packet_index = -1;
      last_cmd_byte = time_us_32();
    }

    return;
  }
  last_cmd_byte = time_us_32();
  if (curr_packet_index == -1) {
    // received packet size part of the message

    if (byte > MAX_COMMAND_LENGTH || byte == 0) {
      return;
    }
    packet_size = byte;
    if (packet_size == 0) {
      return;
    }
#if 0
    static bool led_state = false;
    led_state = !led_state;
    set_led_pin(led_state);
#endif
    // gpio_put(LED_PIN,
    //          !gpio_get(LED_PIN)); // toggle the led state for every packet

  } else {
    // data part of the message
    command_buffer[curr_packet_index] = (uint8_t)byte;
  }

  curr_packet_index++; // next byte of the message for next loop
  if (packet_size == curr_packet_index) {
    if (curr_packet_index < 1) {
      // if we have received a full packet, but the packet size is less than 1,
      // then we have a problem with the data and should just return and wait
      // for the next command
      return;
    }
    command_descriptor command_entry;
    command_entry = command_table[command_buffer[0]];
    command_entry.command_func();

    // reset everything
    packet_size = 0;
    curr_packet_index = -1;
  }
  if (curr_packet_index > packet_size) {
    curr_packet_index = -1;
  }
}

// // int packet_size; // to get the size of the packets in module_new
// void get_next_command_old() {
//   int packet_data;
//   command_descriptor command_entry;

//   // clear the command buffer for the new incoming command
//   // memset(command_buffer, 0, sizeof(command_buffer));
//   if(uart_get_hw(UART_ID)->fr&UART_UARTFR_RXFF_BITS) {
//     send_debug_info(66,66);
//   }

//   // Get the number of bytes of the command packet.
//   // The first byte is the command ID and the following bytes
//   // are the associated data bytes
//   packet_size = get_byte();
//   if(packet_size > MAX_COMMAND_LENGTH) {
//     // if the packet size is larger than the max command size, then we have a
//     problem
//     // with the data and should just return and wait for the next command
//     return;
//   }
//   if (packet_size == PICO_ERROR_TIMEOUT) {
//     // no data, let the main loop continue to run to handle inputs
//     return;
//   } else {
//     // get the rest of the packet
//     for (int i = 0; i < packet_size; i++) {
//       for (int retries = 10; retries > 0; retries--) {
//         packet_data = get_byte();

//         if (packet_data != PICO_ERROR_TIMEOUT) {
//           break;
//         }
//         sleep_ms(1);
//             gpio_put(LED_PIN, !gpio_get(LED_PIN)); // toggle the led state

//       }
//       if (packet_data == PICO_ERROR_TIMEOUT) {
//         return; // failed message
//       }
//       command_buffer[i] = (uint8_t)packet_data;
//     }

//     // the first byte is the command ID.
//     // look up the function and execute it.
//     // data for the command starts at index 1 in the command_buffer
//     command_entry = command_table[command_buffer[0]];

//     // uncomment to see the command and first byte of data
//     // send_debug_info(command_buffer[0], command_buffer[1]);

//     // call the command

//     command_entry.command_func();
//   }
// }

/**************************************
 * Scan all pins set as digital inputs
 * and generate a report.
 */
void scan_digital_inputs() {
  int value;

  // report message

  // index 0 = packet length
  // index 1 = report type
  // index 2 = pin number
  // index 3 = value

  for (int i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
    if (the_digital_pins[i].pin_mode == PIN_MODES::INPUT ||
        the_digital_pins[i].pin_mode == PIN_MODES::INPUT_PULL_UP ||
        the_digital_pins[i].pin_mode == PIN_MODES::INPUT_PULL_DOWN) {
      if (the_digital_pins[i].reporting_enabled) {
        // if the value changed since last read
        value = gpio_get(the_digital_pins[i].pin_number);
        if (value != the_digital_pins[i].last_value) {
          the_digital_pins[i].last_value = value;
          digital_input_report_message[DIGITAL_INPUT_GPIO_PIN] = i;
          digital_input_report_message[DIGITAL_INPUT_GPIO_VALUE] = value;
          serial_write(digital_input_report_message, 4);
        }
      }
    }
  }
}

void scan_analog_inputs() {
  uint16_t value;

  // report message

  // byte 0 = packet length
  // byte 1 = report type
  // byte 2 = pin number
  // byte 3 = high order byte of value
  // byte 4 = low order byte of value

  int differential;

  for (uint8_t i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
    if (the_analog_pins[i].reporting_enabled) { // TODO: move numbering to pin
                                                // number for reporting
      adc_select_input(i);
      value = adc_read();
      differential = abs(value - the_analog_pins[i].last_value);
      if (differential >= the_analog_pins[i].differential) {
        auto msg_span = std::span(analog_input_report_message);
        // trigger value achieved, send out the report
        the_analog_pins[i].last_value = value;
        analog_input_report_message[ANALOG_INPUT_GPIO_PIN] =
            (uint8_t)i + ANALOG_OFFSET;
        std::copy_n(encode_u16(value).cbegin(), sizeof(uint16_t),
                    msg_span.subspan<ANALOG_VALUE_HIGH_BYTE, sizeof(uint16_t)>()
                        .begin());

        // Asserts check out; Removed for performance
        // assert(analog_input_report_message[ANALOG_VALUE_HIGH_BYTE] == (value
        // &
        //        0xFF00) >> 8);
        // assert(analog_input_report_message[ANALOG_VALUE_LOW_BYTE] == value &
        //        0x00FF);
        serial_write(analog_input_report_message, 5);
      }
    }
  }
}

bool watchdog_enabled = false;
uint32_t last_ping = 0;
void ping() {
  static uint8_t random = -1;

  auto special_num = command_buffer[1];
  if (!watchdog_enabled) {
    watchdog_enable(WATCHDOG_TIME,
                    1); // Add watchdog requiring trigger every 5s
    watchdog_enabled = true;
    srand(time_us_32());
    random = rand() % 100; // create some random number to let computer side
                           // know it is the same run
  }
  std::vector<uint8_t> out = {0,           // write len
                              PONG_REPORT, // write type
                              special_num, random};
  out[0] = out.size() - 1; // dont count the packet length
  serial_write(out);
  if (watchdog_enable_shutdown) {
    watchdog_update();
    last_ping = time_us_32();
  }
}

const auto wd_timeout_time = WATCHDOG_TIME * 4000 / 5;

bool timeout_safe() { return time_us_32() - last_ping < wd_timeout_time / 2; }

void check_wd_timeout() {
  // if watchdog is about to run out of time, reset modules
  if (time_us_32() - last_ping >= (wd_timeout_time)) {
    reset_hardware();
  }
}

void reset_hardware() {
  for (auto &sensor : sensors) {
    sensor->resetSensor();
  }
  for (auto &module : modules) {
    module->resetModule();
  }
  reset_neo_pixels();
}
// SENSORSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS

auto sensor_funcs = std::vector<std::pair<
    SENSOR_TYPES, std::function<Sensor *(uint8_t[SENSORS_MAX_SETTINGS_A])>>>{
    {SENSOR_TYPES::VEML6040,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new VEML6040_Sensor(data);
     }},
    {SENSOR_TYPES::TOF_VL53,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new VL53L0X_Sensor(data);
     }},
    {SENSOR_TYPES::MPU_9250,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new MPU9250_Sensor(data);
     }},
    {SENSOR_TYPES::LOAD_CELL,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new HX711_Sensor(data);
     }},
    {SENSOR_TYPES::INA226a,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new INA226_Sensor(data);
     }},
    {SENSOR_TYPES::HMC5883l,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new HMC5883L_Sensor(data);
     }},
    {SENSOR_TYPES::AS5600_t,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new AS5600_Sensor(data);
     }},
    {SENSOR_TYPES::MPU6050_t,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new MPU6050_Module(data);
     }},
    {SENSOR_TYPES::BNO055_t,
     [](uint8_t data[SENSORS_MAX_SETTINGS_A]) {
       return new BNO055_Sensor(data);
     }},
};

void sensor_new() {

  const uint8_t sensor_cmd = command_buffer[1];
  if (sensor_cmd == 0) { // feature detection
    const SENSOR_TYPES type = (SENSOR_TYPES)command_buffer[2];
    bool found = false;
    for (const auto &sensor_f : sensor_funcs) {
      if (sensor_f.first == type) {
        found = true;
        break;
      }
    }
    serial_write({0, // packet length
                  SENSOR_MAIN_REPORT,
                  0, // feature check
                  (uint8_t)type, (uint8_t)(found ? 1u : 0u)});

  } else if (sensor_cmd == 1) {

    const SENSOR_TYPES type = (SENSOR_TYPES)command_buffer[3];
    const uint8_t sensor_num = command_buffer[2];

    uint8_t sensor_data[SENSORS_MAX_SETTINGS_A];
    std::copy(command_buffer + 4, command_buffer + 4 + SENSORS_MAX_SETTINGS_A,
              sensor_data);
    if (type >= SENSOR_TYPES::MAX_SENSORS) {
      return;
    }
    Sensor *sensor = nullptr;
    for (auto sensor_f : sensor_funcs) {
      if (sensor_f.first == type) {
        sensor = sensor_f.second(sensor_data);
        break;
      }
    }
    if (sensor == nullptr) {
      return; // sensor not found
    }

    sensor->type = type;
    sensor->num = sensor_num;

    sensors.push_back(sensor);
  }
}

void readSensors() {
  for (auto &sensor : sensors) {
    sensor->readSensor();
  }
  for (volatile auto &module : modules) {
    module->readModule();
  }
}

/***********************************************/
/***************MODULES*************************/
void module_new() {
  const auto msg_type = command_buffer[1];
  const std::vector<
      std::pair<MODULE_TYPES, std::function<Module *(std::vector<uint8_t> &)>>>
      module_funcs = {
          {MODULE_TYPES::PCA9685,
           [](std::vector<uint8_t> &data) { return new PCA9685_Module(data); }},
          {MODULE_TYPES::HIWONDER_SERVO,
           [](std::vector<uint8_t> &data) { return new Hiwonder_Servo(data); }},
          {MODULE_TYPES::SHUTDOWN_RELAY,
           [](std::vector<uint8_t> &data) {
             return new Shutdown_Relay(data); /* not implemented */
           }},
          {MODULE_TYPES::TMX_SSD1306,
           [](std::vector<uint8_t> &data) { return new TmxSSD1306(data); }},
      };

  if (msg_type == 1) {
    const MODULE_TYPES type = (MODULE_TYPES)command_buffer[3];
    const uint8_t module_num = command_buffer[2];
    std::vector<uint8_t> data;
    data.insert(data.end(), &command_buffer[4], &command_buffer[packet_size]);

    if (type >= MODULE_TYPES::MAX_MODULES) {
      return;
    }
    Module *module = nullptr;
    // if (type == MODULE_TYPES::PCA9685) {
    //   module = new PCA9685_Module(data);
    // } else if (type == MODULE_TYPES::HIWONDER_SERVO) {
    //   module = new Hiwonder_Servo(data);
    // } else if (type == MODULE_TYPES::SHUTDOWN_RELAY) {
    //   return; // not implemented
    //   // module = new Shutdown_Relay(data);
    // } else if (type == MODULE_TYPES::TMX_SSD1306) {
    //   module = new TmxSSD1306(data);
    // } else {
    //   return;
    // }
    for (const auto &[module_type, func] : module_funcs) {
      if (module_type == type) {
        module = func(data);
        break;
      }
    }
    if (module == nullptr) {
      return; // module not found
    }
    module->type = type;
    module->num = module_num;

    modules.push_back(module);
  } else if (msg_type == 0) { // check module type feature detection
    bool found = false;
    const uint8_t module_type_target = command_buffer[2];
    for (const auto &[module_type, func] : module_funcs) {
      if (module_type == module_type_target && func != nullptr) {
        found = true;
        break;
      }
    }
    serial_write({0, // packet length
                  MODULE_MAIN_REPORT,
                  0, // feature check
                  module_type_target, (uint8_t)(found ? 1u : 0u)});
  }
}

void module_data() {
  const uint8_t module_num = command_buffer[1];
  if (module_num > modules.size()) {
    return;
  }
  std::vector<uint8_t> data;
  data.insert(data.end(), &command_buffer[2], &command_buffer[packet_size]);
  modules[module_num]->writeModule(data);
}

bool watchdog_enable_shutdown = true; // if false, then don't do anything with
                                      // the watchdog and just wait for shutdown

void disable_watchdog() {
  hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
}

void enable_watchdog() {
  watchdog_enable(WATCHDOG_TIME,
                  1); // Add watchdog again requiring trigger every 5s
  watchdog_update();
}

/**********************ORIGINAL MODULES******************************/

// DHTS sensor?
void scan_dhts() {
  // read the next dht device
  // one device is read each cycle
  if (dht_count >= 0) {
    if (timer_fired) {
      timer_fired = false;
      int the_index = the_dhts.next_dht_index;
      read_dht(the_dhts.dhts[the_index].data_pin);
      the_dhts.next_dht_index++;
      if (the_dhts.next_dht_index > dht_count) {
        the_dhts.next_dht_index = 0;
      }
    }
  }
}

// TODO: Make use of encode/decode msgs
void read_dht(uint dht_pin) {
  int data[5] = {0, 0, 0, 0, 0};
  uint last = 1;
  uint j = 0;
  float temp_celsius;
  float humidity;
  float nearest;
  int temp_int_part;
  int temp_dec_part;
  int humidity_int_part;
  int humidity_dec_part;

  gpio_set_dir(dht_pin, GPIO_OUT);
  gpio_put(dht_pin, 0);
  sleep_ms(20);
  gpio_set_dir(dht_pin, GPIO_IN);

  sleep_us(1);
  for (uint i = 0; i < DHT_MAX_TIMINGS; i++) {
    uint count = 0;
    while (gpio_get(dht_pin) == last) {
      count++;
      sleep_us(1);
      if (count == 255) {
        break;
      }
    }
    last = gpio_get(dht_pin);
    if (count == 255) {
      break;
    }

    if ((i >= 4) && (i % 2 == 0)) {
      data[j / 8] <<= 1;
      if (count > 46) {
        data[j / 8] |= 1;
      }
      j++;
    }
  }
  if ((j >= 40) &&
      (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
    humidity = (float)((data[0] << 8) + data[1]) / 10;
    if (humidity > 100) {
      humidity = data[0];
    }
    temp_celsius = (float)(((data[2] & 0x7F) << 8) + data[3]) / 10;
    if (temp_celsius > 125) {
      temp_celsius = data[2];
    }
    if (data[2] & 0x80) {
      temp_celsius = -temp_celsius;
    }

    nearest = roundf(temp_celsius * 100) / 100;
    temp_int_part = (int)nearest;
    temp_dec_part = (int)((nearest - temp_int_part) * 100);

    nearest = roundf(humidity * 100) / 100;
    humidity_int_part = (int)nearest;
    humidity_dec_part = (int)((nearest - humidity_int_part) * 100);
  } else {
    // bad data return zeros
    temp_int_part = temp_dec_part = humidity_int_part = humidity_dec_part = 0;
  }
  dht_report_message[DHT_REPORT_PIN] = (int)dht_pin;
  dht_report_message[DHT_HUMIDITY_WHOLE_VALUE] = humidity_int_part;
  dht_report_message[DHT_HUMIDITY_FRAC_VALUE] = humidity_dec_part;

  dht_report_message[DHT_TEMPERATURE_WHOLE_VALUE] = temp_int_part;
  dht_report_message[DHT_TEMPERATURE_FRAC_VALUE] = temp_dec_part;
  serial_write(dht_report_message, 7);
}

#define ENTRY_MAGIC 0xb105f00d
void reset_to_bootloader() {
  hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
  watchdog_hw->scratch[5] = ENTRY_MAGIC;
  watchdog_hw->scratch[6] = ~ENTRY_MAGIC;
  led_debug(10, 100);
  watchdog_reboot(0, 0, 0);

  while (1) {
    tight_loop_contents();
  }
}

#include "hardware/flash.h"

#define FLASH_TARGET_OFFSET (256 * 1024)

const uint8_t *flash_target_contents =
    (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);

void get_id() {
  int id[] = {2, GET_ID, 0};
  id[2] = (uint8_t)(flash_target_contents[0]); // get the id from the flash,
                                               // just the first byte
  serial_write(id, 3);
}

void set_id() {
  // msg format: 2, SET_ID, new_id
  uint8_t new_id = command_buffer[1];
  int id_msg[] = {2, SET_ID, new_id};
  if (new_id ==
      flash_target_contents[0]) { // no need to write anything to flash
    serial_write(id_msg, 3);

    return;
  }
  uint8_t new_id_array[FLASH_PAGE_SIZE] = {0};
  new_id_array[0] = new_id;
  uint32_t ints = save_and_disable_interrupts();

  static_assert(FLASH_PAGE_SIZE == 256);

  flash_range_erase(FLASH_TARGET_OFFSET,
                    FLASH_SECTOR_SIZE); // required to erase before writing

  flash_range_program(FLASH_TARGET_OFFSET, new_id_array, FLASH_PAGE_SIZE);
  restore_interrupts(ints);

  serial_write(id_msg, 3);
}

void feature_detect() {
  // msg format: 2, FEATURE_DETECT, feature_id
  uint8_t feature_id = command_buffer[1];
  std::vector<uint8_t> id_msg = {4, FEATURE_CHECK, feature_id, 0};
  if (feature_id >= command_table.size()) {
    id_msg[3] = 0;
    // id_msg[0]=2;
    serial_write(id_msg);
    return;
  }
  if (command_table[feature_id].command_func == nullptr) {

    id_msg[3] = 0;
  } else {
    id_msg[3] = 1;
  }
  // TODO: add more features for specific types:
  switch (feature_id) {
  case SONAR_NEW:
    id_msg.push_back(MAX_SONARS);
    break;
  case ENCODER_NEW:
    id_msg.push_back(MAX_ENCODERS);
    id_msg.push_back(2); // allow quad enc
    id_msg.push_back(1); // allow pullup/downs
    break;
  case SET_PIN_MODE:
    id_msg.push_back(MAX_DIGITAL_PINS_SUPPORTED);
    id_msg.push_back(12); // adc resolution is 12 bit
    id_msg.push_back(14); // pwm resolution is 14 bit kinda
    id_msg.push_back(MAX_ANALOG_PINS_SUPPORTED);
    for (int i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
      id_msg.push_back(i + ANALOG_OFFSET);
    }
    break;
  case SERVO_ATTACH:
    id_msg.push_back(12);
    break;
  case GET_FIRMWARE_VERSION:
    id_msg.push_back(FIRMWARE_MAJOR);
    id_msg.push_back(FIRMWARE_MINOR);
    break;
  case GET_PICO_UNIQUE_ID:
    break;
  case I2C_BEGIN:
    id_msg.push_back(2); // 2 i2c ports
    break;
  default:
    break;
  }
  serial_write(id_msg);
}

std::vector<Sensor *> sensors;
std::vector<Module *> modules;

/***************************************************************
 *                  MAIN FUNCTION
 ****************************************************************/
void core1_main();
int main() {
  // gpio_init(14);
  // gpio_set_dir(14, GPIO_OUT);
  // gpio_put(14, 0);
  // stdio_init_all();
  stdio_usb_init();
  stdio_set_translate_crlf(&stdio_usb, false);
  // #ifdef WITH_UART_STDIO
  // stdio_set_translate_crlf(&stdio_uart, false);
  // #endif
  stdio_flush();
  check_uart_loopback(); // Mirte-master has pin 0 and 1 tied together, then
  //                        // don't want to use it
  sleep_ms(1000);
  init_led();

  led_debug(5, 100);
  adc_init();
  mm_detect();
  // create an array of pin_descriptors for 100 pins
  // establish the digital pin array
  for (uint8_t i = 0; i < MAX_DIGITAL_PINS_SUPPORTED; i++) {
    the_digital_pins[i].pin_number = i;
    the_digital_pins[i].pin_mode = PIN_MODE_NOT_SET;
    the_digital_pins[i].reporting_enabled = false;
    the_digital_pins[i].last_value = -1;
  }

  // establish the analog pin array
  for (uint8_t i = 0; i < MAX_ANALOG_PINS_SUPPORTED; i++) {
    the_analog_pins[i].reporting_enabled = false;
    the_analog_pins[i].last_value = -1;
  }

  // initialize the sonar structures
  // sonar_data the_hc_sr04s = ;
  for (int i = 0; i < MAX_SONARS; i++) {
    the_hc_sr04s.sonars[i].trig_pin = the_hc_sr04s.sonars[i].echo_pin =
        (uint)-1;
  }

  // blink the board LED twice to show that the board is
  // starting afresh
  led_debug(2, 250);
  set_led_pin(uart_enabled);
  // gpio_put(LED_PIN, uart_enabled);

  // watchdog_enable(WATCHDOG_TIME, 1); // Add watchdog requiring trigger every
  // 5s

  // infinite loop
  uint32_t last_scan = 0;

  // start second core
  multicore_launch_core1(core1_main);

  while (true) {
    // watchdog_update();
    get_next_command();

    if (!stop_reports) {
      if (time_us_32() - last_scan >= (scan_delay)) {
        last_scan += scan_delay;
        scan_digital_inputs();
        scan_analog_inputs();
        scan_sonars();
        scan_dhts();
        scan_encoders();
        readSensors();
        mm_loop();
        if (watchdog_enabled) {
          check_wd_timeout();
        }
      }
      for (auto module : modules) {
        module->updModule();
      }
    }
  }
}

void core1_main() {
  // slow tasks should be done on this core to speed up the main core
  auto last_scan = time_us_32();
  int x = 0;
  while (true) {
    if (time_us_32() - last_scan >= scan_delay) {
      last_scan += scan_delay;
      // for(auto sensor : sensors) {
      //   sensor->core1_update();
      // }

      // Add a memory barrier to signal to GCC that modules may change
      // somehow it otherwise doesnt work, even with volatile or atomic.

      x++;
      if (x % 200 == 0) {
        // just to not trigger on every loop
        asm volatile("" ::: "memory");
      }
      for (volatile auto module : modules) {
        module->core1_update();
      }
    }
  }
}

// Just some checks to make sure the arrays are not changed
static_assert(sizeof(command_buffer) == 30,
              "Command buffer size is not 30 bytes");
static_assert(command_table[3].command_func == &pwm_write,
              "Command table is not correct");
static_assert(command_table[38].command_func == &reset_to_bootloader,
              "Command table is not correct");
static_assert(command_table[33].command_func == &module_new,
              "Command table is not correct");
static_assert(command_table[27].command_func == &set_format_spi,
              "Command table is not correct");