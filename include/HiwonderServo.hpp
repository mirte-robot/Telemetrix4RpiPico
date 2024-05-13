// Modified code from https://github.com/madhephaestus/lx16a-servo
#pragma once
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <algorithm>
#include <stdio.h>
const auto BROADCAST_ID = 0xFE;

enum class HiwonderCommands : uint8_t {
  MOVE_TIME_WRITE = 1,
  MOVE_TIME_READ = 2,
  MOVE_TIME_WAIT_WRITE = 7,
  MOVE_TIME_WAIT_READ = 8,
  MOVE_START = 11,
  MOVE_STOP = 12,
  ID_WRITE = 13,
  ID_READ = 14,
  ANGLE_OFFSET_ADJUST = 17,
  ANGLE_OFFSET_WRITE = 18,
  ANGLE_OFFSET_READ = 19,
  ANGLE_LIMIT_WRITE = 20,
  ANGLE_LIMIT_READ = 21,
  VIN_LIMIT_WRITE = 22,
  VIN_LIMIT_READ = 23,
  TEMP_MAX_LIMIT_WRITE = 24,
  TEMP_MAX_LIMIT_READ = 25,
  TEMP_READ = 26,
  VIN_READ = 27,
  POS_READ = 28,
  OR_MOTOR_MODE_WRITE = 29,
  OR_MOTOR_MODE_READ = 30,
  LOAD_OR_UNLOAD_WRITE = 31,
  LOAD_OR_UNLOAD_READ = 32,
  LED_CTRL_WRITE = 33,
  LED_CTRL_READ = 34,
  LED_ERROR_WRITE = 35,
  LED_ERROR_READ = 36
};
class HiwonderBus {
private:
  int lastCommand = 0;
  typeof(uart1) uart_id = uart1;

public:
  bool _debug = false;
  bool _deepDebug = false;
  HiwonderBus() {}

  // debug enables/disables debug printf's for this servo
  void debug(bool on) { _debug = on; }

  void begin(typeof(uart1) port, int rXpin, int tXpin) {
    const auto BAUD_RATE = 115200;
    const auto DATA_BITS = 8;
    const auto STOP_BITS = 1;
    const auto PARITY = UART_PARITY_NONE;

    this->uart_id = port;
    gpio_init(tXpin);
    gpio_init(rXpin);

    gpio_set_function(tXpin, GPIO_FUNC_UART);
    gpio_set_function(rXpin, GPIO_FUNC_UART);

    auto in = uart_init(uart_id, BAUD_RATE);
    (void)in;
    int actual = uart_set_baudrate(uart_id, BAUD_RATE);
    (void)actual;
    // if (_debug) {
    //   printf("in %i act %i en %i\n", in, actual, uart_is_enabled(uart_id));
    // }
    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(uart_id, false, false);
    uart_set_translate_crlf(uart_id, false);

    // Set our data format
    uart_set_format(uart_id, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(uart_id, true);
    sleep_ms(3);
    while (uart_is_readable(uart_id))
      uart_getc(uart_id);
  }

  // time returns the number of ms to TX/RX n characters
  uint32_t time(uint32_t n) {
    return n * 10 * 1000 / _baud; // 10 bits per char
  }
  uint32_t timeus(uint32_t n) {
    return n * 10 * 1000000 / _baud; // 10 bits per char
  }
  // methods passed through to Serial
  bool available() { return uart_is_readable(uart_id); }
  int read() { return uart_getc(uart_id); }
  void write(const uint8_t *buf, int buflen) {

    sleep_us(10);
    for (int i = 0; i < buflen; i++) {
      uart_putc_raw(uart_id, buf[i]);
      uart_tx_wait_blocking(uart_id);
    }
  }
  int retry = 3;
  void setRetryCount(int count) { retry = count; }
  // write a command with the provided parameters
  // returns true if the command was written without conflict onto the bus
  bool write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt,
                      uint8_t MYID);

  // read sends a command to the servo and reads back the response into the
  // params buffer. returns true if everything checks out correctly.
  bool read_no_retry(uint8_t cmd, uint8_t *params, int param_len, uint8_t MYID);
  // read sends a command to the servo and reads back the response into the
  // params buffer. returns true if everything checks out correctly.
  bool rcv(uint8_t cmd, uint8_t *params, int param_len, uint8_t MYID);

  bool write(HiwonderCommands cmd, const uint8_t *params, int param_cnt,
             uint8_t MYID) {
    return this->write((uint8_t)cmd, params, param_cnt, MYID);
  }
  // write a command with the provided parameters
  // returns true if the command was written without conflict onto the bus
  bool write(uint8_t cmd, const uint8_t *params, int param_cnt, uint8_t MYID) {
    if (retry == 0) {
      return write_no_retry(cmd, params, param_cnt, MYID);
    } else {
      for (int i = 0; i < retry; i++) {
        bool ok = write_no_retry(cmd, params, param_cnt, MYID);
        if (ok) {
          return true;
        }
      }
    }
    return false;
  }
  bool read(HiwonderCommands cmd, uint8_t *params, int param_len,
            uint8_t MYID) {
    return this->read((uint8_t)cmd, params, param_len, MYID);
  }

  // read sends a command to the servo and reads back the response into the
  // params buffer. returns true if everything checks out correctly.
  bool read(uint8_t cmd, uint8_t *params, int param_len, uint8_t MYID) {

    if (retry == 0) {
      return read_no_retry(cmd, params, param_len, MYID);
    } else {
      for (int i = 0; i < retry; i++) {
        bool ok = read_no_retry(cmd, params, param_len, MYID);
        if (ok) {
          return true;
        }
      }
    }
    return false;
  }

  // int _port = NULL;
  int _baud = 115200;
  /**
   * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
   Command value: 31 Length: 4
   Parameter 1: Whether the internal motor of the servo is unloaded power-down
   or not, the range 0 or 1, 0 represents the unloading power down, and the
   servo has no torque output. 1 represents the loaded motor, then the servo has
   a torque output, the default value is 0.
   @return sucess
   */
  bool disableAll() {
    uint8_t params[] = {0};
    return write(HiwonderCommands::LOAD_OR_UNLOAD_WRITE, params, 1,
                 BROADCAST_ID);
  }
  /**
   * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
   Command value: 31 Length: 4
   Parameter 1: Whether the internal motor of the servo is unloaded power-down
   or not, the range 0 or 1, 0 represents the unloading power down, and the
   servo has no torque output. 1 represents the loaded motor, then the servo has
   a torque output, the default value is 0.
   @return sucess
   */
  bool enableAll() {
    uint8_t params[] = {1};
    return write(HiwonderCommands::LOAD_OR_UNLOAD_WRITE, params, 1,
                 BROADCAST_ID);
  }
  /**
   * Command name: SERVO_MOVE_START Command value: 11
   Length: 3
   With the use of command SERVO_MOVE_TIME_WAIT_WRITE, described in
   point 3
   @return sucess
   */
  bool move_sync_start() {
    uint8_t params[1];
    return write(HiwonderCommands::MOVE_START, params, 1, BROADCAST_ID);
  }
  /**
   * Command name: SERVO_MOVE_STOP Command value: 12
   Length: 3
   When the command arrives at the servo, it will stop running
   This is sent to all servos at once
   */
  void stopAll() {
    uint8_t params[1];
    write(HiwonderCommands::MOVE_STOP, params, 1, BROADCAST_ID);
  }
  // id_read returns the ID of the servo, useful if the id is 0xfe, which is
  // broadcast...
  uint8_t id_read() {
    uint8_t params[1];
    if (!read(HiwonderCommands::ID_READ, params, 1, BROADCAST_ID)) {
      return 0;
    }
    return params[0];
  }

  // id_write sets the id of the servo, updates the object's id if write appears
  // successful
  void id_write(uint8_t id) {
    uint8_t params[] = {id};
    write(HiwonderCommands::ID_WRITE, params, 1, BROADCAST_ID);
  }
};

class HiwonderServo {
private:
  bool commandOK = true;
  int32_t lastKnownGoodPosition = 0;
  bool isMotorMode = false;
  bool isInitialized = false;
  // private:
  HiwonderBus *_bus;

public:
  int32_t staticOffset = 0;
  int32_t maxCentDegrees = 240000;
  int32_t minCentDegrees = 0;
  bool disabled = false;
  int fault_count = 0;

  // Used for telemetrix to only publish on change:
  int32_t lastPublishedPosition = 0;

  uint8_t _id = BROADCAST_ID;
  HiwonderServo(HiwonderBus *bus, int id) : _bus(bus), _id(id) {}
  /**
   * Set the current position as a specific angle.
   *
   * This function will first read the current position, then
   * calculate an offset to be applied to all position reads and writes.
   * The offset will make convert the current position to the value passed in
   *
   * This is a function to be called when the motor hits a limit switch
   * and load in a specific value when the limit is reached
   *
   */
  bool calibrate(int32_t currentAngleCentDegrees, int32_t min_angle_cent_deg,
                 int32_t max_angle_cent_deg) {
    if (min_angle_cent_deg >= max_angle_cent_deg) {
      // // Serial.println("Min can not be greater than max for  " + String(_id)
      // + " halting");
      while (1)
        ;
    }
    int32_t current;
    initialize();
    do {
      pos_read();
      current = pos_read_cached();
      if (!isCommandOk()) {
        // Serial.println("Calibration read FAILED! on index " + String(_id));
      }
    } while (
        !isCommandOk()); // this is a calibration and can not be allowed to fail

    staticOffset = currentAngleCentDegrees - current;
    int32_t min_angle_in_Ticks = (min_angle_cent_deg - staticOffset) / 24;
    int32_t max_angle_in_Ticks = (max_angle_cent_deg - staticOffset) / 24;
    // int32_t currentTicks = current / 24;
    int32_t angularOffset = 1450;
    int32_t angularOffsetTicks = angularOffset / 24;
    if (min_angle_in_Ticks < 0 || max_angle_in_Ticks > 1000) {
      int32_t theoretivalMinError =
          currentAngleCentDegrees - min_angle_cent_deg;
      int32_t theoretivalMinErrorTicks = theoretivalMinError / 24;
      int32_t newSetpointTicks = theoretivalMinErrorTicks + angularOffsetTicks;
      int32_t newAngle = (newSetpointTicks * 24) + staticOffset;
      // Serial.println("ERROR! bounds of servo ID " + String(_id) + " can not
      // be outside hardware limit"); Serial.println("\tlower " +
      // String(min_angle_in_Ticks) + " ticks (Must be > 0)");
      // Serial.println("\tcurrent " + String(currentTicks) + " ticks " +
      // String(pos_read_cached() + staticOffset) + "deg");
      // Serial.println("\tupper " + String(max_angle_in_Ticks) + " ticks (Must
      // be < 1000)"); Serial.println("\terror " +
      // String(theoretivalMinErrorTicks) + " ticks "); Serial.println("\tnewset
      // " + String(newSetpointTicks) + " ticks "); Serial.println("\tnewset deg
      // " + String(newAngle) + " centDeg ");

      min_angle_in_Ticks = 0;
      max_angle_in_Ticks = 1000;
      minCentDegrees = (min_angle_in_Ticks * 24) + staticOffset;
      maxCentDegrees = ((max_angle_in_Ticks)*24) + staticOffset;
      setLimitsTicks(min_angle_in_Ticks, max_angle_in_Ticks);
      move_time(newAngle, 0);
      sleep_ms(500);
      return false;
    } else {
      // Serial.println("\nBounds of servo ID " + String(_id) + " ok!");
      // Serial.println("\tlower " + String(min_angle_in_Ticks) + " ticks ");
      // Serial.println("\tcurrent " + String(currentTicks) + " ticks ");
      // Serial.println("\tupper " + String(max_angle_in_Ticks) + " ticks ");
    }
    setLimitsTicks(min_angle_in_Ticks, max_angle_in_Ticks);
    minCentDegrees = (min_angle_in_Ticks * 24) + staticOffset;
    maxCentDegrees = ((max_angle_in_Ticks)*24) + staticOffset;
    // if (abs(min_angle_cent_deg - minCentDegrees) > 24)
    // 	// Serial.println("FAULT Min angle desired was " +
    // String(min_angle_cent_deg) + " got " + String(minCentDegrees)); if
    // (abs(max_angle_cent_deg - maxCentDegrees) > 24)
    // 	// Serial.println("FAULT max angle desired was " +
    // String(max_angle_cent_deg) + " got " + String(maxCentDegrees));
    return true;
  }
  void setLimitsTicks(int32_t lower, int32_t upper) {
    if (lower < 0) {
      lower = 0;
    }
    if (upper > 1000) {
      upper = 1000;
    }
    for (int i = 0; i < 2; i++) {
      uint8_t params[] = {(uint8_t)lower, (uint8_t)(lower >> 8), (uint8_t)upper,
                          (uint8_t)(upper >> 8)};
      commandOK =
          _bus->write(HiwonderCommands::ANGLE_LIMIT_WRITE, params, 4, _id);

      if (isCommandOk()) {
        return;
      }
    }
  }

  // Command name: SERVO_VIN_LIMIT_WRITE Command value: 22
  // Length: 7
  // Parameter 1: lower 8 bits of minimum input voltage
  // Parameter 2: higher 8 bits of minimum input voltage, range 4500~12000mv
  // Parameter 3: lower 8 bits of maximum input voltage
  // Parameter 4: higher 8 bits of maximum input voltage, range 4500~12000mv
  // And the minimum input voltage should always be less than the maximum input
  // voltage. The command is sent to the servo, and the input voltage of the
  // servo will be limited between the minimum and the maximum. If the servo is
  // out of range, the led will flash and alarm (if an LED alarm is set). In
  // order to protect the servo, the motor will be in the unloaded power
  // situation, and the servo will not output torque and the input limited
  // voltage value supports for power-down save.
  void setVoltageLimits(uint32_t lower, uint32_t upper) {
    lower = std::clamp<uint32_t>(lower, 4500, 14000);
    upper = std::clamp<uint32_t>(upper, lower + 1, 14000);
    uint8_t params[] = {(uint8_t)lower, (uint8_t)(lower >> 8), (uint8_t)upper,
                        (uint8_t)(upper >> 8)};
    for (auto i = 0; i < 2; i++) {
      commandOK = _bus->write(HiwonderCommands::VIN_LIMIT_WRITE, params,
                              sizeof(params), _id);

      if (isCommandOk()) {
        return;
      }
    }
  }

  int32_t getMinCentDegrees() { return minCentDegrees; }
  int32_t getMaxCentDegrees() { return maxCentDegrees; }
  bool isCommandOk() { return commandOK; }
  void initialize() {
    if (isInitialized) {
      return;
    }
    isInitialized = true;
    motor_mode(0);
    pos_read();
    readLimits();
    sleep_ms(1);
  }

  void readLimits() {
    uint8_t params[4];
    int numFail = 0;
    do {
      if (!_bus->read(HiwonderCommands::ANGLE_LIMIT_READ, params, 4, _id)) {
        commandOK = false;
        if (_bus->_debug) {
          // Serial.println("ERROR reading limits #" + String(_id));
        }
      } else {
        commandOK = true;
        int lowicks = (params[0] | ((uint16_t)params[1] << 8));
        int highticks = (params[2] | ((uint16_t)params[3] << 8));

        minCentDegrees = (lowicks * 24) + staticOffset;
        maxCentDegrees = (highticks * 24) + staticOffset;
        if (minCentDegrees > maxCentDegrees) {
          // Serial.println(
          // "ERR MotorID:" + String(_id) + " Min set " + String(minCentDegrees)
          // + " max = " + String(maxCentDegrees) + " Min ticks " +
          // String(lowicks) + " max ticks = " + String(highticks));

          maxCentDegrees = 24000;
          minCentDegrees = 0;
        }
        // else
        // Serial.println(
        // "MotorID:" + String(_id) + " Min set " + String(minCentDegrees) + "
        // max = " + String(maxCentDegrees));
      }
    } while (!isCommandOk() &&
             numFail++ <
                 3); // this is a calibration and can not be allowed to fail
  }

  /**
   * Length: 7
   Parameter 1: lower 8 bits of angle value
   Parameter 2: higher 8 bits of angle value.range 0~100. corresponding to the
   servo angle of 0 - 240 °, that means the minimum angle of the servo can be
   varied is 0.24 degree.
   Parameter 3: lower 8 bits of time value
   Parameter 4: higher 8 bits of time value. the range of time is 0~30000ms.
   When the command is sent to servo, the servo will be rotated from current
   angle to parameter angle at uniform speed within param
   */
  void move_time(int32_t angle, uint16_t time) {
    initialize();
    if (angle > maxCentDegrees) {

      // Serial.println("ERROR Capped set at max " + String(maxCentDegrees) + "
      // attempted " + String(angle));
      angle = maxCentDegrees;
    }
    if (angle < minCentDegrees) {
      // Serial.println("ERROR Capped set at min " + String(minCentDegrees) + "
      // attempted " + String(angle));
      angle = minCentDegrees;
    }
    if (isMotorMode) {
      motor_mode(0);
    }
    angle = (angle - staticOffset) / 24;
    if (angle > 1000) {
      angle = 1000;
    }
    if (angle < 0) {
      angle = 0;
    }
    // Serial.println("Setting ticks " + String(angle) + " on ID " +
    // String(_id));
    uint8_t params[] = {(uint8_t)angle, (uint8_t)(angle >> 8), (uint8_t)time,
                        (uint8_t)(time >> 8)};
    commandOK = _bus->write(HiwonderCommands::MOVE_TIME_WRITE, params, 4, _id);
  }
  /**
   * Command name: SERVO_MOVE_TIME_WAIT_WRITE
   Command value: 7
   Length : 7
   Parameter1: lower 8 bits of preset angle
   Parameter2: higher 8 bits of preset angle. range 0~100. corresponding to the
   servo angle of 0 ~ 240 °. that means the minimum angle of the servo can be
   varied is 0.24 degree.
   Parameter3: lower 8 bits of preset time
   Parameter3: higher 8 bits of preset time. the range of time is 0~30000ms.
   The function of this command is similar to this
   “SERVO_MOVE_TIME_WRITE” command in the first point. But the difference
   is that the servo will not immediately turn when the command arrives at the
   servo,the servo will be rotated from current angle to parameter angle at
   unifor m speed within parameter time until the command name SERVO_MOVE_ST ART
   sent to servo(command value of 11) , then the servo will be rotate
   */
  void move_time_and_wait_for_sync(int32_t angle, uint16_t time) {
    initialize();
    if (angle > maxCentDegrees) {
      angle = maxCentDegrees;
      // Serial.println("ERROR Capped set at max " + String(maxCentDegrees));
    }
    if (angle < minCentDegrees) {
      angle = minCentDegrees;
      // Serial.println("ERROR Capped set at min " + String(minCentDegrees));
    }
    if (isMotorMode) {
      motor_mode(0);
    }
    angle = (angle - staticOffset) / 24;
    uint8_t params[] = {(uint8_t)angle, (uint8_t)(angle >> 8), (uint8_t)time,
                        (uint8_t)(time >> 8)};
    commandOK =
        _bus->write(HiwonderCommands::MOVE_TIME_WAIT_WRITE, params, 4, _id);
  }

  /**
   * Command name: SERVO_MOVE_STOP Command value: 12
   Length: 3
   When the command arrives at the servo, it will stop running
   */
  void stop() {
    uint8_t params[1];
    commandOK = _bus->write(HiwonderCommands::MOVE_STOP, params, 1, _id);
  }

  /**
   * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
   Command value: 31 Length: 4
   Parameter 1: Whether the internal motor of the servo is unloaded power-down
   or not, the range 0 or 1, 0 represents the unloading power down, and the
   servo has no torque output. 1 represents the loaded motor, then the servo has
   a torque output, the default value is 0.
   */
  void disable() {
    uint8_t params[] = {0};
    commandOK =
        _bus->write(HiwonderCommands::LOAD_OR_UNLOAD_WRITE, params, 1, _id);
  }
  /**
   * Command name: SERVO_LOAD_OR_UNLOAD_WRITE
   Command value: 31 Length: 4
   Parameter 1: Whether the internal motor of the servo is unloaded power-down
   or not, the range 0 or 1, 0 represents the unloading power down, and the
   servo has no torque output. 1 represents the loaded motor, then the servo has
   a torque output, the default value is 0.
   */
  void enable() {
    uint8_t params[] = {1};
    commandOK =
        _bus->write(HiwonderCommands::LOAD_OR_UNLOAD_WRITE, params, 1, _id);
  }

  /**
   * Command name: SERVO_OR_MOTOR_MODE_WRITE
   Command value: 29
   Length: 7
   Parameter 1: Servo mode, range 0 or 1, 0 for position control mode, 1 for
   motor control mode, default 0,
   Parameter 2: null value
   Parameter 3: lower 8 bits of rotation speed value
   Parameter 4: higher 8 bits of rotation speed value. range -1000~1000,
   Only in the motor control mode is valid, control the motor speed, the value
   of the negative value represents the reverse, positive value represents the
   forward rotation. Write mode and speed do not support power-down save.
   Note: Since the rotation speed is the “signed short int” type of data, it is
   forced to convert the data to convert the data to “unsigned short int “type
   of data before sending the command packet.
   */
  void motor_mode(int16_t speed) {
    bool isMotorMode_tmp = speed != 0;
    uint8_t params[] = {(uint8_t)(isMotorMode_tmp ? 1 : 0), 0, (uint8_t)speed,
                        (uint8_t)(speed >> 8)};
    commandOK =
        _bus->write(HiwonderCommands::OR_MOTOR_MODE_WRITE, params, 4, _id);
    if (commandOK) {
      isMotorMode = isMotorMode_tmp;
    }
  }
  // angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
  void angle_offset_save() {
    uint8_t params[1];
    _bus->write(HiwonderCommands::ANGLE_OFFSET_WRITE, params, 1, _id);
  }
  // angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
  void angle_offset_adjust(int16_t angle) {
    int32_t tmp = (int32_t)angle;

    uint8_t params[] = {(uint8_t)tmp};
    commandOK =
        _bus->write(HiwonderCommands::ANGLE_OFFSET_ADJUST, params, 1, _id);
  }
  // angle_adjust sets the position angle offset in centi-degrees (-3000..3000)
  int16_t read_angle_offset() {
    uint8_t params[1];
    if (!_bus->read(HiwonderCommands::ANGLE_OFFSET_READ, params, 1, _id)) {
      commandOK = false;
      return 0;
    }
    commandOK = true;
    return params[0];
  }

  // pos_read returns the servo position in centi-degrees (0..24000)
  int32_t pos_read() {
    initialize();
    uint8_t params[3];
    if (!_bus->read(HiwonderCommands::POS_READ, params, 2, _id)) {
      // if (_bus->_debug)
      // Serial.print("Position Read failed " + String(_id) + "\n\n");
      commandOK = false;
      return pos_read_cached() + staticOffset;
    }
    commandOK = true;
    lastKnownGoodPosition =
        ((int16_t)params[0] | ((int16_t)params[1] << 8)) * 24;
    return pos_read_cached() + staticOffset;
  }
  /**
   * Get the cached position from the most recent read
   */
  int32_t pos_read_cached() { return lastKnownGoodPosition; }

  // id_read returns the ID of the servo, useful if the id is 0xfe, which is
  // broadcast...
  uint8_t id_read() {
    uint8_t params[1];
    // printf("read %i\n", __LINE__);
    if (!_bus->read(HiwonderCommands::ID_READ, params, 1, BROADCAST_ID)) {
      commandOK = false;
      return 0;
    }
    commandOK = true;
    return params[0];

  } // id_read returns the ID of the servo using its ID for verification
  uint8_t id_verify() {
    uint8_t params[1];
    if (!_bus->read(HiwonderCommands::ID_READ, params, 1, _id)) {
      commandOK = false;
      return 0;
    }
    commandOK = true;
    return params[0];
  }

  // id_write sets the id of the servo, updates the object's id if write appears
  // successful
  void id_write(uint8_t id) {
    uint8_t params[] = {id};
    bool ok = _bus->write(HiwonderCommands::ID_WRITE, params, 1, BROADCAST_ID);
    if (ok && _id != BROADCAST_ID) {
      _id = id;
    }
    commandOK = ok;
  }
  /**
   * Command name: SERVO_OR_MOTOR_MODE_READ
   Command value: 30 Length: 3
   Read the relative values of the servo, for the details of the command package
   that the servo returns to host computer, please refer to the description of
   Table 4 below.
   */
  bool readIsMotorMode() {

    uint8_t params[4];
    if (!_bus->read(HiwonderCommands::OR_MOTOR_MODE_READ, params, 4, _id)) {
      commandOK = false;
      return false;
    }
    commandOK = true;
    isMotorMode = params[0] == 1;
    return isMotorMode;
  }

  // temp_read returns the servo temperature in centigrade
  uint8_t temp() {
    uint8_t params[1];
    if (!_bus->read(HiwonderCommands::TEMP_READ, params, 1, _id)) {
      commandOK = false;
      return 0;
    }
    commandOK = true;
    return params[0];
  }

  // vin_read returns the servo input voltage in millivolts
  uint16_t vin() {
    uint8_t params[2];
    if (!_bus->read(HiwonderCommands::VIN_READ, params, 2, _id)) {
      commandOK = false;
      return 0;
    }
    commandOK = true;
    return params[0] | ((uint16_t)params[1] << 8);
  }
};
