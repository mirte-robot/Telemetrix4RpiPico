#include "module/Hiwonder_Servo.hpp"

#include "led_pin.hpp"
#include "message_types.hpp"
#include "serialization.hpp"

#include "Telemetrix4RpiPico.hpp"

class ScopedMutex {
private:
  mutex_t &mutex;
  bool locked = false;

public:
  ScopedMutex(mutex_t &m) : mutex(m) {}
  void lock() {
    mutex_enter_blocking(&mutex);
    locked = true;
  }
  bool try_lock() {
    if (locked) {
      return true;
    }
    if (!(mutex_try_enter(&mutex, NULL))) {
      return false;
    }
    locked = true;
    return true;
  }
  void unlock() {
    if (locked) {
      mutex_exit(&mutex);
      locked = false;
    }
  }
  ~ScopedMutex() {
    if (locked) {
      mutex_exit(&mutex);
    }
  }
};

void send_debug_info_dis(int x, int y) {
  // send_debug_info(x, y);
}

/**
 * When creating a servo chain:
 * Byte1: 0: uart0, 1: uart1
 * Byte2: rx pin
 * Byte3: tx pin
 * Byte4: wanted amount of servos
 * Byte5-N: ids of servos
 */
Hiwonder_Servo::Hiwonder_Servo(std::vector<uint8_t> &data) {

  auto uart = data[0] == 0 ? uart0 : uart1;
  auto rxPin = data[1];
  auto txPin = data[2];
  auto servos = data[3];
  if (data.size() != (uint8_t)(servos + 4)) {
    return;
  }

  this->bus = new HiwonderBus();
  mutex_init(&this->bus_mutex);
  ScopedMutex bus_mutex(this->bus_mutex);
  send_debug_info_dis(10, __LINE__);
  bus_mutex.lock();
  this->bus->begin(uart, rxPin, txPin);
  this->servos.reserve(servos);
  for (auto i = 0; i < servos; i++) {
    auto id = data[4 + i];
    auto servo = new HiwonderServoItem(this->bus, id);
    mutex_init(&servo->mutex);
    servo->servo.initialize();
    // auto offset = servo->read_angle_offset();
    this->servos.push_back(servo);
    this->enabled_servos++;
  }
  this->bus->enableAll();
  send_debug_info_dis(9, __LINE__);
}

bool Hiwonder_Servo::writeSingle(std::vector<uint8_t> &data, size_t i,
                                 bool single) {
  const auto offset = 2; // 1 for msg_type, 1 for count
  const int numBytes = 5;
  auto data_span =
      std::span(data).subspan(offset + numBytes * i).first<numBytes>();
  auto servoI = data_span[0];
  // TODO: What happens here?
  // TODO: Maybe this needs to decode a i16
  auto angle = (int32_t)decode_u16(data_span.subspan<1, sizeof(uint16_t)>());
  // ((int32_t)data[offset + 1 + numBytes * i] << 8) |
  //        data[offset + 2 + numBytes * i];
  auto time = decode_u16(data_span.subspan<3, sizeof(uint16_t)>());
  send_debug_info(21, time);
  if (servoI >= this->servos.size()) {
    return false;
  }
  // get mutex
  send_debug_info_dis(10, __LINE__);
  ScopedMutex bus_mutex(this->servos[servoI]->mutex);
  if (!bus_mutex.try_lock()) {
    send_debug_info_dis(30, 4);
    return false;
  }
  this->servos[servoI]->lastwritePosition = angle;
  this->servos[servoI]->updated_write = true;
  this->servos[servoI]->write_time = time;
  this->servos[servoI]->disabled =
      false; // in case it was disabled before, we want to try to write again
  // send_debug_info_dis(9, __LINE__);
  return true;
}

void Hiwonder_Servo::writeModule(std::vector<uint8_t> &data) {
  auto data_span = std::span(data);
  auto msg_type = data[0];
  send_debug_info_dis(13, msg_type);
  if (!timeout_safe()) {
    return;
  }
  if (msg_type ==
      SET_ANGLE) { // normal set angle command with one or multiple servos
    auto count = data[1];
    // If just one, directly move, otherwise wait for the other commands to
    // finish before moving
    if (count == 1) {
      this->writeSingle(data, 0, true);
    } else {
      if (data.size() != (uint8_t)(count * 5 + 2)) {
        return;
      }
      for (auto i = 0; i < count; i++) {
        this->writeSingle(data, i, false);
      }
      // this->bus->move_sync_start();
    }
    // TODO: Add Ok Send?
  } else if (msg_type == ENABLE) { // enable msg
    auto count = data[1];
    auto enabled = data[2];
    send_debug_info_dis(10, __LINE__);
    // mutex_enter_blocking(&this->bus_mutex);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    if (count == 0) { // all servos
      if (enabled) {
        this->bus->enableAll();
      } else {
        this->bus->disableAll();
      }
      return;
    }
    for (auto i = 0; i < count; i++) {
      auto servoI = data[3 + i];
      if (servoI >= this->servos.size()) {
        return;
      }
      if (enabled == 1) {
        this->servos[servoI]->servo.enable();
      } else {
        this->servos[servoI]->servo.disable();
      }
    }
    send_debug_info_dis(9, __LINE__);
    // mutex_exit(&this->bus_mutex);
  } else if (msg_type == ID_WRITE) { // update id
    auto new_id = data[1];
    send_debug_info_dis(10, __LINE__);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    this->bus->id_write(new_id);
    send_debug_info_dis(9, __LINE__);
  } else if (msg_type == ID_VERIFY) {
    // id read
    auto check_id = data[1];
    send_debug_info_dis(10, __LINE__);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    HiwonderServo tempServo(this->bus, check_id);
    bool ok = tempServo.id_verify() == check_id;
    // ok = true;
    std::vector<uint8_t> data = {
        ID_VERIFY,   // id check type
        check_id,    // id
        (uint8_t)ok, // ok
    };
    send_debug_info_dis(9, __LINE__);
    this->publishData(data);
  } else if (msg_type == RANGE_WRITE) {
    // range write
    auto id = data[1];
    int16_t min = decode_u16(
        data_span.subspan<2, sizeof(uint16_t)>()); //((int16_t)data[2]
                                                   //<< 8) | data[3];
    int16_t max = decode_u16(
        data_span.subspan<4, sizeof(uint16_t)>()); //((int16_t)data[4]
    //<< 8) | data[5];
    send_debug_info_dis(10, __LINE__);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    this->servos[id]->servo.setLimitsTicks(min / 24,
                                           max /
                                               24); // 24 centidegrees per tick
    send_debug_info_dis(9, __LINE__);
  } else if (msg_type == RANGE_READ) {
    // read range of servo stored in servo

    auto id = data[1];
    // send_debug_info_dis(10, id);
    // send_debug_info_dis(11, this->servos.size());
    send_debug_info_dis(10, __LINE__);
    {
      ScopedMutex bus_mutex(this->bus_mutex);
      bus_mutex.lock();
      send_debug_info_dis(10, __LINE__);
      this->servos[id]->servo.readLimits();

      send_debug_info_dis(9, __LINE__);
    }
    // mutex_exit(&this->bus_mutex);
    auto min = this->servos[id]->servo.minCentDegrees;
    auto max = this->servos[id]->servo.maxCentDegrees;
    std::vector<uint8_t> data = {RANGE_READ, // id check type
                                 id};        //, // id
    // data.reserve(data.size() + 2 * sizeof(uint16_t));

    append_range(data, encode_u16((uint16_t)min));
    append_range(data, encode_u16((uint16_t)max));

    this->publishData(data);
  } else if (msg_type == OFFSET_WRITE) { // Set offset in centideg
    auto id = data[1];
    // TODO: Maybe i16 decode instead
    int16_t offset = (int16_t)decode_i16(
        data_span.subspan<2, sizeof(int16_t)>()); //((int16_t)data[2] << 8) |
                                                  // data[3];
    offset /= 24;
    send_debug_info_dis(10, __LINE__);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    this->servos[id]->servo.angle_offset_adjust(offset);
    this->servos[id]->servo.angle_offset_save();
    send_debug_info_dis(9, __LINE__);
  } else if (msg_type == OFFSET_READ) {
    auto id = data[1];
    send_debug_info_dis(10, __LINE__);
    int16_t offset;
    {
      ScopedMutex bus_mutex(this->bus_mutex);
      bus_mutex.lock();
      offset = (int8_t)this->servos[id]
                   ->servo
                   .read_angle_offset(); // read back as uint8_t, but is signed.
      send_debug_info_dis(9, __LINE__);
    }
    offset *= 24;
    std::vector<uint8_t> data = {OFFSET_READ, // offset type
                                 id};         // id

    append_range(data, encode_i16(offset));
    this->publishData(data);
  } else if (msg_type == VOLTAGE_LIMIT_WRITE) { // write voltage limits
    auto id = data[1];
    // TODO: Shouldn't this use more bytes?
    uint32_t vMin = decode_u16(data_span.subspan<2, sizeof(uint16_t)>());
    uint32_t vMax = decode_u16(data_span.subspan<4, sizeof(uint16_t)>());
    send_debug_info_dis(10, __LINE__);

    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    this->servos[id]->servo.setVoltageLimits(vMin, vMax);
    send_debug_info_dis(9, __LINE__);
  } else if (msg_type == MOTOR_MODE_WRITE) { // motor mode write
    auto id = data[1];
    uint16_t speed = decode_i16(data_span.subspan<2, sizeof(uint16_t)>());
    send_debug_info_dis(10, __LINE__);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    this->servos[id]->servo.motor_mode(speed);
    send_debug_info_dis(9, __LINE__);
  }
  if (msg_type == ADD_SERVO) {
    // This is the actual servo ID
    auto id = data[1];
    send_debug_info_dis(10, __LINE__);
    ScopedMutex bus_mutex(this->bus_mutex);
    bus_mutex.lock();
    auto servo = new HiwonderServoItem(this->bus, id);
    mutex_init(&servo->mutex);
    servo->servo.initialize();
    // auto offset = servo->read_angle_offset();
    this->servos.push_back(servo);
    this->enabled_servos++;
    servo->servo.enable();

    send_debug_info_dis(9, __LINE__);
    std::vector<uint8_t> data = {
        ADD_SERVO,                          // add servo type
        id,                                 // id
        (uint8_t)(this->servos.size() - 1), // idx
    };
    this->publishData(data);
  }
  send_debug_info_dis(11, __LINE__);
}

void Hiwonder_Servo::core1_update() {
  // led_debug(10, 100);
  // return;

  //  if (!timeout_safe()) {
  //   return;
  // }
  // send_debug_info_dis(31, 100);
  if (this->enabled_servos == 0) {
    // send_debug_info(30, 0);
    return;
  }
  ScopedMutex bus_mutex(this->bus_mutex);
  if (!bus_mutex.try_lock()) {
    send_debug_info(30, 1);
    // led_debug(10, 100);
    return;
  }
  // send_debug_info_dis(30, 2);
  for (auto servo : this->servos) {
    if (servo->disabled) { // skip disabled servos, they are very slow
      // send_debug_info(30, 4);
      continue;
    }
    ScopedMutex servo_mutex(servo->mutex);
    if (!servo_mutex.try_lock()) {
      // send_debug_info(30, 3);
      continue;
    }
    auto pos = servo->servo.pos_read();
    if (servo->servo.isCommandOk()) {
      servo->fault_count = 0;
    } else {
      servo->fault_count++;
      if (servo->fault_count > 2) {
        servo->disabled = true;
        // this->enabled_servos--;
      }
      continue;
    }
    //  int32_t pos= time_us_32();
    auto diff = std::abs(pos - servo->lastPublishedPosition);
    const auto min_diff = 1; // 24 * 3; // 3 ticks, or 72 centidegrees

    if (diff > min_diff) {
      servo->updated_read = true;
      servo->lastPublishedPosition = pos;
    }
    // send_debug_info(31, pos);
    servo->lastreadPosition.store(pos);

    // writing updates
    if (servo->updated_write) {
      // send_debug_info_dis(10, __LINE__);
      servo->servo.move_time(servo->lastwritePosition, servo->write_time);
      servo->updated_write = false;
    }
    // send_debug_info_dis(30, __LINE__);
  }
  send_debug_info_dis(30, __LINE__);
}

void Hiwonder_Servo::readModule() {
  if (!timeout_safe()) {
    return;
  }
  if (this->enabled_servos == 0) {
    return;
  }
  // read angle, temp?
  std::vector<uint8_t> data;
  data.reserve(this->servos.size() * 3 + 1);
  data.push_back(GET_ANGLE); // message type servo angles
  // only update position when changed

  for (auto i = -1; auto servo : this->servos) {
    i++;
    if (servo->disabled) { // skip disabled servos, they are very slow
      continue;
    }
    ScopedMutex servo_mutex(servo->mutex);
    if (!servo_mutex.try_lock()) {
      continue;
    }
    if (servo->updated_read || (time_us_32() - servo->last_force_write) >
                                   1'000'000) { // force write every second
      // send_debug_info_dis(10, __LINE__);
      servo->updated_read = false;
      data.push_back(i);
      // Pos is 0...24000 -> 15 bits
      auto pos = servo->lastreadPosition.load();
      // send_debug_info(11, servo->lastreadPosition);
      // servo->lastreadPosition++;
      // sendd
      append_range(data, encode_u16(pos));
      servo->last_force_write = time_us_32();
    } else {
      // send_debug_info(12, servo->last_force_write);
    }
    // send_debug_info_dis(9, __LINE__);
  }
  if (data.size() > 1) {
    // send_debug_info(12, data.size());
    this->publishData(data);
  }
}
