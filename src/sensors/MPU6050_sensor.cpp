#include <sensors/MPU6050_sensor.hpp>

extern void send_debug_info(uint id, uint value);

MPU6050_Module::MPU6050_Module(uint8_t settings[SENSORS_MAX_SETTINGS_A]) {
  // send_debug_info(30, settings[0]);
  // send_debug_info(31, settings[1]);
  this->mpu = new MPU6050(settings[0] == 1 ? i2c1 : i2c0, settings[1]);
  // data format: i2c port, i2c address
}

void MPU6050_Module::readSensor() {
  int16_t accel[3], gyro[3], temp;
  if (!mpu->ok) {
    return;
  }
  mpu->read(accel, gyro, &temp);
  std::vector<uint8_t> data(14);
  for (int i = 0; i < 3; i++) {
    data[i * 2] = accel[i] >> 8;
    data[i * 2 + 1] = accel[i] & 0xFF;
  }
  for (int i = 0; i < 3; i++) {
    data[6 + i * 2] = gyro[i] >> 8;
    data[6 + i * 2 + 1] = gyro[i] & 0xFF;
  }
  data[12] = temp >> 8;
  data[13] = temp & 0xFF;
  this->writeSensorData(data);
}

void MPU6050_Module::resetSensor() { mpu->reset(); }
