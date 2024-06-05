#pragma once

#include <stdint.h>
#include <vector>
enum MODULE_TYPES : uint8_t { // Max 255 modules, but will always fit in a
                              // single byte!
  PCA9685 = 0,                // 16x 12bit PWM
  HIWONDER_SERVO = 1,
  SHUTDOWN_RELAY = 2,
  TMX_SSD1306 = 3,
  MAX_MODULES
};

class Module {
public:
  virtual void readModule() = 0;
  virtual void writeModule(std::vector<uint8_t> &data) = 0;
  virtual void resetModule() = 0;
  bool stop = false;
  void publishData(const std::vector<uint8_t> &data);

  int num = 0;
  MODULE_TYPES type = MODULE_TYPES::MAX_MODULES;
};