#pragma once

#include "ssd1306.h"
#include "textRenderer/TextRenderer.h"

#include "Modules.hpp"
#include <string>
class TmxSSD1306 : public Module {
public:
  void readModule();
  void writeModule(std::vector<uint8_t> &data);
  void resetModule() {}
  enum MessageType { TEXT = 0, TEXT_DONE = 1, BINARY = 2, BINARY_DONE = 3 };
  pico_ssd1306::SSD1306 *display;
  TmxSSD1306(std::vector<uint8_t> &data) {
    auto i2c_port = data[0] ? i2c1 : i2c0;
    this->display =
        new pico_ssd1306::SSD1306(i2c_port, 0x3C, pico_ssd1306::Size::W128xH64);
    this->display->setBuffer(this->frameBuffer.get());
    this->text_buff.reserve(150);
  }
  FrameBuffer frameBuffer;
  std::string text_buff;
  ~TmxSSD1306() { delete display; }
};