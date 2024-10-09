#pragma once

#include "ssd1306.h"
#include "textRenderer/TextRenderer.h"

#include "Modules.hpp"
#include <string>
class TmxSSD1306 : public Module {
public:
  void readModule();
  void writeModule(std::vector<uint8_t> &data);
  void resetModule();
  void updModule();
  enum MessageType { TEXT = 0, TEXT_DONE = 1, BINARY = 2, BINARY_DONE = 3 };
  pico_ssd1306::SSD1306 *display;
  TmxSSD1306(std::vector<uint8_t> &data);
  FrameBuffer frameBuffer;
  std::string text_buff;
  ~TmxSSD1306() { delete display; }
  bool isWriting = false;
  MessageType currentMessage = TEXT;
  size_t currentLen = 0;
};