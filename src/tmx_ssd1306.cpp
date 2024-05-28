#include "tmx_ssd1306.hpp"
void TmxSSD1306::readModule() {
  // This is a dummy function, as the SSD1306 does not have any data to read
}

void TmxSSD1306::writeModule(std::vector<uint8_t> &data) {
  // byte0: 0: text, 1: binary data, 2: binary write
  if (data[0] == MessageType::TEXT) {
    // byte1: length
    // rest: text
    uint8_t length = data[1];
    std::string text(data.begin() + 2, data.begin() + 2 + length);
    this->text_buff.append(text);
    this->display->clear();

  } else if (data[0] == MessageType::TEXT_DONE) {
    pico_ssd1306::drawText(display, font_8x8, this->text_buff.c_str(), 0, 0);
    display->sendBuffer();
  } else if (data[0] == MessageType::BINARY) {
    // byte1: index
    // 16 bytes with data
    uint8_t index = data[1];
    for (int i = 0; i < 16; i++) {
      this->frameBuffer.get()[index + i] = data[2 + i];
    }
  } else if (data[0] == MessageType::BINARY_DONE) {
    display->sendBuffer();
  }
}