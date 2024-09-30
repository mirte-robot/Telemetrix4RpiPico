#include "tmx_ssd1306.hpp"

void TmxSSD1306::readModule() {
  // This is a dummy function, as the SSD1306 does not have any data to read
}
void TmxSSD1306::updModule() {
  if (this->isWriting) {
    // update the oled a line every loop
    this->isWriting = this->display->postWrite();
    if (!this->isWriting) { // done writing
      this->publishData({this->currentMessage, (uint8_t)this->currentLen,
                         (uint8_t)this->display->x, this->display->enabled});
    }
  }
}

void TmxSSD1306::writeModule(std::vector<uint8_t> &data) {
  // byte0: 0: text, 1: text write, 2: binary data, 3: binary write
  if (data.size() == 0) {
    return;
  }
  if (this->display->enabled == false) {

    this->publishData({(uint8_t)data[0], (uint8_t)254});
    return;
  }
  if (data[0] == MessageType::TEXT) {
    // byte1: length
    // rest: text
    uint8_t length = data[1];
    std::string text(data.begin() + 2, data.begin() + 2 + length);
    this->text_buff.append(text);

  } else if (data[0] == MessageType::TEXT_DONE) {
    this->frameBuffer.clear();
    pico_ssd1306::drawText(display, courb08_6x11, this->text_buff.c_str(), 0,
                           0);
    display->sendBuffer(); // 'non'blocking, real data transfer in the postWrite
                           // call.
    auto len = this->text_buff.length();
    this->text_buff.clear();
    this->currentLen = len;
    this->currentMessage = MessageType::TEXT_DONE;
    this->isWriting = true;
  } else if (data[0] == MessageType::BINARY) {
    // byte1: index
    // 16 bytes with data
    uint8_t index = data[1];
    for (int i = 0; i < 16; i++) {
      this->frameBuffer.get()[index * 16 + i] = data[2 + i];
    }
  } else if (data[0] == MessageType::BINARY_DONE) {
    display->sendBuffer();
    this->isWriting = true;
    this->currentMessage = MessageType::BINARY_DONE;
    this->currentLen = 1;
  }
}
TmxSSD1306::TmxSSD1306(std::vector<uint8_t> &data) {
  if (data.size() == 0) {
    return;
  }
  auto i2c_port = data[0] ? i2c1 : i2c0;

  // Ensure this firmware is compatible with other tmx versions.
  auto address = (data.size() > 1) ? data[1] : 0x3C;

  this->display = new pico_ssd1306::SSD1306(i2c_port, address,
                                            pico_ssd1306::Size::W128xH64);
  this->display->setPostWrite(true);
  this->display->setOrientation(0);

  this->display->setBuffer(this->frameBuffer.get());

  this->text_buff.reserve(150);
}

void TmxSSD1306::resetModule() {
  if (!this->display->enabled) {
    return;
  }
  // just show nothing when ROS is stopped
  this->display->clear();
  this->display->sendBuffer();
}