
#include "i2c_helpers.hpp"

// template <typename T>
// int write_i2c_t(int i2c_port, int addr, const T &bytes, bool nostop){
//      i2c_inst_t *i2c;

//     if (i2c_port)
//     {
//         i2c = i2c1;
//     }
//     else
//     {
//         i2c = i2c0;
//     }
//     int i2c_sdk_call_return_value =
//         i2c_write_blocking_until(i2c, (uint8_t)addr, bytes.data(),
//         bytes.size(),
//                                  nostop, make_timeout_time_ms(50));
//     return i2c_sdk_call_return_value == bytes.size();
// }

int write_i2c(int i2c_port, int addr, const std::vector<uint8_t> &bytes,
              bool nostop) {
  return write_i2c_t(i2c_port, addr, bytes, nostop);
}

int read_i2c(int i2c_port, int addr, const std::vector<uint8_t> &write_bytes,
             int bytes_to_read, std::vector<uint8_t> &read_bytes) {

  // write i2c
  if (!write_bytes.empty()) {
    write_i2c(i2c_port, addr, write_bytes, true);
  }
  i2c_inst_t *i2c;

  if (i2c_port) {
    i2c = i2c1;
  } else {
    i2c = i2c0;
  }
  read_bytes.resize(bytes_to_read, 0);
  auto i2c_sdk_return =
      i2c_read_blocking_until(i2c, addr, read_bytes.data(), bytes_to_read,
                              false, make_timeout_time_ms(50));

  return i2c_sdk_return == bytes_to_read;
}

uint32_t millis() { return time_us_32() / 1000; }