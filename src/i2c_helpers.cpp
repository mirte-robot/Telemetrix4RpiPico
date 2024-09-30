
#include "i2c_helpers.hpp"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
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


int reset_i2c(  uint32_t scl_gpio,
  uint32_t sda_gpio, uint32_t port)
 {
  // from https://github.com/apache/nuttx/pull/4786/files
  unsigned int clock_count;
  unsigned int stretch_count;
  uint32_t frequency;
  int pin;
  int ret;

  // DEBUGASSERT(dev);

  // /* Our caller must own a ref */

  // DEBUGASSERT(priv->refs > 0);

  // /* Lock out other clients */

  // i2c_takesem(&priv->mutex);

  ret = -1;

  /* De-init the port */

  // i2c_disable(priv);

  /* Use GPIO configuration to un-wedge the bus */


  gpio_init(sda_gpio);
    gpio_set_dir(sda_gpio, GPIO_OUT);

  // gpio_setdir(sda_gpio, true);
  gpio_set_pulls(sda_gpio, true, false);  /* Pull up */
  gpio_put(sda_gpio, true);

  gpio_init(scl_gpio);
  gpio_set_dir(scl_gpio, true);
  gpio_set_pulls(scl_gpio, true, false);
  gpio_put(scl_gpio, true);

  /* Let SDA go high */

  gpio_put(sda_gpio, true);

  /* Clock the bus until any slaves currently driving it let it go. */

  clock_count = 0;
  while (!gpio_get(sda_gpio))
    {
      /* Give up if we have tried too hard */

      if (clock_count++ > 10)
        {
          goto out;
        }

      /* Sniff to make sure that clock stretching has finished. If the bus
       * never relaxes, the reset has failed.
       */

      stretch_count = 0;
      while (!gpio_get(scl_gpio))
        {
          /* Give up if we have tried too hard */

          if (stretch_count++ > 10)
            {
              goto out;
            }

          sleep_us(10);
        }

      /* Drive SCL low */

      gpio_put(scl_gpio, false);
      sleep_us(10);

      /* Drive SCL high again */

      gpio_put(scl_gpio, true);
      sleep_us(10);
    }

  /* Generate a start followed by a stop to reset slave state machines. */

  gpio_put(sda_gpio, false);
  sleep_us(10);
  gpio_put(scl_gpio, false);
  sleep_us(10);
  gpio_put(scl_gpio, true);
  sleep_us(10);
  gpio_put(sda_gpio, true);
  sleep_us(10);

  ret = 1;

out:

  /* Revert the GPIO configuration. */

  gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
  gpio_set_function(scl_gpio, GPIO_FUNC_I2C);

  /* Reset I2C subsystem */


  /* Re-init the port */
 if (port == 0) {
    i2c_init(i2c0, 100 * 1000);
  } else {
    i2c_init(i2c1, 100 * 1000);
  }

out_without_reinit:

  /* Release the port for re-use by other clients */

  return ret;

}