#include <neopixel.hpp>
static unsigned int WS2812_sm = 0;

#define DMA_CHANNEL (0)
#define DMA_CHANNEL_MASK (1u << DMA_CHANNEL)

#define RESET_TIME_US                                                          \
  (60) // RES time, specification says it needs at least 50 us. Need to pause
       //   bit stream for this time at the end to latch the values into the
       //   LED

static struct semaphore
    reset_delay_complete_sem; // semaphore used to make a delay at the end of
                              // the transfer. Posted when it is safe to output
                              // a new set of values
static alarm_id_t reset_delay_alarm_id; //  alarm id handle for handling delay

void TMX_NeoPixel::init(int LED_count, int LED_pin, int RED_fill,
                        int GREEN_fill, int BLUE_fill) {
  this->LED_count = LED_count;
  this->LED_pin = LED_pin;

  PIO pio = pio0;
  WS2812_sm = WS2812_sm; // state machine used
  uint offset = pio_add_program(pio, &ws2812_program);
  ws2812_init(pio, WS2812_sm, offset, LED_pin, 800000,
              false); // initialize it for 800 kHz
  sem_init(&reset_delay_complete_sem, 1,
           1); // semaphore initially posted so we don't block first time
  dma_init(pio, WS2812_sm, LED_count);

  this->fill(RED_fill, GREEN_fill, BLUE_fill, true);
  this->show();
}
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
  return (uint32_t)(((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) |
                    (uint32_t)(b))
         << 8u;
}

void TMX_NeoPixel::setPixelColor(int pixel, int RED, int GREEN, int BLUE,
                                 bool show) {
  if (pixel < 0 || pixel >= this->LED_count) {
    return;
  }
  this->pixels[pixel] = urgb_u32(RED, GREEN, BLUE);
  if (show) {
    this->show();
  }
}

void TMX_NeoPixel::show() {
  sem_acquire_blocking(&reset_delay_complete_sem); // get semaphore
  dma_channel_set_read_addr(DMA_CHANNEL, (void *)this->pixels,
                            true); // trigger DMA transfer
}

void TMX_NeoPixel::clear() {
  this->fill(0, 0, 0, true);
  this->show();
}

void TMX_NeoPixel::fill(int RED, int GREEN, int BLUE, bool show) {
  for (int i = 0; i < this->LED_count; i++) {
    this->setPixelColor(i, RED, GREEN, BLUE, false);
  }
  if (show) {
    this->show();
  }
}

static int64_t reset_delay_complete(alarm_id_t id, void *user_data) {
  reset_delay_alarm_id = 0;               // reset alarm id
  sem_release(&reset_delay_complete_sem); // release semaphore
  return 0;                               // no repeat
}

void dma_complete_handler(void) {
  if (dma_hw->ints0 & DMA_CHANNEL_MASK) { // are we called for our DMA channel?
    dma_hw->ints0 = DMA_CHANNEL_MASK;     // clear IRQ
    if (reset_delay_alarm_id !=
        0) { // safety check: is there somehow an alarm already running?
      cancel_alarm(reset_delay_alarm_id); // cancel it
    }
    // setup alarm to wait for the required latch-in time for the LES at the end
    // of the transfer
    reset_delay_alarm_id =
        add_alarm_in_us(RESET_TIME_US, reset_delay_complete, NULL, true);
  }
}

static void dma_init(PIO pio, unsigned int sm, uint32_t led_count) {
  dma_claim_mask(
      DMA_CHANNEL_MASK); // check that the DMA channel we want is available
  dma_channel_config channel_config =
      dma_channel_get_default_config(DMA_CHANNEL); // get default configuration
  channel_config_set_dreq(
      &channel_config,
      pio_get_dreq(pio, sm, true)); // configure data request. true: sending
                                    // data to the PIO state machine

  channel_config_set_transfer_data_size(
      &channel_config, DMA_SIZE_32); // data transfer size is 32 bits
  channel_config_set_read_increment(
      &channel_config,
      true); // each read of the data will increase the read pointer
  dma_channel_configure(DMA_CHANNEL, &channel_config,
                        &pio->txf[sm], // write address: write to PIO FIFO
                        NULL,          // don't provide a read address yet
                        led_count,     // number of transfers
                        false);        // don't start yet
  irq_set_exclusive_handler(
      DMA_IRQ_0,
      dma_complete_handler); // after DMA all data, raise an interrupt
  dma_channel_set_irq0_enabled(DMA_CHANNEL,
                               true); // map DMA channel to interrupt
  irq_set_enabled(DMA_IRQ_0, true);   // enable interrupt
}
