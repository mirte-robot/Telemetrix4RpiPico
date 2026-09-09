#include "sensors/sonar.hpp"
#include "Telemetrix4RpiPico.hpp"

// sonar report message

int sonar_report_message[] = {5,
                              SONAR_DISTANCE,
                              SONAR_TRIG_PIN,
                              SONAR_ECHO_PIN,
                              M_WHOLE_VALUE,
                              CM_WHOLE_VALUE};
// static_assert(sonar_report_message[0] == sizeof(sonar_report_message));
sonar_data the_hc_sr04s = {.next_sonar_index = 0,
                           .trigger_timer = {},
                           .trigger_mask = 0,
                           .sonars = {},
                           .mutex = {}};

// number of active sonars
int sonar_count = -1;
//  uint sonar_offset;

void sonar_new() {
  // add the sonar to the sonar struct to be processed within
  // the main loop
  uint trig_pin = command_buffer[SONAR_TRIGGER_PIN];
  uint echo_pin = command_buffer[SONAR_ECHO_PIN];
  //  count == 0 -> 1 sonars
  // count is actually the index...
  if (sonar_count >= (MAX_SONARS - 1)) {
    return;
  }

  //  first cancel timer to not trigger during adding the sonar
  if (sonar_count > -1) {
    // When it's the first sonar, no timer has been added yet
    // When already created one timer, remove it before recreating it at a
    // higher rate.
    cancel_repeating_timer(&the_hc_sr04s.trigger_timer);
  } else {
    mutex_init(&the_hc_sr04s.mutex);
  }
  mutex_enter_blocking(&the_hc_sr04s.mutex);
  sonar_count++;

  the_hc_sr04s.sonars[sonar_count].trig_pin = trig_pin;
  the_hc_sr04s.sonars[sonar_count].echo_pin = echo_pin;
  the_hc_sr04s.sonars[sonar_count].last_time_diff = 0;
  the_hc_sr04s.trigger_mask |= 1ul << trig_pin;
  gpio_init(trig_pin);
  gpio_set_dir(trig_pin, GPIO_OUT);
  if (trig_pin != echo_pin) {
    gpio_init(echo_pin);
    the_hc_sr04s.sonars[sonar_count].single_pin = false;
  } else {
    the_hc_sr04s.sonars[sonar_count].single_pin = true;
  }
  gpio_set_dir(echo_pin, GPIO_IN);
  gpio_set_irq_enabled_with_callback(
      echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &sonar_callback);

  mutex_exit(&the_hc_sr04s.mutex);

  // Add or update timer when adding a new sonar, to trigger at 10Hz/sonar
  int hz = 10 * (sonar_count + 1); // 10 hz per sonar
  // negative timeout means exact delay (rather than delay between callbacks)
  if (!add_repeating_timer_ms(1000 / hz, sonar_timer_callback, NULL,
                              &the_hc_sr04s.trigger_timer)) {
    return;
  }
}

void sonar_callback(uint gpio, uint32_t events) {
  auto time = time_us_32(); // record time at start, then processing wont have
                            // any influence.
  if (events & GPIO_IRQ_EDGE_FALL) {
    // stop time
    for (int i = 0; i <= sonar_count; i++) {
      hc_sr04_descriptor *sonar = &the_hc_sr04s.sonars[i];
      if (gpio == sonar->echo_pin) {
        if (!mutex_try_enter(
                &the_hc_sr04s.mutex,
                NULL)) { // if mutex is locked, return, discard this reading
          return;
        }
        // only need mutex on this update, as start_time is only read/updated in
        // this interrupt
        sonar->last_time_diff = time - sonar->start_time;
        mutex_exit(&the_hc_sr04s.mutex);
        return;
      }
    }
  } else if (events & GPIO_IRQ_EDGE_RISE) {
    // start time
    for (int i = 0; i <= sonar_count; i++) {
      hc_sr04_descriptor *sonar = &the_hc_sr04s.sonars[i];
      if (gpio == sonar->echo_pin) {
        sonar->start_time = time;
        return;
      }
    }
  }
}

bool sonar_timer_callback(repeating_timer_t *rt) {
  (void)rt;
  // every interrupt, trigger one sonar and increase counter for next round.
  // results in 10Hz per sonar, without crosstalk.
  static uint8_t sonar_counter = 0;
  auto sonar_pin = the_hc_sr04s.sonars[sonar_counter].trig_pin;
  auto sonar_single = the_hc_sr04s.sonars[sonar_counter].single_pin;
  sonar_counter++;
  if (sonar_counter > sonar_count) {
    sonar_counter = 0;
  }

  if (sonar_single) {
    gpio_set_dir(sonar_pin, GPIO_OUT);
  }
  gpio_put(sonar_pin, 1);
  busy_wait_us(10); // TODO maybe: change this timer to other core to not block
  gpio_put(sonar_pin, 0);
  if (sonar_single) {
    gpio_set_dir(sonar_pin, GPIO_IN);
  }
  return true;
}

void scan_sonars() {
  uint32_t current_time = time_us_32();
  static uint32_t last_scan = 0;
  if (current_time - last_scan < 100'000) {
    return; // Only update at 10Hz.
  }
  if (sonar_count <
      0) { // mutex not initialized and don't need to scan the empty list
    last_scan += 100'000;
    return;
  }

  if (!mutex_try_enter(&the_hc_sr04s.mutex, NULL)) {
    // also don't update the last_scan variable
    return;
  }
  last_scan += 100'000;
  for (int i = 0; i <= sonar_count; i++) {
    hc_sr04_descriptor *sonar = &the_hc_sr04s.sonars[i];

    uint16_t distance = 0xFFFF;
    if (sonar->last_time_diff == (uint32_t)-1) {
      // No update
      if (gpio_get(sonar->echo_pin) == 1) {
        // echo pin high, still waiting for response
        continue;
      }
      distance = 0xFFFC; // error value
    } else if ((current_time - sonar->start_time) >
               1'000'000) // if too long since last trigger, send 0
    {
      distance = 0xFFFE; // must result in NaN in the computer
    } else if (sonar->last_time_diff > 30'000) {
      // HC-SR04 has max range of 4 / 5m, with a
      // timeout pulse longer than 35ms
      distance = 0xFFFD; // must result in +Inf in the computer
    } else {
      // 1cm increments
      distance = (sonar->last_time_diff) / (58);
    }
    send_debug_info(4, sonar->last_time_diff);
    if (distance == sonar->last_dist) {
      continue;
    }
    sonar->last_dist = distance;

    sonar_report_message[SONAR_TRIG_PIN] = (uint8_t)sonar->trig_pin;
    sonar_report_message[SONAR_ECH_PIN] = (uint8_t)sonar->echo_pin;
    sonar_report_message[M_WHOLE_VALUE] =
        (uint8_t)(distance >> 8); // high byte, not M
    sonar_report_message[CM_WHOLE_VALUE] =
        (distance) & 0xFF; // low byte, not CM anymore
    static_assert(
        sizeof(sonar_report_message) / sizeof(sonar_report_message[0]) == 6);
    serial_write(sonar_report_message, 6);
    sonar->last_time_diff = -1; // signal for next loop that there's no new data
  }
  mutex_exit(&the_hc_sr04s.mutex);
}