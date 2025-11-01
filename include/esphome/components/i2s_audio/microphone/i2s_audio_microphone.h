// Updated i2s_audio_microphone.h (minimal change: no functional changes, just inherits the updated enum)
#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include "esphome/core/helpers.h"      // For clamp, etc.

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include <vector>
#include <functional>

namespace esphome {
namespace i2s_audio {

class I2SAudioMicrophone : public I2SAudioComponent {
 public:
  void setup();
  void start() override;  // Logs warning; use subscriptions for auto-start
  void stop() override;   // Logs warning; use subscriptions for auto-stop

  void set_din_pin(gpio_num_t pin) { this->din_pin_ = pin; }

  size_t read(int16_t *buf, size_t len);

#if SOC_I2S_SUPPORTS_ADC
  void set_adc_channel(adc1_channel_t channel) {
    this->adc_channel_ = channel;
    this->adc_ = true;
  }
#endif

  void set_channel(i2s_chan_config_t config) { this->channel_config_ = config; }
  void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
  void set_bits_per_sample(i2s_data_bit_width_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }

  // New pub-sub interface
  size_t subscribe(std::function<void(const int16_t *, size_t)> &&callback);
  void unsubscribe(size_t subscription_id);

 protected:
  void internal_start();
  void internal_stop();

  gpio_num_t din_pin_{I2S_GPIO_UNUSED};
#if SOC_I2S_SUPPORTS_ADC
  adc1_channel_t adc_channel_{ADC1_CHANNEL_MAX};
  bool adc_{false};
#endif
  i2s_chan_config_t channel_config_;
  i2s_chan_handle_t channel_;
  uint32_t sample_rate_;
  i2s_data_bit_width_t bits_per_sample_;

  // Primitives
  SemaphoreHandle_t callback_mutex_{nullptr};  // Repurposed from access_mutex_
  EventGroupHandle_t control_events_{nullptr};
  TaskHandle_t mic_task_handle_{nullptr};
  volatile bool running_{false};
  bool primitives_allocated_{false};  // Track if sem/mutex/etc. allocated
  size_t chunk_size_{1024};  // Bytes read from I2S per iteration

#define STOP_EVENT BIT0
#define DONE_EVENT BIT1

  // Callback management
  struct Subscription {
    size_t id;
    std::function<void(const int16_t *, size_t)> callback;  // num_samples
  };
  std::vector<Subscription> data_callbacks_;
  size_t next_subscription_id_{1};

  static void mic_reader_task(void *param);
};

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32