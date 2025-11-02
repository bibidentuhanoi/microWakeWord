// Updated i2s_audio_microphone.cpp (key changes: enum references updated to new State values; state_ set to ERROR on error conditions; initial state preserved as STOPPED via parent; minimal adjustments to internal_stop for early state setting)
#include "esphome/components/i2s_audio/microphone/i2s_audio_microphone.h"

#ifdef USE_ESP32

#if SOC_I2S_SUPPORTS_ADC
#include <driver/i2s.h>
#else
#include <driver/i2s_std.h>
#endif
#include <esp_log.h>

namespace esphome {
namespace i2s_audio {

static const size_t BUFFER_SIZE = 512;
static const char *const TAG = "i2s_audio.microphone";

void I2SAudioMicrophone::setup() {
  if (this->primitives_allocated_) return;

  this->callback_mutex_ = xSemaphoreCreateMutex();
  this->control_events_ = xEventGroupCreate();
  if (!this->callback_mutex_ || !this->control_events_) {
    this->report_error(ErrorCode::MIC_PRIMITIVES_ALLOC_FAILED, "Failed to create sync primitives");
    return;
  }
  this->primitives_allocated_ = true;
}

void I2SAudioMicrophone::start() {
  ESP_LOGW(TAG, "Manual start called on microphone; prefer using subscriptions for automatic management.");
  this->internal_start();
}

void I2SAudioMicrophone::stop() {
  ESP_LOGW(TAG, "Manual stop called on microphone; prefer using subscriptions for automatic management.");
  this->internal_stop();
}

size_t I2SAudioMicrophone::read(int16_t *buf, size_t len) {
  ESP_LOGW(TAG, "read() called but deprecated; use data callbacks instead.");
  return 0;
}

size_t I2SAudioMicrophone::subscribe(std::function<void(const int16_t *, size_t)> &&callback) {
  if (!this->primitives_allocated_) {
    this->report_error(ErrorCode::MIC_PRIMITIVES_ALLOC_FAILED, "Primitives not allocated; call setup() first");
    return 0;
  }
  xSemaphoreTake(this->callback_mutex_, portMAX_DELAY);
  size_t id = this->next_subscription_id_++;
  this->data_callbacks_.push_back({id, std::move(callback)});
  if (this->data_callbacks_.size() == 1 && !this->running_) {
    this->internal_start();
  }
  xSemaphoreGive(this->callback_mutex_);
  return id;
}

void I2SAudioMicrophone::unsubscribe(size_t subscription_id) {
  xSemaphoreTake(this->callback_mutex_, portMAX_DELAY);
  auto it = std::find_if(this->data_callbacks_.begin(), this->data_callbacks_.end(),
                         [subscription_id](const Subscription &sub) { return sub.id == subscription_id; });
  if (it != this->data_callbacks_.end()) {
    this->data_callbacks_.erase(it);
    if (this->data_callbacks_.empty() && this->running_) {
      this->internal_stop();
    }
  }
  xSemaphoreGive(this->callback_mutex_);
}

void I2SAudioMicrophone::internal_start() {
  if (this->has_error()) return;
  if (this->is_running()) return;

  if (this->din_pin_ == I2S_GPIO_UNUSED) {
    this->report_error(ErrorCode::I2S_PIN_CONFIG_INVALID, "DIN pin not set");
    return;
  }
#if SOC_I2S_SUPPORTS_ADC
  if (this->adc_) {
    if (this->parent_->get_port() != I2S_NUM_0) {
      this->report_error(ErrorCode::MIC_ADC_CONFIG_INVALID, "Internal ADC only works on I2S0!");
      return;
    }
  }
#endif

  this->state_ = STARTING;

  esp_err_t err;
  ESP_ERROR_CHECK(i2s_new_channel(&this->channel_config_, NULL, &this->channel_));
  i2s_std_config_t rx_std_cfg = {
      .clk_cfg = {
          .sample_rate_hz = this->sample_rate_,
          .clk_src = I2S_CLK_SRC_DEFAULT,
          .mclk_multiple = I2S_MCLK_MULTIPLE_1024,
      },
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(this->bits_per_sample_, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
          .mclk = this->mclk_pin_, 
          .bclk = this->bclk_pin_,
          .ws = this->lrclk_pin_,
          .dout = I2S_GPIO_UNUSED,
          .din = this->din_pin_,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
      }};
  rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
  err = i2s_channel_init_std_mode(this->channel_, &rx_std_cfg);
  if (err != ESP_OK) {
    this->report_error(ErrorCode::I2S_CHANNEL_INIT_FAILED, "Error initializing I2S channel: %s", esp_err_to_name(err));
    return;
  }
  err = i2s_channel_enable(this->channel_);
  if (err != ESP_OK) {
    this->report_error(ErrorCode::I2S_CHANNEL_ENABLE_FAILED, "Error enabling I2S channel: %s", esp_err_to_name(err));
    return;
  }

  this->running_ = true;
  xTaskCreate(I2SAudioMicrophone::mic_reader_task, "mic_read", 4096, this, 18, &this->mic_task_handle_);

  this->state_ = RUNNING;
  ESP_LOGI(TAG, "Microphone started");
}

void I2SAudioMicrophone::internal_stop() {
  if (!this->is_running()) return;

  this->state_ = STOPPING;
  this->running_ = false;
  
  esp_err_t err = i2s_channel_disable(this->channel_);
  if (err != ESP_OK) {
    this->report_error(ErrorCode::I2S_CHANNEL_DISABLE_FAILED, "Error disabling I2S channel: %s", esp_err_to_name(err));
  }

  xEventGroupWaitBits(this->control_events_, DONE_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));

  if (this->channel_ != nullptr) {
      err = i2s_del_channel(this->channel_);
      if (err != ESP_OK) {
          this->report_error(ErrorCode::I2S_CHANNEL_DELETE_FAILED, "Error deleting I2S channel: %s", esp_err_to_name(err));
      }
      this->channel_ = nullptr;
  }
  if (this->state_ != ERROR) {
    this->state_ = STOPPED;
  }
  ESP_LOGI(TAG, "Microphone stopped");
}

void I2SAudioMicrophone::mic_reader_task(void *param) {
  I2SAudioMicrophone *self = static_cast<I2SAudioMicrophone *>(param);
  static uint8_t temp_buffer[1024];
  static int16_t processed[512];

  while (self->running_) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(self->channel_, temp_buffer, self->chunk_size_, &bytes_read, portMAX_DELAY);

    if (err != ESP_OK || !self->running_) {
      if (self->running_) {
          self->report_error(ErrorCode::I2S_READ_FAILED, "I2S read error: %s", esp_err_to_name(err));
      }
      break;
    }
    if (bytes_read == 0) continue;

    const int16_t *samples = nullptr;
    size_t num_samples = 0;

    if (self->bits_per_sample_ == I2S_DATA_BIT_WIDTH_32BIT) {
      num_samples = bytes_read / sizeof(int32_t);
      for (size_t i = 0; i < num_samples; i++) {
        int32_t temp = reinterpret_cast<int32_t *>(temp_buffer)[i] >> 16;
        processed[i] = clamp<int16_t>(temp, INT16_MIN, INT16_MAX);
      }
      samples = processed;
    } else {
      num_samples = bytes_read / sizeof(int16_t);
      samples = reinterpret_cast<const int16_t *>(temp_buffer);
    }

    if (num_samples > 0) {
      xSemaphoreTake(self->callback_mutex_, portMAX_DELAY);
      for (const auto &sub : self->data_callbacks_) {
        sub.callback(samples, num_samples);
      }
      xSemaphoreGive(self->callback_mutex_);
    }
  }
  xEventGroupSetBits(self->control_events_, DONE_EVENT);
  vTaskDelete(NULL);
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32