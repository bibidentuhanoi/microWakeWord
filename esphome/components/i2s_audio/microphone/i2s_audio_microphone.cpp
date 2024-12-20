#include "esphome/components/i2s_audio/microphone/i2s_audio_microphone.h"

#ifdef USE_ESP32

#if SOC_I2S_SUPPORTS_ADC
#include <driver/i2s.h>
#else
#include <driver/i2s_std.h>
#endif
#include <esp_log.h>

namespace esphome
{
  namespace i2s_audio
  {

    static const size_t BUFFER_SIZE = 512;

    static const char *const TAG = "i2s_audio.microphone";

    void I2SAudioMicrophone::setup()
    {
      ESP_LOGI(TAG, "Setting up I2S Audio Microphone...");
#if SOC_I2S_SUPPORTS_ADC
      if (this->adc_)
      {
        if (this->parent_->get_port() != I2S_NUM_0)
        {
          ESP_LOGE(TAG, "Internal ADC only works on I2S0!");
          this->mark_failed();
          return;
        }
      }
      else
#endif
    }

    void I2SAudioMicrophone::start()
    {
      if (this->is_failed())
        return;
      if (this->state_ == microphone::STATE_RUNNING)
        return; // Already running
      this->state_ = microphone::STATE_STARTING;
      this->start_();
    }

    void I2SAudioMicrophone::start_()
    {
      if (!this->try_lock())
      {
        return; // Waiting for another i2s to return lock
      }

      esp_err_t err;
#if SOC_I2S_SUPPORTS_ADC
      if (this->adc_)
      {
        config.mode = (i2s_mode_t)(config.mode | I2S_MODE_ADC_BUILT_IN);
        err = i2s_driver_install(this->parent_->get_port(), &config, 0, nullptr);
        if (err != ESP_OK)
        {
          ESP_LOGW(TAG, "Error installing I2S driver: %s", esp_err_to_name(err));
          this->status_set_error();
          return;
        }

        err = i2s_set_adc_mode(ADC_UNIT_1, this->adc_channel_);
        if (err != ESP_OK)
        {
          ESP_LOGW(TAG, "Error setting ADC mode: %s", esp_err_to_name(err));
          this->status_set_error();
          return;
        }
        err = i2s_adc_enable(this->parent_->get_port());
        if (err != ESP_OK)
        {
          ESP_LOGW(TAG, "Error enabling ADC: %s", esp_err_to_name(err));
          this->status_set_error();
          return;
        }
      }
      else
#endif
      {
        i2s_chan_handle_t channel;

        ESP_ERROR_CHECK(i2s_new_channel(&this->channel_config_, NULL, &channel));
        this->channel_ = channel;

        i2s_std_config_t rx_std_cfg = {
            .clk_cfg = {
                .sample_rate_hz = this->sample_rate_,
                .clk_src = I2S_CLK_SRC_DEFAULT,
                .mclk_multiple = I2S_MCLK_MULTIPLE_384,
            },
            .slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG(this->bits_per_sample_, I2S_SLOT_MODE_MONO),
            .gpio_cfg = {
                .mclk = this->mclk_pin_, // some codecs may require mclk signal, this example doesn't need it
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
        if (err != ESP_OK)
        {
          ESP_LOGW(TAG, "Error installing I2S driver: %s", esp_err_to_name(err));
          this->status_set_error();
          return;
        }

        err = i2s_channel_enable(this->channel_);
        if (err != ESP_OK)
        {
          ESP_LOGW(TAG, "Error enabling I2S channel: %s", esp_err_to_name(err));
          this->status_set_error();
          return;
        }
      }
      this->state_ = microphone::STATE_RUNNING;
      this->high_freq_.start();
    }

    void I2SAudioMicrophone::stop()
    {
      if (this->state_ == microphone::STATE_STOPPED || this->is_failed())
        return;
      if (this->state_ == microphone::STATE_STARTING)
      {
        this->state_ = microphone::STATE_STOPPED;
        return;
      }
      this->state_ = microphone::STATE_STOPPING;
      this->stop_();
    }

    void I2SAudioMicrophone::stop_()
    {
      esp_err_t err;
#if SOC_I2S_SUPPORTS_ADC
      if (this->adc_)
      {
        err = i2s_adc_disable(this->parent_->get_port());
        if (err != ESP_OK)
        {
          ESP_LOGW(TAG, "Error disabling ADC: %s", esp_err_to_name(err));
          this->status_set_error();
          return;
        }
      }
#endif
      err = i2s_channel_disable(this->channel_);
      if (err != ESP_OK)
      {
        ESP_LOGW(TAG, "Error disabling I2S microphone: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
      }
      err = i2s_del_channel(this->channel_);
      if (err != ESP_OK)
      {
        ESP_LOGW(TAG, "Error deleting I2S channel: %s", esp_err_to_name(err));
        this->status_set_error();
        return;
      }
      this->unlock();
      this->state_ = microphone::STATE_STOPPED;
      this->high_freq_.stop();
      this->status_clear_error();
    }

    size_t I2SAudioMicrophone::read(int16_t *buf, size_t len)
    {
      size_t bytes_read = 0;
      esp_err_t err = i2s_channel_read(this->channel_, buf, len, &bytes_read, (1000 / portTICK_PERIOD_MS));
      if (err != ESP_OK)
      {
        ESP_LOGW(TAG, "Error reading from I2S microphone: %s", esp_err_to_name(err));
        this->status_set_warning();
        return 0;
      }
      if (bytes_read == 0)
      {
        this->status_set_warning();
        return 0;
      }
      this->status_clear_warning();
      if (this->bits_per_sample_ == I2S_DATA_BIT_WIDTH_16BIT)
      {
        return bytes_read;
      }
      else if (this->bits_per_sample_ == I2S_DATA_BIT_WIDTH_32BIT)
      {
        std::vector<int16_t> samples;
        size_t samples_read = bytes_read / sizeof(int32_t);
        samples.resize(samples_read);
        for (size_t i = 0; i < samples_read; i++)
        {
          int32_t temp = reinterpret_cast<int32_t *>(buf)[i] >> 11;
          samples[i] = clamp<int16_t>(temp, INT16_MIN, INT16_MAX);
        }
        memcpy(buf, samples.data(), samples_read * sizeof(int16_t));
        return samples_read * sizeof(int16_t);
      }
      else
      {
        ESP_LOGE(TAG, "Unsupported bits per sample: %d", this->bits_per_sample_);
        return 0;
      }
    }

    void I2SAudioMicrophone::read_()
    {
      std::vector<int16_t> samples;
      samples.resize(BUFFER_SIZE);
      size_t bytes_read = this->read(samples.data(), BUFFER_SIZE / sizeof(int16_t));
      samples.resize(bytes_read / sizeof(int16_t));
      this->data_callbacks_.call(samples);
    }

    void I2SAudioMicrophone::loop()
    {
      switch (this->state_)
      {
      case microphone::STATE_STOPPED:
        break;
      case microphone::STATE_STARTING:
        this->start_();
        break;
      case microphone::STATE_RUNNING:
        if (this->data_callbacks_.size() > 0)
        {
          this->read_();
        }
        break;
      case microphone::STATE_STOPPING:
        this->stop_();
        break;
      }
    }

  } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32
