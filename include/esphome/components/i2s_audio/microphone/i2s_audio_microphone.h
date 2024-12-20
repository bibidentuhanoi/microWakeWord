#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include "esphome/components/microphone/microphone.h"
#include "esphome/core/component.h"

namespace esphome
{
  namespace i2s_audio
  {

    class I2SAudioMicrophone : public I2SAudioComponent, public microphone::Microphone
    {
    public:
      void setup() override;
      void start() override;
      void stop() override;

      void loop() override;

      void set_din_pin(gpio_num_t pin) { this->din_pin_ = pin; }

      size_t read(int16_t *buf, size_t len) override;

#if SOC_I2S_SUPPORTS_ADC
      void set_adc_channel(adc1_channel_t channel)
      {
        this->adc_channel_ = channel;
        this->adc_ = true;
      }
#endif

      void set_channel(i2s_chan_config_t config) { this->channel_config_ = config; }
      void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
      void set_bits_per_sample(i2s_data_bit_width_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }

    protected:
      void start_();
      void stop_();
      void read_();

      gpio_num_t din_pin_{I2S_GPIO_UNUSED};
#if SOC_I2S_SUPPORTS_ADC
      adc1_channel_t adc_channel_{ADC1_CHANNEL_MAX};
      bool adc_{false};
#endif
      i2s_chan_config_t channel_config_;
      i2s_chan_handle_t channel_;
      uint32_t sample_rate_;
      i2s_data_bit_width_t bits_per_sample_;

      HighFrequencyLoopRequester high_freq_;
    };

  } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32
