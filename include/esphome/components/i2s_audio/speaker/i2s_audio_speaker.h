#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"

#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"

namespace esphome
{
    namespace i2s_audio
    {

        // Forward declare the Component class's methods to avoid ambiguity
        class I2SAudioSpeaker : public I2SAudioComponent, public speaker::Speaker
        {
        public:
            void setup() override;
            void start() override;
            void stop() override;

            void loop() override;

            void set_dout_pin(gpio_num_t pin) { this->dout_pin_ = pin; }

            size_t write(const uint8_t *data, size_t length) override; // Changed from bool to size_t

            void set_channel(i2s_chan_config_t config) { this->channel_config_ = config; }
            void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
            void set_bits_per_sample(i2s_data_bit_width_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }
            void set_external_dac(bool external_dac) { this->external_dac_ = external_dac; }

            // Explicitly resolve ambiguous method calls by forwarding to the Speaker implementation
            bool is_failed() const { return speaker::Speaker::is_failed(); }

        protected:
            void start_();
            void stop_();
            void write_();

            gpio_num_t dout_pin_{I2S_GPIO_UNUSED};
            bool external_dac_{true}; // Default to external DAC mode
            i2s_chan_config_t channel_config_;
            i2s_chan_handle_t channel_;
            uint32_t sample_rate_;
            i2s_data_bit_width_t bits_per_sample_;

            HighFrequencyLoopRequester high_freq_;
        };

    } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32