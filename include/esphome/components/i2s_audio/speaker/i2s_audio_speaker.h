// Updated i2s_audio_speaker.h
// i2s_audio_speaker.h
#pragma once

#ifdef USE_ESP32

#include "../i2s_audio.h"
#include "driver/gpio.h"

#include "esphome/core/ring_buffer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace esphome
{
    namespace i2s_audio
    {

        class I2SAudioSpeaker : public I2SAudioComponent
        {
        public:
            void setup();
            void start() override;
            void stop() override;
            size_t write(const uint8_t *data, size_t length, bool finish = true);
            
            // Configuration setters
            void set_dout_pin(gpio_num_t pin) { this->dout_pin_ = pin; }
            void set_sd_pin(gpio_num_t pin) { this->sd_pin_ = pin; }
            void set_channel(i2s_chan_config_t config) { this->channel_config_ = config; }
            void set_sample_rate(uint32_t sample_rate) { this->sample_rate_ = sample_rate; }
            void set_bits_per_sample(i2s_data_bit_width_t bits_per_sample) { this->bits_per_sample_ = bits_per_sample; }
            void set_external_dac(bool external_dac) { this->external_dac_ = external_dac; }
            void set_volume(float volume)
            {
                if (volume < 0.0f)
                    volume = 0.0f;
                if (volume > 1.0f)
                    volume = 1.0f;
                this->volume_ = volume;
            }
            void set_buffer_size(size_t buffer_size) { this->buffer_size_ = buffer_size; }

        protected:
            static void speaker_task(void *param);
            bool finish();
            void interrupt();

            // Existing configuration
            gpio_num_t dout_pin_{I2S_GPIO_UNUSED};
            gpio_num_t sd_pin_{GPIO_NUM_NC};
            bool external_dac_{true};
            i2s_chan_config_t channel_config_;
            i2s_chan_handle_t channel_;
            uint32_t sample_rate_;
            i2s_data_bit_width_t bits_per_sample_;
            float volume_{1.0f};

            // New member variables
            std::unique_ptr<RingBuffer> buffer_;
            size_t buffer_size_{65536};
            TaskHandle_t task_handle_{nullptr};
            SemaphoreHandle_t completion_semaphore_{nullptr};
            SemaphoreHandle_t buffer_mutex_{nullptr};
            SemaphoreHandle_t shutdown_semaphore_{nullptr};
            volatile bool interrupted_{false};
            uint32_t last_write_timestamp_{0};
            // For auto-interrupt: Track last writing task
            TaskHandle_t last_task_handle_{nullptr};
        };

    } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32