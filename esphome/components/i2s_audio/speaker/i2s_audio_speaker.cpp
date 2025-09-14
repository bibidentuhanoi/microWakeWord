#include "esphome/components/i2s_audio/speaker/i2s_audio_speaker.h"

#ifdef USE_ESP32

#include <driver/i2s_std.h>
#include <esp_log.h>

namespace esphome
{
    namespace i2s_audio
    {

        static const char *const TAG = "i2s_audio.speaker";
        constexpr size_t VOLUME_CHUNK_SAMPLES = 128;  // 2.67ms @ 48kHz (optimal stack size)
        constexpr size_t VOLUME_CHUNK_BYTES = VOLUME_CHUNK_SAMPLES * sizeof(int16_t);
        void I2SAudioSpeaker::setup()
        {
            ESP_LOGI(TAG, "Setting up I2S Audio Speaker...");
        }
        void I2SAudioSpeaker::start()
        {
            if (speaker::Speaker::is_failed())
                return;
            if (this->state_ == speaker::STATE_RUNNING)
                return; // Already running
            this->state_ = speaker::STATE_STARTING;
            this->start_();
        }
        void I2SAudioSpeaker::start_()
        {
            if (!this->try_lock())
            {
                return; // Waiting for another i2s to return lock
            }

            if (this->sd_pin_ != GPIO_NUM_NC) {
                gpio_reset_pin(this->sd_pin_);
                gpio_set_direction(this->sd_pin_ , GPIO_MODE_OUTPUT);
                gpio_set_level(this->sd_pin_, 1);

            }
            esp_err_t err;
            i2s_chan_handle_t channel;

            ESP_ERROR_CHECK(i2s_new_channel(&this->channel_config_, &channel, NULL));
            this->channel_ = channel;

            i2s_std_config_t tx_std_cfg = {
                .clk_cfg = {
                    .sample_rate_hz = this->sample_rate_,
                    .clk_src = I2S_CLK_SRC_DEFAULT,
                    .mclk_multiple = I2S_MCLK_MULTIPLE_1024, // CHANGED FROM 1024 TO 384
                },
                .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(this->bits_per_sample_, I2S_SLOT_MODE_MONO),
                .gpio_cfg = {
                    .mclk = this->mclk_pin_,
                    .bclk = this->bclk_pin_,
                    .ws = this->lrclk_pin_,
                    .dout = this->dout_pin_,
                    .din = I2S_GPIO_UNUSED,
                    .invert_flags = {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
                }};

            tx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
            err = i2s_channel_init_std_mode(this->channel_, &tx_std_cfg);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error installing I2S driver: %s", esp_err_to_name(err));
                speaker::Speaker::status_set_error();
                return;
            }

            err = i2s_channel_enable(this->channel_);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error enabling I2S channel: %s", esp_err_to_name(err));
                speaker::Speaker::status_set_error();
                return;
            }

            this->state_ = speaker::STATE_RUNNING;
            this->high_freq_.start();
        }

        void I2SAudioSpeaker::stop()
        {
            if (this->state_ == speaker::STATE_STOPPED || speaker::Speaker::is_failed())
                return;
            if (this->state_ == speaker::STATE_STARTING)
            {
                this->state_ = speaker::STATE_STOPPED;
                return;
            }
            this->state_ = speaker::STATE_STOPPING;
            this->stop_();
        }

        void I2SAudioSpeaker::stop_()
        {
            esp_err_t err;

            err = i2s_channel_disable(this->channel_);
            // Disable amplifier BEFORE channel cleanup (critical!)
            if (this->sd_pin_ != GPIO_NUM_NC) {
                gpio_set_level(this->sd_pin_, 0);
            }
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error disabling I2S speaker: %s", esp_err_to_name(err));
                speaker::Speaker::status_set_error();
                return;
            }

            err = i2s_del_channel(this->channel_);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "Error deleting I2S channel: %s", esp_err_to_name(err));
                speaker::Speaker::status_set_error();
                return;
            }

            this->unlock();
            this->state_ = speaker::STATE_STOPPED;
            this->high_freq_.stop();
            speaker::Speaker::status_clear_error();
        }
        size_t I2SAudioSpeaker::write(const uint8_t *data, size_t length)
        {
            if (this->state_ != speaker::STATE_RUNNING)
                return 0;

            // Full volume optimization - bypass processing
            if (this->volume_ >= 0.99f) {
                size_t bytes_written = 0;
                esp_err_t err = i2s_channel_write(this->channel_, data, length, &bytes_written, portMAX_DELAY);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "I2S write error: %s", esp_err_to_name(err));
                    speaker::Speaker::status_set_warning();
                    return 0;
                }
                speaker::Speaker::status_clear_warning();
                return bytes_written;
            }

            // Volume processing with stack-based chunking
            size_t total_written = 0;
            size_t remaining = length;
            const uint8_t *src = data;
            int16_t chunk_buffer[VOLUME_CHUNK_SAMPLES];  // Fixed stack allocation (256 bytes)

            while (remaining > 0) {
                // Determine chunk size (even byte count for 16-bit samples)
                size_t chunk_size = (remaining < VOLUME_CHUNK_BYTES) ? 
                                    (remaining & ~1) : VOLUME_CHUNK_BYTES;
                if (chunk_size == 0) break;  // Ensure even byte count

                size_t num_samples = chunk_size / sizeof(int16_t);
                size_t bytes_written = 0;

                if (this->volume_ <= 0.0f) {
                    // Mute optimization - write zeros directly
                    memset(chunk_buffer, 0, chunk_size);
                    esp_err_t err = i2s_channel_write(this->channel_, 
                                                     (uint8_t*)chunk_buffer, 
                                                     chunk_size, 
                                                     &bytes_written, 
                                                     portMAX_DELAY);
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "Zero write error: %s", esp_err_to_name(err));
                        break;
                    }
                } else {
                    // Volume scaling with in-place processing
                    memcpy(chunk_buffer, src, chunk_size);
                    
                    // Fast integer volume scaling (avoids float per-sample)
                    int32_t scaled_volume = static_cast<int32_t>(this->volume_ * 32768.0f);
                    
                    for (size_t i = 0; i < num_samples; i++) {
                        int32_t sample = static_cast<int32_t>(chunk_buffer[i]) * scaled_volume;
                        sample = (sample + 16384) >> 15;  // Rounding shift
                        chunk_buffer[i] = (sample > 32767) ? 32767 : 
                                         ((sample < -32768) ? -32768 : static_cast<int16_t>(sample));
                    }

                    esp_err_t err = i2s_channel_write(this->channel_, 
                                                     (uint8_t*)chunk_buffer, 
                                                     chunk_size, 
                                                     &bytes_written, 
                                                     portMAX_DELAY);
                    if (err != ESP_OK) {
                        ESP_LOGW(TAG, "Scaled write error: %s", esp_err_to_name(err));
                        break;
                    }
                }

                total_written += bytes_written;
                src += bytes_written;
                remaining -= bytes_written;

                // Exit if partial write occurred
                if (bytes_written < chunk_size) break;
            }

            if (total_written > 0) {
                speaker::Speaker::status_clear_warning();
            } else {
                speaker::Speaker::status_set_warning();
            }
            return total_written;
        }

        void I2SAudioSpeaker::write_()
        {
            // This is a placeholder for any periodic write operations
            // For a speaker component, most writing will be driven by incoming data calls
        }

        void I2SAudioSpeaker::loop()
        {
            switch (this->state_)
            {
            case speaker::STATE_STOPPED:
                break;
            case speaker::STATE_STARTING:
                this->start_();
                break;
            case speaker::STATE_RUNNING:
                // For speakers, most activity happens during write() calls
                // But we could add any periodic tasks here
                break;
            case speaker::STATE_STOPPING:
                this->stop_();
                break;
            }
        }

    } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32
