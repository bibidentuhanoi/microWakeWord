#include "esphome/components/i2s_audio/speaker/i2s_audio_speaker.h"

#ifdef USE_ESP32

#include <driver/i2s_std.h>
#include <esp_log.h>

namespace esphome
{
    namespace i2s_audio
    {

        static const char *const TAG = "i2s_audio.speaker";
        constexpr size_t VOLUME_CHUNK_SAMPLES = 256;  // 2.67ms @ 48kHz (optimal stack size)
        constexpr size_t VOLUME_CHUNK_BYTES = VOLUME_CHUNK_SAMPLES * sizeof(int16_t);
        void I2SAudioSpeaker::setup()
        {
            ESP_LOGI(TAG, "Setting up I2S Audio Speaker...");
        }

        void I2SAudioSpeaker::start()
        {
            if (this->task_handle_ != nullptr || speaker::Speaker::is_failed())
            {
                return; // Already running or in a failed state
            }

            ESP_LOGD(TAG, "Starting speaker task...");

            this->buffer_ = RingBuffer::create(this->buffer_size_);
            if (!this->buffer_)
            {
                ESP_LOGE(TAG, "Failed to create ring buffer");
                speaker::Speaker::status_set_error();
                return;
            }

            this->completion_semaphore_ = xSemaphoreCreateBinary();
            if (this->completion_semaphore_ == nullptr)
            {
                ESP_LOGE(TAG, "Failed to create completion semaphore");
                this->buffer_.reset();
                speaker::Speaker::status_set_error();
                return;
            }

            this->buffer_mutex_ = xSemaphoreCreateMutex();
            if (this->buffer_mutex_ == nullptr)
            {
                ESP_LOGE(TAG, "Failed to create buffer mutex");
                vSemaphoreDelete(this->completion_semaphore_);
                this->buffer_.reset();
                speaker::Speaker::status_set_error();
                return;
            }

            this->shutdown_semaphore_ = xSemaphoreCreateBinary();
            if (this->shutdown_semaphore_ == nullptr) {
                ESP_LOGE(TAG, "Failed to create shutdown semaphore");
                vSemaphoreDelete(this->buffer_mutex_);
                vSemaphoreDelete(this->completion_semaphore_);
                this->buffer_.reset();
                speaker::Speaker::status_set_error();
                return;
            }

            this->state_ = STATE_IDLE;

            BaseType_t result = xTaskCreate(
                speaker_task,
                "speaker_task",
                4096, // Stack size in words
                this, // Task parameter
                tskIDLE_PRIORITY + 2,
                &this->task_handle_);

            if (result != pdPASS)
            {
                this->task_handle_ = nullptr;
                ESP_LOGE(TAG, "Failed to create speaker task");
                vSemaphoreDelete(this->buffer_mutex_);
                vSemaphoreDelete(this->completion_semaphore_);
                this->buffer_.reset();
                speaker::Speaker::status_set_error();
            }
        }

        void I2SAudioSpeaker::stop()
        {
            if (this->task_handle_ == nullptr)
            {
                return; // Not running
            }

            ESP_LOGD(TAG, "Stopping speaker task...");

            this->state_ = STATE_STOPPING;

            // Wait for the task to confirm shutdown
            if (xSemaphoreTake(this->shutdown_semaphore_, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGW(TAG, "Speaker task did not shut down cleanly.");
                // If timeout occurs, forcefully delete the task
                if (this->task_handle_ != nullptr) {
                    vTaskDelete(this->task_handle_);
                    this->task_handle_ = nullptr;
                }
            }


            if (this->completion_semaphore_ != nullptr)
            {
                vSemaphoreDelete(this->completion_semaphore_);
                this->completion_semaphore_ = nullptr;
            }
            if (this->buffer_mutex_ != nullptr)
            {
                vSemaphoreDelete(this->buffer_mutex_);
                this->buffer_mutex_ = nullptr;
            }
            this->buffer_.reset();

            ESP_LOGD(TAG, "Speaker task stopped.");
            this->state_ = STATE_IDLE;
            speaker::Speaker::status_clear_error();
        }

        size_t I2SAudioSpeaker::write(const uint8_t *data, size_t length)
        {
            if (this->state_ == STATE_STOPPING || this->task_handle_ == nullptr)
            {
                return 0;
            }

            if (xSemaphoreTake(this->buffer_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
            {
                ESP_LOGW(TAG, "Failed to acquire buffer mutex in write");
                return 0;
            }

            size_t written = this->buffer_->write(const_cast<uint8_t *>(data), length);
            if (written > 0)
            {
                this->last_write_timestamp_ = millis();
            }

            xSemaphoreGive(this->buffer_mutex_);
            return written;
        }

        void I2SAudioSpeaker::interrupt()
        {
            if (this->state_ == STATE_STOPPING || this->task_handle_ == nullptr)
            {
                return;
            }

            if (xSemaphoreTake(this->buffer_mutex_, portMAX_DELAY) == pdTRUE)
            {
                ESP_LOGD(TAG, "Interrupt called; buffer available before reset: %d bytes", this->buffer_->available());
                this->interrupted_ = true;
                this->buffer_->reset();

                // Flush any pending DMA by writing silence (ensures no I2S hardware leftovers)
                std::vector<uint8_t> silence(VOLUME_CHUNK_BYTES * 2, 0);  // Small silence chunk
                size_t bytes_written = 0;
                i2s_channel_write(this->channel_, silence.data(), silence.size(), &bytes_written, pdMS_TO_TICKS(10));
                ESP_LOGD(TAG, "DMA flushed with %d silence bytes", bytes_written);

                xSemaphoreGive(this->buffer_mutex_);
            }
            xSemaphoreGive(this->completion_semaphore_);
        }

        bool I2SAudioSpeaker::finish()
        {
            if (this->state_ == STATE_STOPPING || this->task_handle_ == nullptr)
            {
                return false;
            }

            
            this->interrupted_ = false;

            // Clear any stale semaphore signal from a previous playback
            xSemaphoreTake(this->completion_semaphore_, 0);
            
            // Wait for the semaphore, which is given on completion or interrupt
            xSemaphoreTake(this->completion_semaphore_, portMAX_DELAY);
            
            return !this->interrupted_;
        }

        void I2SAudioSpeaker::loop()
        {
            // No-op, all logic is in the background task now
        }
            void I2SAudioSpeaker::speaker_task(void *param)
            {
                auto *instance = static_cast<I2SAudioSpeaker *>(param);

                if (!instance->try_lock())
                {
                    ESP_LOGE(TAG, "Failed to lock I2S peripheral");
                    instance->task_handle_ = nullptr;
                    vTaskDelete(NULL);
                    return;
                }

                // I2S Hardware Initialization
                if (instance->sd_pin_ != GPIO_NUM_NC)
                {
                    gpio_reset_pin(instance->sd_pin_);
                    gpio_set_direction(instance->sd_pin_, GPIO_MODE_OUTPUT);
                    gpio_set_level(instance->sd_pin_, 0); // Start with amplifier off
                }

                esp_err_t err;
                ESP_ERROR_CHECK(i2s_new_channel(&instance->channel_config_, &instance->channel_, NULL));

                i2s_std_config_t tx_std_cfg = {
                    .clk_cfg = {
                        .sample_rate_hz = instance->sample_rate_,
                        .clk_src = I2S_CLK_SRC_DEFAULT,
                        .ext_clk_freq_hz = 0,  // Explicitly initialize
                        .mclk_multiple = I2S_MCLK_MULTIPLE_256,  // Changed to 256 for better 16kHz support
                    },
                    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(instance->bits_per_sample_, I2S_SLOT_MODE_STEREO),  // Changed to STEREO to avoid mono swap bug
                    .gpio_cfg = {
                        .mclk = instance->mclk_pin_,
                        .bclk = instance->bclk_pin_,
                        .ws = instance->lrclk_pin_,
                        .dout = instance->dout_pin_,
                        .din = I2S_GPIO_UNUSED,
                        .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                    }};
                tx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;  // Use both slots (duplicate mono to stereo)

                err = i2s_channel_init_std_mode(instance->channel_, &tx_std_cfg);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "Error installing I2S driver: %s", esp_err_to_name(err));
                    instance->speaker::Speaker::status_set_error();
                    instance->unlock();
                    instance->task_handle_ = nullptr;
                    vTaskDelete(NULL);
                    return;
                }

                err = i2s_channel_enable(instance->channel_);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "Error enabling I2S channel: %s", esp_err_to_name(err));
                    instance->speaker::Speaker::status_set_error();
                    instance->unlock();
                    instance->task_handle_ = nullptr;
                    vTaskDelete(NULL);
                    return;
                }
                
                std::vector<uint8_t> chunk_buffer(VOLUME_CHUNK_BYTES);
                std::vector<uint8_t> silent_chunk(VOLUME_CHUNK_BYTES, 0);

                // Main Task Loop
                while (instance->state_ != STATE_STOPPING)
                {
                    size_t bytes_read = instance->buffer_->read(chunk_buffer.data(), chunk_buffer.size(), pdMS_TO_TICKS(10));

                    if (instance->interrupted_)
                    {
                        bytes_read = 0;
                        instance->interrupted_ = false; 
                    }

                    if (bytes_read > 0)
                    {
                        if (instance->state_ == STATE_IDLE)
                        {
                            instance->state_ = STATE_PLAYING;
                            if (instance->sd_pin_ != GPIO_NUM_NC)
                            {
                                gpio_set_level(instance->sd_pin_, 1);
                                vTaskDelay(pdMS_TO_TICKS(10));
                            }
                        }
                        
                        // Volume scaling logic (on mono data)
                        if (instance->volume_ < 0.99f) {
                            int16_t *samples = (int16_t *)chunk_buffer.data();
                            size_t num_samples = bytes_read / sizeof(int16_t);
                            if (instance->volume_ <= 0.0f) {
                                memset(samples, 0, bytes_read);
                            } else {
                                int32_t scaled_volume = static_cast<int32_t>(instance->volume_ * 32768.0f);
                                for (size_t i = 0; i < num_samples; i++) {
                                    int32_t sample = static_cast<int32_t>(samples[i]) * scaled_volume;
                                    sample = (sample + 16384) >> 15;
                                    samples[i] = (sample > 32767) ? 32767 : ((sample < -32768) ? -32768 : static_cast<int16_t>(sample));
                                }
                            }
                        }

                        // Duplicate mono to stereo to avoid ESP-IDF mono swap bug
                        size_t mono_bytes = bytes_read;
                        std::vector<uint8_t> stereo_chunk(mono_bytes * 2);
                        int16_t *mono_samples = (int16_t *)chunk_buffer.data();
                        int16_t *stereo_samples = (int16_t *)stereo_chunk.data();
                        size_t num_samples = mono_bytes / sizeof(int16_t);
                        for (size_t i = 0; i < num_samples; ++i) {
                            stereo_samples[i * 2] = mono_samples[i];      // Left channel
                            stereo_samples[i * 2 + 1] = mono_samples[i];  // Right channel
                        }

                        size_t bytes_written = 0;
                        i2s_channel_write(instance->channel_, stereo_chunk.data(), mono_bytes * 2, &bytes_written, portMAX_DELAY);
                        ESP_LOGD(TAG, "Wrote %d stereo bytes (from %d mono)", bytes_written, mono_bytes);  // Add for debug
                    }
                    else
                    {
                        if (instance->state_ == STATE_PLAYING && (millis() - instance->last_write_timestamp_ > 200))
                        {
                            ESP_LOGD(TAG, "Buffer idle; starting extended silence flush to prevent pop/repeat");
                            
                            // Extended flush: Write silence in loops for ~500ms to fully drain DMA and fade amp
                            const size_t flush_duration_ms = 500;
                            const size_t silence_per_loop = silent_chunk.size();  // ~128ms stereo
                            size_t total_flushed = 0;
                            auto start_time = millis();
                            while ((millis() - start_time) < flush_duration_ms) {
                                size_t bytes_written = 0;
                                i2s_channel_write(instance->channel_, silent_chunk.data(), silence_per_loop, &bytes_written, pdMS_TO_TICKS(10));
                                total_flushed += bytes_written;
                                if (bytes_written < silence_per_loop) break;  // I2S stalled
                                vTaskDelay(pdMS_TO_TICKS(1));  // Yield
                            }
                            ESP_LOGD(TAG, "Flushed %lu silence bytes over %lu ms", static_cast<unsigned long>(total_flushed), static_cast<unsigned long>(millis() - start_time));

                            if (instance->sd_pin_ != GPIO_NUM_NC)
                            {
                                gpio_set_level(instance->sd_pin_, 0);
                            }
                            instance->state_ = STATE_IDLE;
                            xSemaphoreGive(instance->completion_semaphore_);
                        }
                    }
                }

                // Task Teardown
                i2s_channel_disable(instance->channel_);
                if (instance->sd_pin_ != GPIO_NUM_NC)
                {
                    gpio_set_level(instance->sd_pin_, 0);
                }
                i2s_del_channel(instance->channel_);
                instance->unlock();
                xSemaphoreGive(instance->shutdown_semaphore_);
                instance->task_handle_ = nullptr;
                vTaskDelete(NULL);
            }

    } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32
