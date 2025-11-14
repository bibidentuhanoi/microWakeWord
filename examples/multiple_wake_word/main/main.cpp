#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <driver/i2s_std.h>
#include <cmath>
#include <vector>

// Core library components
#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/micro_wake_word/micro_wake_word.h"

// TFLite models
#include "BMO.h"
#include "alexa.h"
#include "hey_jarvis.h"

// ============================= LOGGING TAG =============================
static const char *TAG = "Multi Wake Word Example";

// =========================== HARDWARE CONFIG ===========================
// INMP441 Microphone Pins
#define I2S_MIC_BCK_PIN GPIO_NUM_47 // Bit Clock
#define I2S_MIC_WS_PIN GPIO_NUM_21  // Word Select (or Left/Right Clock)
#define I2S_MIC_SD_PIN GPIO_NUM_48  // Serial Data
#define I2S_MIC_PORT        I2S_NUM_0

// MAX98357A Speaker Pins
#define I2S_SPEAKER_BCLK_PIN GPIO_NUM_12 // Bit Clock
#define I2S_SPEAKER_LRC_PIN GPIO_NUM_13  // Word Select (or Left/Right Clock)
#define I2S_SPEAKER_DOUT_PIN GPIO_NUM_11 // Serial Data Out
#define I2S_SPEAKER_SD_PIN GPIO_NUM_10   // Shutdown
#define I2S_SPEAKER_PORT        I2S_NUM_1

// ======================== AUDIO CONFIGURATION ==========================
// Audio Config
#define SAMPLE_RATE_HZ 16000
#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_32BIT
#define SPEAKER_BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT

// Global Components
static esphome::i2s_audio::I2SAudioMicrophone microphone;
static esphome::i2s_audio::I2SAudioSpeaker speaker;
static esphome::micro_wake_word::MicroWakeWord wake_word;

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== MULTI WAKE WORD EXAMPLE STARTING ===");

    ESP_LOGI(TAG, "Configuring Microphone...");
    microphone.set_bclk_pin(I2S_MIC_BCK_PIN);
    microphone.set_lrclk_pin(I2S_MIC_WS_PIN);
    microphone.set_din_pin(I2S_MIC_SD_PIN);
    microphone.set_channel(I2S_CHANNEL_DEFAULT_CONFIG(I2S_MIC_PORT, I2S_ROLE_MASTER));
    microphone.set_sample_rate(SAMPLE_RATE_HZ);
    microphone.set_bits_per_sample(BITS_PER_SAMPLE);
    microphone.setup();

    ESP_LOGI(TAG, "Configuring Speaker...");
    speaker.set_bclk_pin(I2S_SPEAKER_BCLK_PIN);
    speaker.set_lrclk_pin(I2S_SPEAKER_LRC_PIN);
    speaker.set_dout_pin(I2S_SPEAKER_DOUT_PIN);
    speaker.set_channel(I2S_CHANNEL_DEFAULT_CONFIG(I2S_SPEAKER_PORT, I2S_ROLE_MASTER));
    speaker.set_sample_rate(SAMPLE_RATE_HZ);
    speaker.set_bits_per_sample(SPEAKER_BITS_PER_SAMPLE);
    speaker.setup();
    speaker.set_volume(0.5f);
    speaker.start();

    ESP_LOGI(TAG, "Configuring Wake Word Engine...");
    wake_word.set_microphone(&microphone);
    
    wake_word.add_wake_word_model(const_cast<uint8_t *>(BMO_tflite), 0.97f, 5, "BMO", 25560);
    wake_word.add_wake_word_model(const_cast<uint8_t *>(alexa_tflite), 0.97f, 5, "alexa", 22160);
    wake_word.add_wake_word_model(const_cast<uint8_t *>(hey_jarvis_tflite), 0.97f, 5, "hey_jarvis", 22684);

    wake_word.set_features_step_size(10);
    wake_word.setup();

    if (microphone.has_error()) {
        ESP_LOGE(TAG, "Microphone setup failed! Error: %s", microphone.get_error_message().c_str());
        return;
    }
    if (speaker.has_error()) {
        ESP_LOGE(TAG, "Speaker setup failed! Error: %s", speaker.get_error_message().c_str());
        return;
    }
    if (wake_word.has_error()) {
        ESP_LOGE(TAG, "Wake word setup failed! Error: %s", wake_word.get_error_message().c_str());
        return;
    }

    wake_word.add_detection_callback([&](std::string detected_wake_word) {
        
        // --- CAUTION ---
        // If you trigger a new recording here (e.g., for a voice command),
        // call wake_word.pause() first to prevent re-triggers during the recording.
        // Call wake_word.resume() once your other recording task is finished.

        if (detected_wake_word == "BMO") {
            ESP_LOGI(TAG, "*** BMO DETECTED! ***");
            // Example:
            // wake_word.pause();
            // record_voice_command();
            // wake_word.resume();
        
        } else if (detected_wake_word == "alexa") {
            ESP_LOGI(TAG, "*** alexa DETECTED! ***");
            
        } else if (detected_wake_word == "hey_jarvis") {
            ESP_LOGI(TAG, "*** hey_jarvis DETECTED! ***");
        }
    });

    ESP_LOGI(TAG, "Starting wake word detection...");
    wake_word.start();

    ESP_LOGI(TAG, "Setup complete. System is running.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}