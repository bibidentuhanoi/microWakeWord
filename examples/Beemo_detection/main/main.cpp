#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <driver/i2s_std.h>
#include <cmath>
#include <vector>

// Core library components
#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/micro_wake_word/micro_wake_word.h"

// tflite model for "Beemo"
#include "BMO.h"

// ============================= LOGGING TAG =============================
static const char *TAG = "Beemon Wake Word Example";


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
#define SAMPLE_RATE_HZ 16000
#define BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_32BIT // For microphone
#define SPEAKER_BITS_PER_SAMPLE I2S_DATA_BIT_WIDTH_16BIT // For speaker

// ======================== GLOBAL COMPONENTS ============================
// Use static to ensure they are allocated once and exist for the life of the app.
static esphome::i2s_audio::I2SAudioMicrophone microphone;
static esphome::i2s_audio::I2SAudioSpeaker speaker;
static esphome::micro_wake_word::MicroWakeWord wake_word;

// Simple function to play a tone on the speaker
void play_activation_tone(esphome::i2s_audio::I2SAudioSpeaker &spkr) {
    const int duration_ms = 200;
    const int frequency = 440; // A4 note
    const int volume = 8192; // 25% of 16-bit max
    const size_t num_samples = (SAMPLE_RATE_HZ * duration_ms) / 1000;
    
    std::vector<int16_t> buffer(num_samples);

    for (size_t i = 0; i < num_samples; ++i) {
        double angle = 2.0 * M_PI * frequency * i / SAMPLE_RATE_HZ;
        buffer[i] = static_cast<int16_t>(volume * sin(angle));
    }

    ESP_LOGI(TAG, "Playing activation tone...");
    spkr.write(reinterpret_cast<const uint8_t*>(buffer.data()), buffer.size() * sizeof(int16_t));
    ESP_LOGI(TAG, "Tone finished.");
}


// ============================= APP MAIN ================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== WAKE WORD EXAMPLE STARTING ===");

    // 1. CONFIGURE MICROPHONE
    ESP_LOGI(TAG, "Configuring Microphone...");
    microphone.set_bclk_pin(I2S_MIC_BCK_PIN);
    microphone.set_lrclk_pin(I2S_MIC_WS_PIN);
    microphone.set_din_pin(I2S_MIC_SD_PIN);
    microphone.set_channel(I2S_CHANNEL_DEFAULT_CONFIG(I2S_MIC_PORT, I2S_ROLE_MASTER));
    microphone.set_sample_rate(SAMPLE_RATE_HZ);
    microphone.set_bits_per_sample(BITS_PER_SAMPLE);
    microphone.setup();

    // 2. CONFIGURE SPEAKER
    ESP_LOGI(TAG, "Configuring Speaker...");
    speaker.set_bclk_pin(I2S_SPEAKER_BCLK_PIN);
    speaker.set_lrclk_pin(I2S_SPEAKER_LRC_PIN);
    speaker.set_dout_pin(I2S_SPEAKER_DOUT_PIN);
    speaker.set_sd_pin(I2S_SPEAKER_SD_PIN);
    speaker.set_channel(I2S_CHANNEL_DEFAULT_CONFIG(I2S_SPEAKER_PORT, I2S_ROLE_MASTER));
    speaker.set_sample_rate(SAMPLE_RATE_HZ);
    speaker.set_bits_per_sample(SPEAKER_BITS_PER_SAMPLE);
    speaker.setup();
    speaker.set_volume(0.5f); // Set volume to 50%
    speaker.start(); // Start the speaker task immediately

    // 3. CONFIGURE WAKE WORD
    ESP_LOGI(TAG, "Configuring Wake Word Engine...");
    wake_word.set_microphone(&microphone);
    wake_word.add_wake_word_model(const_cast<uint8_t *>(BMO_tflite), 0.97f, 5, "BMO", 25556);
    wake_word.set_features_step_size(10);
    wake_word.setup();

    // 4. CHECK FOR CONFIGURATION ERRORS
    if (microphone.has_error()) {
        ESP_LOGE(TAG, "Microphone setup failed! Error: %s", microphone.get_error_message().c_str());
        return; // Halt on error
    }
    if (speaker.has_error()) {
        ESP_LOGE(TAG, "Speaker setup failed! Error: %s", speaker.get_error_message().c_str());
        return; // Halt on error
    }
    if (wake_word.has_error()) {
        ESP_LOGE(TAG, "Wake word setup failed! Error: %s", wake_word.get_error_message().c_str());
        return; // Halt on error
    }

    // 5. DEFINE THE DETECTION CALLBACK
    wake_word.add_detection_callback([&](std::string detected_wake_word) {
        ESP_LOGI(TAG, "Wake word '%s' DETECTED!", detected_wake_word.c_str());
        
        // When the wake word is detected, pause the detection...
        wake_word.pause();
        
        // ...play a confirmation sound...
        play_activation_tone(speaker);

        // ...and then resume detection.
        ESP_LOGI(TAG, "Resuming wake word detection.");
        wake_word.resume();
    });

    // 6. START THE WAKE WORD ENGINE
    ESP_LOGI(TAG, "Starting wake word detection...");
    wake_word.start();

    ESP_LOGI(TAG, "Setup complete. System is running and listening for 'Beemo'.");

    // Keep the main task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
