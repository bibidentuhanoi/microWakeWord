#include <driver/i2s_std.h>
#include <stdio.h>

#include "esphome/components/i2s_audio/microphone/i2s_audio_microphone.h"
#include "esphome/components/micro_wake_word/micro_wake_word.h"

// tflite model for "Hey Jarvis"
#include "hey_jarvis.h"

// INMP441 microphone
#define I2S_BCK_PIN GPIO_NUM_1
#define I2S_SD_PIN GPIO_NUM_2
#define I2S_WS_PIN GPIO_NUM_3
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE_HZ 16000

bool gDetected = false;

void wakeWordDetected(std::string detected_wake_word);
void wakeWordDetectionTask(void *params);

void wakeWordDetected(std::string detected_wake_word) { gDetected = true; }

void wakeWordDetectionTask(void *params) {
  esphome::i2s_audio::I2SAudioMicrophone microphone;
  esphome::micro_wake_word::MicroWakeWord wakeWord;

  uint8_t *model = const_cast<uint8_t *>(hey_jarvis_tflite);

  // setup and initialize the microphone
  microphone.set_bclk_pin(I2S_BCK_PIN);
  microphone.set_lrclk_pin(I2S_WS_PIN);
  microphone.set_din_pin(I2S_SD_PIN);
  microphone.set_channel(I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER));
  microphone.set_sample_rate(SAMPLE_RATE_HZ);
  microphone.set_bits_per_sample(I2S_DATA_BIT_WIDTH_32BIT);
  wakeWord.set_microphone(&microphone);
  wakeWord.add_wake_word_model(model, 0.97f, 5, "Hey Jarvis", 22940);
  wakeWord.set_features_step_size(10);
  wakeWord.add_detection_callback(&wakeWordDetected);

  wakeWord.setup();
  wakeWord.start();

  for (;;) {
    wakeWord.loop();
    if (gDetected) {
      printf("Wake word detected!\n");
      gDetected = false;
      // start listening again
      wakeWord.start();
    }
  }
}

extern "C" void app_main(void) {
  xTaskCreatePinnedToCore(wakeWordDetectionTask, "wake_word_task", 4096, NULL,
                          2, NULL, 1);
}
