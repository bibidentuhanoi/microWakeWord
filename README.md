# ESP32 I2S Audio Library with Wake Word

This project is a standalone C++ library for ESP32, providing solid I2S audio components for microphone, speaker, and wake word detection. The core components are derived from the excellent work done in the [ESPHome project](https://esphome.io/) and have been refactored to be used as a self-contained ESP-IDF library.

## Introduction

This library provides a set of high-level C++ classes to simplify audio processing on the ESP32. It is designed for real-time applications, such as voice assistants, and is built on top of FreeRTOS for non-blocking, concurrent operation.

Key features include:

  - **I2S Microphone:** A subscription-based microphone component for capturing audio data.
  - **I2S Speaker:** A non-blocking speaker component for audio playback.
  - **Micro Wake Word:** A wake word detection engine powered by TensorFlow Lite for Microcontrollers.
  - **Unified Error Handling:** A centralized and robust error reporting system.

## How to Use

To integrate this library, include the necessary headers and instantiate the required components.

### 1\. Component Initialization

First, statically declare the components:

```cpp
#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/micro_wake_word/micro_wake_word.h"

static esphome::i2s_audio::I2SAudioMicrophone microphone;
static esphome::i2s_audio::I2SAudioSpeaker speaker;
static esphome::micro_wake_word::MicroWakeWord wake_word;
```

### 2\. Hardware Configuration

Configure the I/O pins and I2S parameters for each component.

```cpp
// Configure Microphone
microphone.set_bclk_pin(I2S_MIC_BCK_PIN);
microphone.set_lrclk_pin(I2S_MIC_WS_PIN);
microphone.set_din_pin(I2S_MIC_SD_PIN);
microphone.set_sample_rate(16000);
microphone.setup();

// Configure Speaker
speaker.set_bclk_pin(I2S_SPEAKER_BCLK_PIN);
speaker.set_lrclk_pin(I2S_SPEAKER_LRC_PIN);
speaker.set_dout_pin(I2S_SPEAKER_DOUT_PIN);
speaker.set_sample_rate(16000);
speaker.setup();
speaker.start(); // Start the background task
```

### 3\. Wake Word Setup

Configure the wake word engine and link it to the microphone. You can add multiple wake word models by calling `add_wake_word_model` for each one.

```cpp
#include "alexa.h"      // Assumes alexa_tflite
#include "hey_jarvis.h" // Assumes hey_jarvis_tflite

wake_word.set_microphone(&microphone);

// Add the first wake word
wake_word.add_wake_word_model(
    const_cast<uint8_t *>(alexa_tflite), // Model data
    0.97f,        // Confidence threshold
    5,            // Sensitivity (higher = more sensitive)
    "alexa",      // Name (must be unique and is used in the callback)
    22940         // Arena size (must be correct for the model)
);

// Add a second wake word
wake_word.add_wake_word_model(
    const_cast<uint8_t *>(hey_jarvis_tflite),
    0.97f,
    5,
    "hey_jarvis",
    22940
);

wake_word.setup();
```

### 4\. Error Checking

After calling `setup()` on each component, it is important to check for initialization errors.

```cpp
if (microphone.has_error() || speaker.has_error() || wake_word.has_error()) {
    ESP_LOGE(TAG, "Component setup failed! Error: %s", wake_word.get_error_message().c_str());
    // Handle error
}
```

### 5\. Detection Callback

Register a callback function to be executed when a wake word is detected. The `std::string` passed to the callback will be the **name** you assigned in `add_wake_word_model`, allowing you to identify which wake word was triggered.

```cpp
wake_word.add_detection_callback([&](std::string detected_wake_word) {
    ESP_LOGI(TAG, "Wake word detected: %s", detected_wake_word.c_str());

    // --- CAUTION ---
    // If you trigger a new audio task (like recording a voice command),
    // pause the engine first to prevent re-triggers.
    wake_word.pause();

    if (detected_wake_word == "alexa") {
        ESP_LOGI(TAG, "*** alexa DETECTED! ***");
        // Perform action for Alexa
    
    } else if (detected_wake_word == "hey_jarvis") {
        ESP_LOGI(TAG, "*** hey_jarvis DETECTED! ***");
        // Perform action for Jarvis
    }

    // Resume detection once your action is complete
    // (e.g., after recording a voice command or playing a sound)
    wake_word.resume();
});
```

### 6\. Start Detection

Finally, start the wake word detection engine.

```cpp
wake_word.start();
```

## Architecture

The library is designed around a few core principles to ensure stability and modularity in a real-time environment.

  - **Decoupled Components:** Each component (microphone, speaker, wake word) operates independently. Communication is handled through clean interfaces, such as the microphone's publisher/subscriber model.
  - **Non-Blocking Operations:** All I/O-intensive operations are managed in dedicated FreeRTOS tasks. This ensures that the main application logic is never blocked, allowing for responsive applications.
  - **Centralized State and Error Management:** All components inherit from a common base class, `I2SAudioComponent`, which manages a unified state machine (`RUNNING`, `STOPPED`, `ERROR`, etc.) and provides a centralized error reporting system for robust and consistent error handling.

## Future Roadmap (TODO)

  - [ ] Add a guide or Jupyter/Colab notebook for training custom wake word models.
  - [ ] Add more complete examples (e.g., a full voice assistant loop, audio playback examples).