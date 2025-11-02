# ESP32 I2S Audio Library with Wake Word

This project is a standalone C++ library for ESP32, providing solid I2S audio components for microphone, speaker, and wake word detection. The core components are derived from the excellent work done in the [ESPHome project](https://esphome.io/) and have been refactored to be used as a self-contained ESP-IDF library.

## Introduction

This library provides a set of high-level C++ classes to simplify audio processing on the ESP32. It is designed for real-time applications, such as voice assistants, and is built on top of FreeRTOS for non-blocking, concurrent operation.

Key features include:
-   **I2S Microphone:** A subscription-based microphone component for capturing audio data.
-   **I2S Speaker:** A non-blocking speaker component for audio playback.
-   **Micro Wake Word:** A wake word detection engine powered by TensorFlow Lite for Microcontrollers.
-   **Unified Error Handling:** A centralized and robust error reporting system.

## How to Use

To integrate this library, include the necessary headers and instantiate the required components.

### 1. Component Initialization

First, statically declare the components:

```cpp
#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esphome/components/micro_wake_word/micro_wake_word.h"

static esphome::i2s_audio::I2SAudioMicrophone microphone;
static esphome::i2s_audio::I2SAudioSpeaker speaker;
static esphome::micro_wake_word::MicroWakeWord wake_word;
```

### 2. Hardware Configuration

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

### 3. Wake Word Setup

Configure the wake word engine and link it to the microphone.

```cpp
wake_word.set_microphone(&microphone);
wake_word.add_wake_word_model(model_data, confidence, sensitivity, "Wake Word", arena_size);
wake_word.setup();
```

### 4. Error Checking

After calling `setup()` on each component, it is important to check for initialization errors.

```cpp
if (microphone.has_error() || speaker.has_error() || wake_word.has_error()) {
    ESP_LOGE(TAG, "Component setup failed! Error: %s", wake_word.get_error_message().c_str());
    // Handle error
}
```

### 5. Detection Callback

Register a callback function to be executed when the wake word is detected.

```cpp
wake_word.add_detection_callback([&](std::string detected_wake_word) {
    ESP_LOGI(TAG, "Wake word detected: %s", detected_wake_word.c_str());
    wake_word.pause();
    // Perform action (e.g., play a sound)
    wake_word.resume();
});
```

### 6. Start Detection

Finally, start the wake word detection engine.

```cpp
wake_word.start();
```

## Architecture

The library is designed around a few core principles to ensure stability and modularity in a real-time environment.

-   **Decoupled Components:** Each component (microphone, speaker, wake word) operates independently. Communication is handled through clean interfaces, such as the microphone's publisher/subscriber model.
-   **Non-Blocking Operations:** All I/O-intensive operations are managed in dedicated FreeRTOS tasks. This ensures that the main application logic is never blocked, allowing for responsive applications.
-   **Centralized State and Error Management:** All components inherit from a common base class, `I2SAudioComponent`, which manages a unified state machine (`RUNNING`, `STOPPED`, `ERROR`, etc.) and provides a centralized error reporting system for robust and consistent error handling.
