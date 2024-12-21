# Micro Wake Word Standalone Component for ESP-IDF

This project is a standalone version of the micro wake word feature found in the ESPHome project. It allows for the detection of specific wake words using the ESP32 platform without the need for the full ESPHome setup.

## Usage
To use this component in your project simply add it as a dependency like so:
```
dependencies:
  micro_wake_word:
    git: https://github.com/0xD34D/micro_wake_word_standalone.git
```

## Example

An example of how to use this component can be found in the `examples` folder. This example demonstrates how to initialize the micro wake word component, configure it, and handle wake word detection events.
