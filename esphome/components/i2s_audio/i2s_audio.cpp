#include "esphome/components/i2s_audio/i2s_audio.h"
#include "esp_log.h"
#include <cstdarg>
#include <cstdio>


#ifdef USE_ESP32

namespace esphome {
namespace i2s_audio {

static const char *const TAG = "i2s_audio";

const char* error_code_to_name(ErrorCode code) {
    switch (code) {
        case ErrorCode::NO_ERROR: return "NO_ERROR";
        case ErrorCode::ALLOC_FAILED: return "ALLOC_FAILED";
        case ErrorCode::SEMAPHORE_CREATE_FAILED: return "SEMAPHORE_CREATE_FAILED";
        case ErrorCode::MUTEX_CREATE_FAILED: return "MUTEX_CREATE_FAILED";
        case ErrorCode::EVENT_GROUP_CREATE_FAILED: return "EVENT_GROUP_CREATE_FAILED";
        case ErrorCode::TASK_CREATE_FAILED: return "TASK_CREATE_FAILED";
        case ErrorCode::I2S_CHANNEL_INIT_FAILED: return "I2S_CHANNEL_INIT_FAILED";
        case ErrorCode::I2S_CHANNEL_ENABLE_FAILED: return "I2S_CHANNEL_ENABLE_FAILED";
        case ErrorCode::I2S_CHANNEL_DISABLE_FAILED: return "I2S_CHANNEL_DISABLE_FAILED";
        case ErrorCode::I2S_CHANNEL_DELETE_FAILED: return "I2S_CHANNEL_DELETE_FAILED";
        case ErrorCode::I2S_READ_FAILED: return "I2S_READ_FAILED";
        case ErrorCode::I2S_WRITE_FAILED: return "I2S_WRITE_FAILED";
        case ErrorCode::I2S_PIN_CONFIG_INVALID: return "I2S_PIN_CONFIG_INVALID";
        case ErrorCode::MIC_SUBSCRIBE_FAILED: return "MIC_SUBSCRIBE_FAILED";
        case ErrorCode::MIC_PRIMITIVES_ALLOC_FAILED: return "MIC_PRIMITIVES_ALLOC_FAILED";
        case ErrorCode::MIC_ADC_CONFIG_INVALID: return "MIC_ADC_CONFIG_INVALID";
        case ErrorCode::MODEL_OPS_REGISTER_FAILED: return "MODEL_OPS_REGISTER_FAILED";
        case ErrorCode::MODEL_LOAD_FAILED: return "MODEL_LOAD_FAILED";
        case ErrorCode::WAKE_WORD_MODEL_INIT_FAILED: return "WAKE_WORD_MODEL_INIT_FAILED";
        case ErrorCode::VAD_MODEL_INIT_FAILED: return "VAD_MODEL_INIT_FAILED";
        case ErrorCode::FRONTEND_POPULATE_FAILED: return "FRONTEND_POPULATE_FAILED";
        case ErrorCode::FEATURES_GENERATE_FAILED: return "FEATURES_GENERATE_FAILED";
        case ErrorCode::PROBABILITY_UPDATE_FAILED: return "PROBABILITY_UPDATE_FAILED";
        case ErrorCode::BUFFER_MUTEX_ACQUIRE_FAILED: return "BUFFER_MUTEX_ACQUIRE_FAILED";
        case ErrorCode::RING_BUFFER_READ_FAILED: return "RING_BUFFER_READ_FAILED";
        case ErrorCode::RING_BUFFER_WRITE_FAILED: return "RING_BUFFER_WRITE_FAILED";
        case ErrorCode::TASK_SHUTDOWN_TIMEOUT: return "TASK_SHUTDOWN_TIMEOUT";
        case ErrorCode::MICROPHONE_HAS_ERROR: return "MICROPHONE_HAS_ERROR";
        case ErrorCode::GENERIC_ERROR: return "GENERIC_ERROR";
        default: return "UNKNOWN_ERROR";
    }
}

void I2SAudioComponent::report_error(ErrorCode code, const char *format, ...) {
    if (code == ErrorCode::NO_ERROR) return;

    this->last_error_ = code;
    this->state_ = ERROR;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    this->last_error_message_ = buffer;

    ESP_LOGE(TAG, "Error %s (%d): %s", error_code_to_name(code), static_cast<int>(code), buffer);
}

void I2SAudioComponent::setup() {
  static i2s_port_t next_port_num = I2S_NUM_0;

  if (next_port_num >= I2S_NUM_AUTO) {
    return;
  }

  this->port_ = next_port_num;
  next_port_num = (i2s_port_t) (next_port_num + 1);

  // ESP_LOGCONFIG(TAG, "Setting up I2S Audio...");
}

}  // namespace i2s_audio
}  // namespace esphome

#endif  // USE_ESP32
