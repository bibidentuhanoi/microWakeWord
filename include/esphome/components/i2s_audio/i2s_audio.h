#pragma once

#ifdef USE_ESP32

#include <driver/i2s_std.h>
#include "esphome/core/helpers.h"
#include <string>

namespace esphome
{
  namespace i2s_audio
  {
    enum State {
      STOPPED = 0,
      STARTING,
      RUNNING,
      STOPPING,
      IDLE,
      ERROR,
    };

    enum class ErrorCode : int {
        NO_ERROR = 0,
        // General/Resource Errors
        ALLOC_FAILED = -1,
        SEMAPHORE_CREATE_FAILED = -2,
        MUTEX_CREATE_FAILED = -3,
        EVENT_GROUP_CREATE_FAILED = -4,
        TASK_CREATE_FAILED = -5,
        // I2S-Specific Errors
        I2S_CHANNEL_INIT_FAILED = -10,
        I2S_CHANNEL_ENABLE_FAILED = -11,
        I2S_CHANNEL_DISABLE_FAILED = -12,
        I2S_CHANNEL_DELETE_FAILED = -13,
        I2S_READ_FAILED = -14,
        I2S_WRITE_FAILED = -15,
        I2S_PIN_CONFIG_INVALID = -16,
        // Microphone-Specific Errors
        MIC_SUBSCRIBE_FAILED = -20,
        MIC_PRIMITIVES_ALLOC_FAILED = -21,
        MIC_ADC_CONFIG_INVALID = -22,
        // Wake Word/Model Errors
        MODEL_OPS_REGISTER_FAILED = -30,
        MODEL_LOAD_FAILED = -31,
        WAKE_WORD_MODEL_INIT_FAILED = -32,
        VAD_MODEL_INIT_FAILED = -33,
        FRONTEND_POPULATE_FAILED = -34,
        FEATURES_GENERATE_FAILED = -35,
        PROBABILITY_UPDATE_FAILED = -36,
        // Runtime/Operational Errors
        BUFFER_MUTEX_ACQUIRE_FAILED = -40,
        RING_BUFFER_READ_FAILED = -41,
        RING_BUFFER_WRITE_FAILED = -42,
        TASK_SHUTDOWN_TIMEOUT = -43,
        MICROPHONE_HAS_ERROR = -44,
        // Fallback
        GENERIC_ERROR = -999
    };

    const char* error_code_to_name(ErrorCode code);

    class I2SAudioComponent
    {
    public:
      virtual ~I2SAudioComponent() = default;
      void setup();
      virtual void start() = 0;
      virtual void stop() = 0;
      
      State get_state() const { return this->state_; }
      bool is_running() const { return this->state_ == RUNNING; }
      bool is_stopped() const { return this->state_ == STOPPED; }
      bool has_error() const { return this->state_ == ERROR || this->last_error_ != ErrorCode::NO_ERROR; }

      ErrorCode get_last_error() const { return this->last_error_; }
      std::string get_error_message() const { return this->last_error_message_; }

      i2s_std_gpio_config_t get_gpio_config() const
      {
        return {
            .mclk = this->mclk_pin_, // some codecs may require mclk signal, this example doesn't need it
            .bclk = this->bclk_pin_,
            .ws = this->lrclk_pin_,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            }};
      }

      void set_mclk_pin(gpio_num_t pin) { this->mclk_pin_ = pin; }
      void set_bclk_pin(gpio_num_t pin) { this->bclk_pin_ = pin; }
      void set_lrclk_pin(gpio_num_t pin) { this->lrclk_pin_ = pin; }

      void lock() { this->lock_.lock(); }
      bool try_lock() { return this->lock_.try_lock(); }
      void unlock() { this->lock_.unlock(); }

      i2s_port_t get_port() const { return this->port_; }

    protected:
      void report_error(ErrorCode code, const char* format, ...) __attribute__((format(printf, 3, 4)));
      Mutex lock_;
      State state_{STOPPED};

      ErrorCode last_error_{ErrorCode::NO_ERROR};
      std::string last_error_message_;

      gpio_num_t mclk_pin_{I2S_GPIO_UNUSED};
      gpio_num_t bclk_pin_{I2S_GPIO_UNUSED};
      gpio_num_t lrclk_pin_;
      i2s_port_t port_{};
    };

  } // namespace i2s_audio
} // namespace esphome

#include "speaker/i2s_audio_speaker.h"
#include "microphone/i2s_audio_microphone.h"

#endif // USE_ESP32
