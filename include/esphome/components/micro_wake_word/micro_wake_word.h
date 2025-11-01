#pragma once

#ifdef USE_ESP_IDF

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>

#include <frontend_util.h>
#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>

#include "esphome/components/i2s_audio/microphone/i2s_audio_microphone.h"
#include "esphome/core/ring_buffer.h"
#include "preprocessor_settings.h"
#include "streaming_model.h"

namespace esphome
{
  namespace micro_wake_word
  {

    // The number of audio slices to process before accepting a positive detection
    static const uint8_t MIN_SLICES_BEFORE_DETECTION = 74;

    enum state_t : uint8_t {
      STOPPED = 0,
      RUNNING,
      IDLE,  // Used for paused state
      ERROR,
    };

    inline bool is_running(state_t s) { return s == state_t::RUNNING; }
    inline bool has_failed(state_t s) { return s == state_t::ERROR; }

    class MicroWakeWord
    {
    public:
      void setup();
      void start();
      void stop();
      void reset();

      void pause();
      void resume();

      bool is_running() const { return micro_wake_word::is_running(this->state_); }
      bool has_error() const { return micro_wake_word::has_failed(this->state_); }

      void set_features_step_size(uint8_t step_size)
      {
        this->features_step_size_ = step_size;
      }

      void set_microphone(i2s_audio::I2SAudioMicrophone *microphone)
      {
        this->microphone_ = microphone;
      }

      void add_wake_word_model(const uint8_t *model_start, float probability_cutoff,
                               size_t sliding_window_average_size,
                               const std::string &wake_word,
                               size_t tensor_arena_size);

      void add_detection_callback(std::function<void(std::string)> &&detection_callback)
      {
        this->detection_callbacks_.add(std::move(detection_callback));
      }

#ifdef USE_MICRO_WAKE_WORD_VAD
      void add_vad_model(const uint8_t *model_start, float probability_cutoff,
                         size_t sliding_window_size, size_t tensor_arena_size);
#endif

    protected:
      i2s_audio::I2SAudioMicrophone *microphone_{nullptr};
      state_t state_{state_t::RUNNING};
      
      TaskHandle_t processing_task_handle_{nullptr};
      EventGroupHandle_t task_events_{nullptr};
      static void processing_task_wrapper(void *param);
      void processing_task();

      std::unique_ptr<RingBuffer> ring_buffer_;

      std::vector<WakeWordModel> wake_word_models_;

#ifdef USE_MICRO_WAKE_WORD_VAD
      std::unique_ptr<VADModel> vad_model_;
#endif

      tflite::MicroMutableOpResolver<20> streaming_op_resolver_;

      struct FrontendConfig frontend_config_;
      struct FrontendState frontend_state_;

      int16_t ignore_windows_{-MIN_SLICES_BEFORE_DETECTION};
      uint8_t features_step_size_;
      int16_t *preprocessor_audio_buffer_{nullptr};

      bool detected_{false};
      std::string detected_wake_word_{""};
      CallbackManager<void(std::string)> detection_callbacks_{};

      void set_state_(state_t state);

      bool has_enough_samples_();
      bool allocate_buffers_();
      void deallocate_buffers_();
      bool load_models_();
      void unload_models_();
      void update_model_probabilities_();
      bool detect_wake_words_();
      bool generate_features_for_window_(int8_t features[PREPROCESSOR_FEATURE_SIZE]);
      void reset_states_();
      bool register_streaming_ops_(tflite::MicroMutableOpResolver<20> &op_resolver);

      inline uint16_t new_samples_to_get_()
      {
        return (this->features_step_size_ * (AUDIO_SAMPLE_FREQUENCY / 1000));
      }

      size_t mic_subscription_id_{0};
      void on_audio_received(const int16_t *data, size_t num_samples);
      SemaphoreHandle_t data_sem_{nullptr};
    };

  } // namespace micro_wake_word
} // namespace esphome

#endif // USE_ESP_IDF