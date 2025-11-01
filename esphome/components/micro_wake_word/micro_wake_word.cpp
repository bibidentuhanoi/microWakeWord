#include "esphome/components/micro_wake_word/micro_wake_word.h"
#include "esphome/components/micro_wake_word/streaming_model.h"

#ifdef USE_ESP_IDF

#include "esp_log.h"
#include "esphome/core/helpers.h"
#include <frontend.h>
#include <frontend_util.h>
#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <cmath>
#include "esphome/components/i2s_audio/microphone/i2s_audio_microphone.h"

namespace esphome
{
  namespace micro_wake_word
  {

    static const char *const TAG = "micro_wake_word";

    static const size_t SAMPLE_RATE_HZ = 16000;
    static const size_t BUFFER_LENGTH = 64;
    static const size_t BUFFER_SIZE = SAMPLE_RATE_HZ / 1000 * BUFFER_LENGTH;

#define TASK_DONE_BIT BIT0

    void MicroWakeWord::setup()
    {
      ESP_LOGI(TAG, "Setting up microWakeWord...");

      if (!this->register_streaming_ops_(this->streaming_op_resolver_))
      {
        this->state_ = state_t::ERROR;
        return;
      }

      ESP_LOGI(TAG, "Micro Wake Word initialized");

      this->frontend_config_.window.size_ms = FEATURE_DURATION_MS;
      this->frontend_config_.window.step_size_ms = this->features_step_size_;
      this->frontend_config_.filterbank.num_channels = PREPROCESSOR_FEATURE_SIZE;
      this->frontend_config_.filterbank.lower_band_limit = 125.0;
      this->frontend_config_.filterbank.upper_band_limit = 7500.0;
      this->frontend_config_.noise_reduction.smoothing_bits = 10;
      this->frontend_config_.noise_reduction.even_smoothing = 0.025;
      this->frontend_config_.noise_reduction.odd_smoothing = 0.06;
      this->frontend_config_.noise_reduction.min_signal_remaining = 0.05;
      this->frontend_config_.pcan_gain_control.enable_pcan = 1;
      this->frontend_config_.pcan_gain_control.strength = 0.95;
      this->frontend_config_.pcan_gain_control.offset = 80.0;
      this->frontend_config_.pcan_gain_control.gain_bits = 21;
      this->frontend_config_.log_scale.enable_log = 1;
      this->frontend_config_.log_scale.scale_shift = 6;

      // Start detecting immediately
      if (!this->load_models_() || !this->allocate_buffers_()) {
        this->state_ = state_t::ERROR;
        return;
      }

      this->mic_subscription_id_ = this->microphone_->subscribe(
          [this](const int16_t *data, size_t num_samples) { this->on_audio_received(data, num_samples); });

      this->data_sem_ = xSemaphoreCreateBinary();

      this->task_events_ = xEventGroupCreate();

      xTaskCreate(this->processing_task_wrapper, "ww_process", 4096, this, 5, &this->processing_task_handle_);

      this->set_state_(state_t::RUNNING);
    }

    void MicroWakeWord::start() {
      // Since it starts automatically, start() can be a no-op or resume if paused/stopped
      if (this->state_ == state_t::STOPPED || this->state_ == state_t::IDLE) {
        this->resume();
      }
    }

    void MicroWakeWord::stop() {
      this->set_state_(state_t::STOPPED);

      if (this->processing_task_handle_) {
        xEventGroupWaitBits(this->task_events_, TASK_DONE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(500));
        vEventGroupDelete(this->task_events_);
        this->task_events_ = nullptr;
        this->processing_task_handle_ = nullptr;
      }

      this->microphone_->unsubscribe(this->mic_subscription_id_);

      this->unload_models_();
      this->deallocate_buffers_();

      if (this->data_sem_) {
        vSemaphoreDelete(this->data_sem_);
        this->data_sem_ = nullptr;
      }
    }

    void MicroWakeWord::pause() {
      this->set_state_(state_t::IDLE);
    }

    void MicroWakeWord::resume() {
      this->set_state_(state_t::RUNNING);
    }

    void MicroWakeWord::add_wake_word_model(const uint8_t *model_start,
                                            float probability_cutoff,
                                            size_t sliding_window_average_size,
                                            const std::string &wake_word,
                                            size_t tensor_arena_size)
    {
      this->wake_word_models_.emplace_back(model_start, probability_cutoff,
                                           sliding_window_average_size, wake_word,
                                           tensor_arena_size);
    }

#ifdef USE_MICRO_WAKE_WORD_VAD
    void MicroWakeWord::add_vad_model(const uint8_t *model_start,
                                      float probability_cutoff,
                                      size_t sliding_window_size,
                                      size_t tensor_arena_size)
    {
      this->vad_model_ = make_unique<VADModel>(
          model_start, probability_cutoff, sliding_window_size, tensor_arena_size);
    }
#endif
    
    void MicroWakeWord::processing_task_wrapper(void *param) {
      static_cast<MicroWakeWord *>(param)->processing_task();
    }

    void MicroWakeWord::processing_task() {
      while (this->state_ != state_t::STOPPED) {
        
        ESP_LOGD(TAG, "Processing task: beginning detection loop.");

        while (micro_wake_word::is_running(this->state_)) {
          
          while (!this->has_enough_samples_()) {
            if (xSemaphoreTake(this->data_sem_, pdMS_TO_TICKS(1000)) != pdTRUE) {
              if (this->microphone_->has_error()) {
                ESP_LOGE(TAG, "Microphone error detected; setting state to ERROR");
                this->set_state_(state_t::ERROR);
                break;
              }
              continue;
            }
          }
          if (this->state_ == state_t::ERROR) break;
          
          this->update_model_probabilities_();
          if (this->state_ == state_t::ERROR) {
            ESP_LOGE(TAG, "Error during model probability update.");
            break; 
          }

          if (this->detect_wake_words_()) {
            ESP_LOGD(TAG, "Wake Word '%s' Detected", (this->detected_wake_word_).c_str());
            this->detected_ = true;
            this->detection_callbacks_.call(this->detected_wake_word_);
            this->detected_wake_word_ = "";
            this->reset_states_();
            ESP_LOGD(TAG, "Wake word component reset and ready for next detection.");
          }
        }
        
        if (this->state_ == state_t::ERROR) {
          ESP_LOGE(TAG, "Error detected in processing task. Initiating recovery...");
          this->unload_models_();
          this->deallocate_buffers_();
          ESP_LOGI(TAG, "Waiting 1 second before recovery attempt...");
          vTaskDelay(pdMS_TO_TICKS(1000));
          
          if (this->load_models_() && this->allocate_buffers_()) {
            this->reset_states_();
            this->set_state_(state_t::RUNNING);
            ESP_LOGI(TAG, "Recovery successful, resuming wake word detection.");
          } else {
            ESP_LOGE(TAG, "Recovery failed. Retrying in 2 seconds...");
            vTaskDelay(pdMS_TO_TICKS(2000));
          }
        } else if (this->state_ == state_t::IDLE) {
          vTaskDelay(pdMS_TO_TICKS(100));  // Prevent busy loop during pause
        }
      }
      
      ESP_LOGI(TAG, "Processing task exiting (state changed to STOPPED).");
      xEventGroupSetBits(this->task_events_, TASK_DONE_BIT);
      this->processing_task_handle_ = nullptr;
      vTaskDelete(NULL);
    }

    void MicroWakeWord::set_state_(state_t state) {
      this->state_ = state;
    }

    void MicroWakeWord::on_audio_received(const int16_t *data, size_t num_samples) {
      // Assuming ring_buffer_ is allocated in allocate_buffers_
      size_t bytes_written = this->ring_buffer_->write((void *)data, num_samples * sizeof(int16_t));
      if (bytes_written < num_samples * sizeof(int16_t)) {
        ESP_LOGW(TAG, "Ring buffer overflow");
      }
      xSemaphoreGive(this->data_sem_);
    }

    bool MicroWakeWord::allocate_buffers_()
    {
      this->ring_buffer_ = RingBuffer::create(BUFFER_SIZE * sizeof(int16_t));
      if (!this->ring_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate ring buffer");
        return false;
      }

      ExternalRAMAllocator<int16_t> audio_samples_allocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
      this->preprocessor_audio_buffer_ = audio_samples_allocator.allocate(this->new_samples_to_get_());
      if (!this->preprocessor_audio_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate preprocessor audio buffer");
        return false;
      }

      return true;
    }

    void MicroWakeWord::deallocate_buffers_()
    {
      this->ring_buffer_.reset();

      ExternalRAMAllocator<int16_t> audio_samples_allocator(ExternalRAMAllocator<int16_t>::ALLOW_FAILURE);
      audio_samples_allocator.deallocate(this->preprocessor_audio_buffer_,
                                         this->new_samples_to_get_());
      this->preprocessor_audio_buffer_ = nullptr;
    }

    bool MicroWakeWord::load_models_()
    {
      if (!FrontendPopulateState(&this->frontend_config_, &this->frontend_state_,
                                 AUDIO_SAMPLE_FREQUENCY))
      {
        ESP_LOGD(TAG, "Failed to populate frontend state");
        FrontendFreeStateContents(&this->frontend_state_);
        return false;
      }

      for (auto &model : this->wake_word_models_)
      {
        if (!model.load_model(this->streaming_op_resolver_))
        {
          ESP_LOGE(TAG, "Failed to initialize a wake word model %s.", model.get_wake_word().c_str());
          this->set_state_(state_t::ERROR);
          return false;
        }
      }
#ifdef USE_MICRO_WAKE_WORD_VAD
      if (!this->vad_model_->load_model(this->streaming_op_resolver_))
      {
        ESP_LOGE(TAG, "Failed to initialize VAD model.");
        this->set_state_(state_t::ERROR);
        return false;
      }
#endif

      return true;
    }

    void MicroWakeWord::unload_models_()
    {
      FrontendFreeStateContents(&this->frontend_state_);

      for (auto &model : this->wake_word_models_)
      {
        model.unload_model();
      }
#ifdef USE_MICRO_WAKE_WORD_VAD
      this->vad_model_->unload_model();
#endif
    }

    void MicroWakeWord::update_model_probabilities_()
    {
      int8_t audio_features[PREPROCESSOR_FEATURE_SIZE];

      if (!this->generate_features_for_window_(audio_features))
      {
        return;
      }

      this->ignore_windows_ = std::min(this->ignore_windows_ + 1, 0);

      for (auto &model : this->wake_word_models_)
      {
        model.perform_streaming_inference(audio_features);
      }
#ifdef USE_MICRO_WAKE_WORD_VAD
      this->vad_model_->perform_streaming_inference(audio_features);
#endif
    }

    bool MicroWakeWord::detect_wake_words_()
    {
      if (this->ignore_windows_ < 0)
      {
        return false;
      }

#ifdef USE_MICRO_WAKE_WORD_VAD
      bool vad_state = this->vad_model_->determine_detected();
#endif

      for (auto &model : this->wake_word_models_)
      {
        if (model.determine_detected())
        {
#ifdef USE_MICRO_WAKE_WORD_VAD
          if (vad_state)
          {
#endif
            this->detected_wake_word_ = model.get_wake_word();
            return true;
#ifdef USE_MICRO_WAKE_WORD_VAD
          }
          else
          {
            ESP_LOGD(TAG, "Wake word model predicts %s, but VAD model doesn't.",
                     model.get_wake_word().c_str());
          }
#endif
        }
      }

      return false;
    }

    bool MicroWakeWord::has_enough_samples_()
    {
      return this->ring_buffer_->available() >=
             (this->features_step_size_ * (AUDIO_SAMPLE_FREQUENCY / 1000)) *
                 sizeof(int16_t);
    }

    bool MicroWakeWord::generate_features_for_window_(
        int8_t features[PREPROCESSOR_FEATURE_SIZE])
    {
      if (!this->has_enough_samples_())
      {
        return false;
      }

      size_t bytes_read = this->ring_buffer_->read(
          (void *)(this->preprocessor_audio_buffer_),
          this->new_samples_to_get_() * sizeof(int16_t), pdMS_TO_TICKS(200));

      if (bytes_read == 0)
      {
        ESP_LOGE(TAG, "Could not read data from Ring Buffer");
        return false;
      }
      else if (bytes_read < this->new_samples_to_get_() * sizeof(int16_t))
      {
        return false;
      }

      size_t num_samples_read;
      struct FrontendOutput frontend_output = FrontendProcessSamples(
          &this->frontend_state_, this->preprocessor_audio_buffer_,
          this->new_samples_to_get_(), &num_samples_read);

      for (size_t i = 0; i < frontend_output.size; ++i)
      {
        constexpr int32_t value_scale = 256;
        constexpr int32_t value_div = 666; 
        int32_t value =
            ((frontend_output.values[i] * value_scale) + (value_div / 2)) /
            value_div;
        value -= 128;
        if (value < -128)
        {
          value = -128;
        }
        if (value > 127)
        {
          value = 127;
        }
        features[i] = value;
      }

      return true;
    }

    void MicroWakeWord::reset_states_()
    {
      ESP_LOGD(TAG, "Resetting buffers and probabilities");
      if (this->ring_buffer_)
        this->ring_buffer_->reset();
      this->ignore_windows_ = -MIN_SLICES_BEFORE_DETECTION;
      for (auto &model : this->wake_word_models_)
      {
        model.reset_probabilities();
      }
#ifdef USE_MICRO_WAKE_WORD_VAD
      this->vad_model_->reset_probabilities();
#endif
    }

    void MicroWakeWord::reset() {
      this->reset_states_();
    }

    bool MicroWakeWord::register_streaming_ops_(
        tflite::MicroMutableOpResolver<20> &op_resolver)
    {
      if (op_resolver.AddCallOnce() != kTfLiteOk) return false;
      if (op_resolver.AddVarHandle() != kTfLiteOk) return false;
      if (op_resolver.AddReshape() != kTfLiteOk) return false;
      if (op_resolver.AddReadVariable() != kTfLiteOk) return false;
      if (op_resolver.AddStridedSlice() != kTfLiteOk) return false;
      if (op_resolver.AddConcatenation() != kTfLiteOk) return false;
      if (op_resolver.AddAssignVariable() != kTfLiteOk) return false;
      if (op_resolver.AddConv2D() != kTfLiteOk) return false;
      if (op_resolver.AddMul() != kTfLiteOk) return false;
      if (op_resolver.AddAdd() != kTfLiteOk) return false;
      if (op_resolver.AddMean() != kTfLiteOk) return false;
      if (op_resolver.AddFullyConnected() != kTfLiteOk) return false;
      if (op_resolver.AddLogistic() != kTfLiteOk) return false;
      if (op_resolver.AddQuantize() != kTfLiteOk) return false;
      if (op_resolver.AddDepthwiseConv2D() != kTfLiteOk) return false;
      if (op_resolver.AddAveragePool2D() != kTfLiteOk) return false;
      if (op_resolver.AddMaxPool2D() != kTfLiteOk) return false;
      if (op_resolver.AddPad() != kTfLiteOk) return false;
      if (op_resolver.AddPack() != kTfLiteOk) return false;
      if (op_resolver.AddSplitV() != kTfLiteOk) return false;

      return true;
    }

  } 
} 

#endif