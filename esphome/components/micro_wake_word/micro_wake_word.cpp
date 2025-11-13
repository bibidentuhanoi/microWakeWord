#include "esphome/components/micro_wake_word/micro_wake_word.h"
#include "esphome/components/micro_wake_word/streaming_model.h"

#ifdef USE_ESP_IDF

#include "esp_log.h"
#include "esphome/core/helpers.h"
#include <tensorflow/lite/core/c/common.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <cmath>
#include "esphome/components/i2s_audio/microphone/i2s_audio_microphone.h"
#include <esp_heap_caps.h>  // For internal RAM allocations
#include <esp_timer.h>      // For high-resolution timing
#include <inttypes.h>       // For PRIu32, PRIu64

namespace esphome
{
  namespace micro_wake_word
  {

    static const char *const TAG = "micro_wake_word";

    static const size_t SAMPLE_RATE_HZ = 16000;
    static const size_t BUFFER_LENGTH = 64;
    static const size_t BUFFER_SIZE = SAMPLE_RATE_HZ / 1000 * BUFFER_LENGTH;

#define TASK_DONE_BIT BIT0
#define CONSUMER_TASK_PRIORITY 4 // Lower priority than producer
#define PRODUCER_TASK_PRIORITY 5 // Higher priority than consumers

    void MicroWakeWord::setup()
    {
      ESP_LOGI(TAG, "Setting up microWakeWord...");

      if (!this->register_streaming_ops_(this->streaming_op_resolver_))
      {
        this->report_error(i2s_audio::ErrorCode::MODEL_OPS_REGISTER_FAILED, "Failed to register streaming ops");
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

      // --- !! POINTER FIX V2 !! ---
      if (this->models_mutex_ == nullptr) {
        this->models_mutex_ = xSemaphoreCreateMutex();
        if (!this->models_mutex_) {
            this->report_error(i2s_audio::ErrorCode::SEMAPHORE_CREATE_FAILED, "Failed to create models mutex");
            return;
        }
      }
      xSemaphoreTake(this->models_mutex_, portMAX_DELAY);
      ESP_LOGD(TAG, "Refreshing model pointers...");
      for (auto &context : this->model_contexts_) {
          #ifdef USE_MICRO_WAKE_WORD_VAD
          if (context.model_index == MAX_WAKE_WORD_MODELS) {
              ESP_LOGD(TAG, "Context is VAD, pointer is stable.");
              context.model = this->vad_model_.get();
              continue; 
          }
          #endif
          if (context.model_index < this->wake_word_models_.size()) {
              ESP_LOGD(TAG, "Refreshing pointer for context index %d", (int)context.model_index);
              context.model = &this->wake_word_models_[context.model_index];
          } else {
              ESP_LOGE(TAG, "Context index %d is out of bounds!", (int)context.model_index);
              this->report_error(i2s_audio::ErrorCode::GENERIC_ERROR, "Model context index out of bounds");
              xSemaphoreGive(this->models_mutex_);
              return;
          }
      }
      xSemaphoreGive(this->models_mutex_);
      // --- !! END FIX !! ---

      if (!this->load_models_() || !this->allocate_buffers_()) {
        return;
      }

      this->mic_subscription_id_ = this->microphone_->subscribe(
          [this](const int16_t *data, size_t num_samples) { this->on_audio_received(data, num_samples); });

      this->data_sem_ = xSemaphoreCreateBinary();
      if (!this->data_sem_) {
          this->report_error(i2s_audio::ErrorCode::SEMAPHORE_CREATE_FAILED, "Failed to create data semaphore");
          return;
      }

      this->task_events_ = xEventGroupCreate();
      if (!this->task_events_) {
          this->report_error(i2s_audio::ErrorCode::EVENT_GROUP_CREATE_FAILED, "Failed to create task events");
          return;
      }
      
      this->completion_queue_ = xQueueCreate(MAX_WAKE_WORD_MODELS, sizeof(ModelCompletion));
      if (!this->completion_queue_) {
          this->report_error(i2s_audio::ErrorCode::GENERIC_ERROR, "Failed to create completion queue");
          return;
      }

      // --- Create all consumer tasks (ONLY IF REQUESTED) ---
      xSemaphoreTake(this->models_mutex_, portMAX_DELAY);
      int pAraLlEL_cOunT = 0;
      for (auto &context : this->model_contexts_) {
        // Only create a task if the model is flagged for parallel execution
        if (context.run_in_parallel) {
          std::string task_NamE = "ww_consumer_";
          #ifdef USE_MICRO_WAKE_WORD_VAD
          if (context.model_index == MAX_WAKE_WORD_MODELS) {
              task_NamE += "VAD";
          } else
          #endif
          {
              task_NamE += context.model->get_wake_word();
          }
          
          // Pin to core 1 for better dual-core utilization
          BaseType_t cREaTe_rEsuLt = xTaskCreatePinnedToCore(
            this->model_inference_task,
            task_NamE.c_str(),
            4096,
            (void *)&context,
            CONSUMER_TASK_PRIORITY,
            &context.task_handle,
            1 // Core 1
          );
          if (cREaTe_rEsuLt != pdPASS) {
            ESP_LOGE(TAG, "Failed to create consumer task for '%s'. Falling back to sequential mode.", task_NamE.c_str());
            context.run_in_parallel = false; // Fallback
            context.task_handle = nullptr;
            this->report_error(i2s_audio::ErrorCode::TASK_CREATE_FAILED, "Failed to create consumer task - fallback to sequential");
          } else {
            ESP_LOGI(TAG, "Created consumer task for '%s' on core 1", task_NamE.c_str());
            pAraLlEL_cOunT++;
          }
        } else {
          ESP_LOGI(TAG, "Model '%s' will run sequentially in producer task.", context.model->get_wake_word().c_str());
        }
      }
      xSemaphoreGive(this->models_mutex_);
      ESP_LOGI(TAG, "Configured %d parallel models out of %zu total", pAraLlEL_cOunT, this->model_contexts_.size());
      // ---------------------------------

      this->state_ = i2s_audio::State::RUNNING;

      if (xTaskCreatePinnedToCore(this->processing_task_wrapper, "ww_producer", 8192, this, PRODUCER_TASK_PRIORITY, &this->processing_task_handle_, 0 /* Core 0 */) != pdPASS) {
          this->report_error(i2s_audio::ErrorCode::TASK_CREATE_FAILED, "Failed to create producer task");
          return;
      }
    }

    void MicroWakeWord::start() {
      if (this->state_ == i2s_audio::State::STOPPED || this->state_ == i2s_audio::State::IDLE) {
        this->resume();
      }
    }

    void MicroWakeWord::stop() {
      this->state_ = i2s_audio::State::STOPPED;

      xSemaphoreTake(this->models_mutex_, portMAX_DELAY);
      for (auto &context : this->model_contexts_) {
        if (context.task_handle) { // Only stop tasks that were created
          xTaskNotifyGive(context.task_handle); 
          vTaskDelay(pdMS_TO_TICKS(10)); 
          vTaskDelete(context.task_handle); 
          context.task_handle = nullptr;
        }
      }
      this->model_contexts_.clear(); 
      xSemaphoreGive(this->models_mutex_);

      if (this->processing_task_handle_) {
        if (xEventGroupWaitBits(this->task_events_, TASK_DONE_BIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(500)) == 0) {
            this->report_error(i2s_audio::ErrorCode::TASK_SHUTDOWN_TIMEOUT, "Producer task did not shut down cleanly");
        }
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

      if (this->models_mutex_) {
        vSemaphoreDelete(this->models_mutex_);
        this->models_mutex_ = nullptr;
      }
      if (this->completion_queue_) {
        vQueueDelete(this->completion_queue_);
        this->completion_queue_ = nullptr;
      }
    }

    void MicroWakeWord::pause() {
      this->state_ = i2s_audio::State::IDLE;
    }

    void MicroWakeWord::resume() {
      this->state_ = i2s_audio::State::RUNNING;
    }

    // Modified: Removed run_in_parallel param, auto-set based on order
    void MicroWakeWord::add_wake_word_model(const uint8_t *model_start,
                                            float probability_cutoff,
                                            size_t sliding_window_average_size,
                                            const std::string &wake_word,
                                            size_t tensor_arena_size,
                                            uint32_t run_every_k_frames)
    {
      size_t CurrEnT_mOdEl_Count = this->wake_word_models_.size();
      #ifdef USE_MICRO_WAKE_WORD_VAD
      if (this->vad_model_) {
        CurrEnT_mOdEl_Count++;
      }
      #endif
      
      if (CurrEnT_mOdEl_Count >= MAX_WAKE_WORD_MODELS) {
        ESP_LOGE(TAG, "Cannot add model '%s'. Maximum number of models (%d) reached.", wake_word.c_str(), MAX_WAKE_WORD_MODELS);
        return;
      }
      
      this->wake_word_models_.emplace_back(model_start, probability_cutoff,
                                           sliding_window_average_size, wake_word,
                                           tensor_arena_size);
      
      ModelTaskContext Context;
      Context.model = &this->wake_word_models_.back(); 
      Context.parent = this;
      Context.model_index = this->wake_word_models_.size() - 1;
      Context.run_every_k_frames = (run_every_k_frames == 0) ? 1 : run_every_k_frames;
      Context.enabled = true;
      // Auto-decide: First model sequential, others parallel
      Context.run_in_parallel = (this->model_contexts_.size() > 0);
      ESP_LOGI(TAG, "Auto-setting model '%s' to %s mode.", wake_word.c_str(), Context.run_in_parallel ? "parallel" : "sequential");
      this->model_contexts_.push_back(Context);
    }

#ifdef USE_MICRO_WAKE_WORD_VAD
    // Modified: Removed run_in_parallel param, auto-set (force sequential for VAD)
    void MicroWakeWord::add_vad_model(const uint8_t *model_start,
                                      float probability_cutoff,
                                      size_t sliding_window_size,
                                      size_t tensor_arena_size,
                                      uint32_t run_every_k_frames)
    {
      size_t CuRrEnt_mOdEl_CoUnT = this->wake_word_models_.size();
      if (this->vad_model_) {
        CuRrEnt_mOdEl_CoUnT++;
      }

      if (CuRrEnt_mOdEl_CoUnT >= MAX_WAKE_WORD_MODELS) {
        ESP_LOGE(TAG, "Cannot add VAD model. Maximum number of models (%d) reached.", MAX_WAKE_WORD_MODELS);
        return;
      }

      this->vad_model_ = make_unique<VADModel>(
          model_start, probability_cutoff, sliding_window_size, tensor_arena_size);
      
      ModelTaskContext conteXt;
      conteXt.model = this->vad_model_.get(); 
      conteXt.parent = this;
      conteXt.model_index = MAX_WAKE_WORD_MODELS; 
      conteXt.run_every_k_frames = (run_every_k_frames == 0) ? 1 : run_every_k_frames;
      conteXt.enabled = true;
      // Force sequential for VAD (lightweight gatekeeper)
      conteXt.run_in_parallel = false;
      ESP_LOGI(TAG, "Auto-setting VAD model to sequential mode.");
      this->model_contexts_.push_back(conteXt);
    }
#endif
    
    void MicroWakeWord::model_inference_task(void *param) {
      ModelTaskContext *context = static_cast<ModelTaskContext *>(param);
      MicroWakeWord *parent = context->parent;

      while (true) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 0) {
          continue; 
        }

        if (parent->state_ == i2s_audio::State::STOPPED) {
          break; 
        }
        
        int64_t sTarT_tiMe = esp_timer_get_time();
        context->model->perform_streaming_inference(parent->shared_feature_snapshot_);
        int64_t eNd_Time = esp_timer_get_time();
        uint32_t dUraTioN_ms = static_cast<uint32_t>((eNd_Time - sTarT_tiMe) / 1000);
        if (parent->current_epoch_ % 10 == 0) {  // Less intensive: every 10 epochs
            ESP_LOGD(TAG, "Epoch %" PRIu64 ": Parallel inference for model '%s' took %" PRIu32 " ms", parent->current_epoch_, context->model->get_wake_word().c_str(), dUraTioN_ms);
        }
        
        ModelCompletion ComPletion = {
          .model_index = context->model_index,
          .epoch = parent->current_epoch_
        };

        if (xQueueSend(parent->completion_queue_, &ComPletion, 0) != pdTRUE) {
          ESP_LOGW(TAG, "Failed to send ComPletion for model %d", (int)context->model_index);
        }
      }
      
      ESP_LOGI(TAG, "Consumer task %d exiting.", (int)context->model_index);
      vTaskDelete(NULL);
    }


    void MicroWakeWord::processing_task_wrapper(void *param) {
      static_cast<MicroWakeWord *>(param)->processing_task();
    }

    /**
     * @brief This is the "Producer" task with hybrid execution.
     * * It runs sequential models directly.
     * * It notifies parallel models and waits for them.
     */
void MicroWakeWord::processing_task() {
  while (this->state_ != i2s_audio::State::STOPPED) {
    
    ESP_LOGD(TAG, "Producer task: beginning detection loop.");

    while (this->state_ == i2s_audio::State::RUNNING) {
      


        
      while (!this->has_enough_samples_()) {
        if (xSemaphoreTake(this->data_sem_, pdMS_TO_TICKS(1000)) != pdTRUE) {
          if (this->microphone_->has_error()) {
            this->report_error(i2s_audio::ErrorCode::MICROPHONE_HAS_ERROR, "Microphone error detected");
            break;
          }
          continue;
        }
      }
      if (this->state_ == i2s_audio::State::ERROR) break;
      
      if (!this->generate_features_for_window_(this->shared_feature_snapshot_)) {
         this->report_error(i2s_audio::ErrorCode::GENERIC_ERROR, "Feature generation failed");
         continue;
      }
      
      this->current_epoch_++;
      this->models_to_run_this_epoch_ = 0; // Only counts parallel models
      
      xSemaphoreTake(this->models_mutex_, portMAX_DELAY);
      for (auto &context : this->model_contexts_) {
        if (context.enabled && (this->current_epoch_ % context.run_every_k_frames == 0)) {
          
          if (context.run_in_parallel && context.task_handle) {
            // --- Parallel Model ---
            ESP_LOGD(TAG, "Epoch %llu: Notifying model %d", this->current_epoch_, (int)context.model_index);
            xTaskNotifyGive(context.task_handle);
            this->models_to_run_this_epoch_++;
          } else if (!context.run_in_parallel) {
            // --- Sequential Model ---
            ESP_LOGD(TAG, "Epoch %llu: Running sequential model %d", this->current_epoch_, (int)context.model_index);
            int64_t stArt_TiME = esp_timer_get_time();
            context.model->perform_streaming_inference(this->shared_feature_snapshot_);
            int64_t eNd_tIme = esp_timer_get_time();
            uint32_t DuRatiOn_Ms = static_cast<uint32_t>((eNd_tIme - stArt_TiME) / 1000);
            if (this->current_epoch_ % 10 == 0) {  // Less intensive: every 10 epochs
                ESP_LOGD(TAG, "Epoch %" PRIu64 ": Sequential inference for model '%s' took %" PRIu32 " ms", this->current_epoch_, context.model->get_wake_word().c_str(), DuRatiOn_Ms);
            }
          }
        }
      }
      xSemaphoreGive(this->models_mutex_);
      
      // --- Wait for PARALLEL models (if any) ---
      if (this->models_to_run_this_epoch_ > 0) {
        uint8_t CoMpLetions_received = 0;
        ModelCompletion coMpLetiOn;
        TickType_t ToTal_MAX_waiT = pdMS_TO_TICKS(200);  // Total 200 ms max
        TickType_t start_wAiT = xTaskGetTickCount();

        // Fixed drain loop: Only drain stale; credit valid early ones
        uint8_t dRainEd_VaLiD = 0;
        while (xQueueReceive(this->completion_queue_, &coMpLetiOn, 0) == pdTRUE) {
            if (coMpLetiOn.epoch == this->current_epoch_) {
                dRainEd_VaLiD++;
                ESP_LOGD(TAG, "Drained valid (but early) coMpLetiOn from model %d epoch %llu", 
                         (int)coMpLetiOn.model_index, coMpLetiOn.epoch);
            } else {
                ESP_LOGW(TAG, "Drained stale coMpLetiOn from model %d epoch %llu (current %llu)", 
                         (int)coMpLetiOn.model_index, coMpLetiOn.epoch, this->current_epoch_);
            }
        }
        CoMpLetions_received = dRainEd_VaLiD;
        if (dRainEd_VaLiD > 0) {
            ESP_LOGD(TAG, "Credited %d early completions for epoch %llu", dRainEd_VaLiD, this->current_epoch_);
        }

        while (CoMpLetions_received < this->models_to_run_this_epoch_) {
          TickType_t tIme_waiTeD = xTaskGetTickCount() - start_wAiT;
          if (tIme_waiTeD >= ToTal_MAX_waiT) {
            ESP_LOGE(TAG, "Epoch %llu: Timed out waiting for models (total %lu ms). Expected %d, got %d.",
                     this->current_epoch_, (unsigned long)pdTICKS_TO_MS(tIme_waiTeD), this->models_to_run_this_epoch_, CoMpLetions_received);
            break; 
          }

          TickType_t waIT_RemaIning = ToTal_MAX_waiT - tIme_waiTeD;
          
          if (xQueueReceive(this->completion_queue_, &coMpLetiOn, waIT_RemaIning) == pdTRUE) {
            if (coMpLetiOn.epoch == this->current_epoch_) {
              CoMpLetions_received++;
              ESP_LOGD(TAG, "Epoch %llu: Got coMpLetiOn from model %d", this->current_epoch_, (int)coMpLetiOn.model_index);
            } else {
              ESP_LOGW(TAG, "Epoch %llu: Received stale coMpLetiOn from model %d (epoch %llu)",
                       this->current_epoch_, (int)coMpLetiOn.model_index, coMpLetiOn.epoch);
            }
          }
        }
        
        if (CoMpLetions_received < this->models_to_run_this_epoch_) {
            ESP_LOGW(TAG, "Epoch %llu: Partial completions (%d/%d) - continuing anyway.", 
                     this->current_epoch_, CoMpLetions_received, this->models_to_run_this_epoch_);
        }
      }
      // --- End Wait ---

      // --- Fuse results and detect wake words ---
      // This part is unchanged; it checks all models regardless of how they ran.
      if (this->detect_wake_words_()) {
        ESP_LOGD(TAG, "Wake Word '%s' Detected", (this->detected_wake_word_).c_str());
        this->detected_ = true;
        this->detection_callbacks_.call(this->detected_wake_word_);
        this->detected_wake_word_ = "";
        this->reset_states_();
        ESP_LOGD(TAG, "Wake word component reset and ready for next detection.");
      }
    }
    
    // ... (Error handling) ...
    if (this->state_ == i2s_audio::State::ERROR) {
      ESP_LOGE(TAG, "Error detected in producer task. Initiating recovery...");
      this->unload_models_();
      this->deallocate_buffers_();
      ESP_LOGI(TAG, "Waiting 1 second before recovery attempt...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      
      if (this->load_models_() && this->allocate_buffers_()) {
        this->reset_states_();
        this->state_ = i2s_audio::State::RUNNING;
        ESP_LOGI(TAG, "Recovery successful, resuming wake word detection.");
      } else {
        this->report_error(i2s_audio::ErrorCode::GENERIC_ERROR, "Recovery failed");
        vTaskDelay(pdMS_TO_TICKS(2000));
      }
    } else if (this->state_ == i2s_audio::State::IDLE) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
  
  ESP_LOGI(TAG, "Producer task exiting (state changed to STOPPED).");
  xEventGroupSetBits(this->task_events_, TASK_DONE_BIT);
  this->processing_task_handle_ = nullptr;
  vTaskDelete(NULL);
}

    void IRAM_ATTR MicroWakeWord::on_audio_received(const int16_t *data, size_t num_samples) {
      if (this->state_ != i2s_audio::State::RUNNING) {
        return; 
      }
      size_t bytes_written = this->ring_buffer_->write((void *)data, num_samples * sizeof(int16_t));
      if (bytes_written < num_samples * sizeof(int16_t)) {
        this->report_error(i2s_audio::ErrorCode::RING_BUFFER_WRITE_FAILED, "Ring buffer overflow");
      }
      xSemaphoreGive(this->data_sem_);
    }

    bool MicroWakeWord::allocate_buffers_()
    {
      this->ring_buffer_ = RingBuffer::create(BUFFER_SIZE * sizeof(int16_t));
      if (!this->ring_buffer_) {
        this->report_error(i2s_audio::ErrorCode::ALLOC_FAILED, "Failed to allocate ring buffer");
        return false;
      }

      // Allocate preprocessor_audio_buffer_ in internal RAM, aligned for DMA/speed
      size_t audio_size = this->new_samples_to_get_() * sizeof(int16_t);
      this->preprocessor_audio_buffer_ = static_cast<int16_t*>(heap_caps_aligned_alloc(4, audio_size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT | MALLOC_CAP_DMA));
      if (!this->preprocessor_audio_buffer_) {
        this->report_error(i2s_audio::ErrorCode::ALLOC_FAILED, "Failed to allocate preprocessor audio buffer in internal RAM");
        return false;
      }
      ESP_LOGI(TAG, "Preprocessor buffer allocated in internal RAM at %p", this->preprocessor_audio_buffer_);

      return true;
    }

    void MicroWakeWord::deallocate_buffers_()
    {
      this->ring_buffer_.reset();

      if (this->preprocessor_audio_buffer_) {
        heap_caps_free(this->preprocessor_audio_buffer_);
        this->preprocessor_audio_buffer_ = nullptr;
      }
    }

    bool MicroWakeWord::load_models_()
    {
      if (!FrontendPopulateState(&this->frontend_config_, &this->frontend_state_,
                                 AUDIO_SAMPLE_FREQUENCY))
      {
        this->report_error(i2s_audio::ErrorCode::FRONTEND_POPULATE_FAILED, "Failed to populate frontend state");
        FrontendFreeStateContents(&this->frontend_state_);
        return false;
      }

      for (auto &context : this->model_contexts_)
      {
        if (!context.model->load_model(this->streaming_op_resolver_))
        {
          ESP_LOGE(TAG, "Failed to initialize model at index %d. Model data is NULL or corrupt.", (int)context.model_index);
          this->report_error(i2s_audio::ErrorCode::WAKE_WORD_MODEL_INIT_FAILED, 
                             "Failed to initialize a model (see logs for index).");
          return false;
        }
      }
      return true;
    }

    void MicroWakeWord::unload_models_()
    {
      FrontendFreeStateContents(&this->frontend_state_);

      for (auto &context : this->model_contexts_)
      {
        context.model->unload_model();
      }
    }

    bool MicroWakeWord::detect_wake_words_()
    {
      if (this->ignore_windows_ < 0)
      {
        this->ignore_windows_ = std::min(this->ignore_windows_ + 1, 0);
        return false;
      }

#ifdef USE_MICRO_WAKE_WORD_VAD
      bool vad_state = false;
      if (this->vad_model_) {
         vad_state = this->vad_model_->determine_detected();
      }
#endif

      for (auto &context : this->model_contexts_)
      {
#ifdef USE_MICRO_WAKE_WORD_VAD
        if (context.model_index == MAX_WAKE_WORD_MODELS) {
          continue; 
        }
#endif
        if (context.model->determine_detected())
        {
#ifdef USE_MICRO_WAKE_WORD_VAD
          if (vad_state)
          {
#endif
            this->detected_wake_word_ = context.model->get_wake_word();
            return true;
#ifdef USE_MICRO_WAKE_WORD_VAD
          }
          else
          {
            ESP_LOGD(TAG, "Wake word model predicts %s, but VAD model doesn't.",
                     context.model->get_wake_word().c_str());
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

    bool IRAM_ATTR MicroWakeWord::generate_features_for_window_(
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
        this->report_error(i2s_audio::ErrorCode::RING_BUFFER_READ_FAILED, "Could not read data from Ring Buffer");
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
      
      for (auto &context : this->model_contexts_)
      {
        context.model->reset_probabilities();
      }
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