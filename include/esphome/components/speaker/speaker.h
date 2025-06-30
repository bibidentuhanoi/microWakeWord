#pragma once

#include "esphome/core/helpers.h"

namespace esphome
{
    namespace speaker
    {

        enum State : uint8_t
        {
            STATE_STOPPED = 0,
            STATE_STARTING,
            STATE_RUNNING,
            STATE_STOPPING,
        };

        class Speaker
        {
        public:
            virtual void start() = 0;
            virtual void stop() = 0;

            // Write audio data to the speaker
            // Returns true if successful, false otherwise
            virtual size_t write(const uint8_t *data, size_t length) = 0;

            // Optional callback for when the speaker is ready for more data
            void add_ready_callback(std::function<void()> &&ready_callback)
            {
                this->ready_callbacks_.add(std::move(ready_callback));
            }

            // Status reporting
            bool is_running() const { return this->state_ == STATE_RUNNING; }
            bool is_stopped() const { return this->state_ == STATE_STOPPED; }

            // Check if the speaker can accept more data
            virtual bool has_buffer_space() { return this->is_running(); }

        protected:
            State state_{STATE_STOPPED};

            // Callbacks for when the speaker is ready for more data
            CallbackManager<void()> ready_callbacks_{};

            // Status tracking for error reporting
            bool status_has_error_{false};
            bool status_has_warning_{false};

            void status_set_error() { this->status_has_error_ = true; }
            void status_clear_error() { this->status_has_error_ = false; }
            void status_set_warning() { this->status_has_warning_ = true; }
            void status_clear_warning() { this->status_has_warning_ = false; }
            bool is_failed() const { return this->status_has_error_; }
        };

    } // namespace speaker
} // namespace esphome