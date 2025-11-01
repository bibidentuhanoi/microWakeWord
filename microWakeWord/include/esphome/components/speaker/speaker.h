// Updated speaker.h
#pragma once

#include "esphome/core/helpers.h"

namespace esphome
{
    namespace speaker
    {

        enum state_t : uint8_t
        {
            IDLE = 0,
            RUNNING,
            STOPPED,
            ERROR
        };

        inline bool is_running(state_t s) { return s == state_t::RUNNING; }  // Now inline
        inline bool has_failed(state_t s) { return s == state_t::ERROR; }    // Now inline
        class Speaker
        {
        public:
            virtual void start() = 0;
            virtual void stop() = 0;

            // Write audio data to the speaker
            // Returns true if successful, false otherwise
            virtual size_t write(const uint8_t *data, size_t length, bool finish = false) = 0;

            // Optional callback for when the speaker is ready for more data
            void add_ready_callback(std::function<void()> &&ready_callback)
            {
                this->ready_callbacks_.add(std::move(ready_callback));
            }

            // Status reporting
            bool is_running() const { return this->state_ == state_t::RUNNING; }
            bool is_stopped() const { return this->state_ == state_t::IDLE || this->state_ == state_t::STOPPED; }

            // Check if the speaker can accept more data
            virtual bool has_buffer_space() { return this->is_running(); }

        protected:
            state_t state_{state_t::IDLE};

            // Callbacks for when the speaker is ready for more data
            CallbackManager<void()> ready_callbacks_{};

            // Status tracking for error reporting
            bool status_has_warning_{false};

            void status_set_error() { this->state_ = state_t::ERROR; }
            void status_clear_error() { this->state_ = state_t::IDLE; }
            void status_set_warning() { this->status_has_warning_ = true; }
            void status_clear_warning() { this->status_has_warning_ = false; }
            bool is_failed() const { return this->state_ == state_t::ERROR; }
        };

    } // namespace speaker
} // namespace esphome