#pragma once

#include "esphome/core/helpers.h"

namespace esphome {
namespace microphone {

enum state_t : uint8_t {
  IDLE = 0,
  RUNNING,
  STOPPED,
  ERROR,
};

inline bool is_running(state_t s) { return s == state_t::RUNNING; }
inline bool has_failed(state_t s) { return s == state_t::ERROR; }

class Microphone {
 public:
  virtual void start() = 0;
  virtual void stop() = 0;
  void add_data_callback(std::function<void(const std::vector<int16_t> &)> &&data_callback) {
    this->data_callbacks_.add(std::move(data_callback));
  }
  virtual size_t read(int16_t *buf, size_t len) = 0;

  bool is_running() const { return this->state_ == state_t::RUNNING; }
  bool is_stopped() const { return this->state_ == state_t::STOPPED; }

 protected:
  state_t state_{STOPPED};

  CallbackManager<void(const std::vector<int16_t> &)> data_callbacks_{};
};

}  // namespace microphone
}  // namespace esphome