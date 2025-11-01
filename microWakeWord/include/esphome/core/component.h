#pragma once

#include <cstdint>

namespace esphome {
class Component {
 public:
  virtual void setup();
  virtual void loop();
};



}  // namespace esphome
