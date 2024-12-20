#pragma once

#ifdef USE_ESP32

#include <driver/i2s_std.h>
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome
{
  namespace i2s_audio
  {
    class I2SAudioComponent : public Component
    {
    public:
      void setup() override;

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
      Mutex lock_;

      gpio_num_t mclk_pin_{I2S_GPIO_UNUSED};
      gpio_num_t bclk_pin_{I2S_GPIO_UNUSED};
      gpio_num_t lrclk_pin_;
      i2s_port_t port_{};
    };

  } // namespace i2s_audio
} // namespace esphome

#endif // USE_ESP32
