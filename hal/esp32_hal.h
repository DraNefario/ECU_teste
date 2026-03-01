#pragma once

#include <Arduino.h>

#include "hal_interfaces.h"

namespace ecu::hal {

class Esp32Hal final : public IEcuHal {
 public:
  static constexpr int kPinNe = 4;
  static constexpr int kPinG = 5;
  static constexpr int kPinIgt = 6;
  static constexpr int kPinInj = 7;
  static constexpr int kPinTps = 34;

  Esp32Hal();
  void begin();

  void attachSignalInterrupts(void (*onG)(), void (*onNe)());
  void attachTimerCallbacks(void (*onIgnTimer)(), void (*onInjTimer)());

  uint32_t microsNow() const override;
  uint32_t millisNow() const override;
  void setIgnitionPin(bool high) override;
  void setInjectorPin(bool high) override;
  void armIgnitionTimerUs(uint32_t us) override;
  void armInjectorTimerUs(uint32_t us) override;
  void stopIgnitionTimer() override;
  void stopInjectorTimer() override;
  uint16_t readTpsRaw() override;

 private:
  hw_timer_t *ignTimer_ = nullptr;
  hw_timer_t *injTimer_ = nullptr;
};

}  // namespace ecu::hal
