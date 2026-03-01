#pragma once

#include <cstdint>

namespace ecu::hal {

class IEcuHal {
 public:
  virtual ~IEcuHal() = default;

  virtual uint32_t microsNow() const = 0;
  virtual uint32_t millisNow() const = 0;

  virtual void setIgnitionPin(bool high) = 0;
  virtual void setInjectorPin(bool high) = 0;

  virtual void armIgnitionTimerUs(uint32_t us) = 0;
  virtual void armInjectorTimerUs(uint32_t us) = 0;
  virtual void stopIgnitionTimer() = 0;
  virtual void stopInjectorTimer() = 0;

  virtual uint16_t readTpsRaw() = 0;
};

}  // namespace ecu::hal
