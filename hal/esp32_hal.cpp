#include "esp32_hal.h"

namespace ecu::hal {

namespace {
inline void armOneShot(hw_timer_t *timer, uint32_t us) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timerAlarm(timer, us, false, 0);
#else
  timerAlarmWrite(timer, us, false);
  timerAlarmEnable(timer);
#endif
}

inline void stopTimer(hw_timer_t *timer) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timerStop(timer);
#else
  timerAlarmDisable(timer);
#endif
}
}  // namespace

Esp32Hal::Esp32Hal() = default;

void Esp32Hal::begin() {
  pinMode(kPinNe, INPUT);
  pinMode(kPinG, INPUT);
  pinMode(kPinIgt, OUTPUT);
  pinMode(kPinInj, OUTPUT);
  pinMode(kPinTps, INPUT);

  digitalWrite(kPinIgt, LOW);
  digitalWrite(kPinInj, LOW);
  analogReadResolution(12);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ignTimer_ = timerBegin(1000000);
  injTimer_ = timerBegin(1000000);
#else
  ignTimer_ = timerBegin(0, 80, true);
  injTimer_ = timerBegin(1, 80, true);
#endif
}

void Esp32Hal::attachSignalInterrupts(void (*onG)(), void (*onNe)()) {
  attachInterrupt(digitalPinToInterrupt(kPinG), onG, RISING);
  attachInterrupt(digitalPinToInterrupt(kPinNe), onNe, RISING);
}

void Esp32Hal::attachTimerCallbacks(void (*onIgnTimer)(), void (*onInjTimer)()) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  timerAttachInterrupt(ignTimer_, onIgnTimer);
  timerAttachInterrupt(injTimer_, onInjTimer);
#else
  timerAttachInterrupt(ignTimer_, onIgnTimer, true);
  timerAttachInterrupt(injTimer_, onInjTimer, true);
#endif
}

uint32_t Esp32Hal::microsNow() const { return micros(); }
uint32_t Esp32Hal::millisNow() const { return millis(); }

void Esp32Hal::setIgnitionPin(bool high) { digitalWrite(kPinIgt, high ? HIGH : LOW); }
void Esp32Hal::setInjectorPin(bool high) { digitalWrite(kPinInj, high ? HIGH : LOW); }

void Esp32Hal::armIgnitionTimerUs(uint32_t us) { armOneShot(ignTimer_, us); }
void Esp32Hal::armInjectorTimerUs(uint32_t us) { armOneShot(injTimer_, us); }
void Esp32Hal::stopIgnitionTimer() { stopTimer(ignTimer_); }
void Esp32Hal::stopInjectorTimer() { stopTimer(injTimer_); }

uint16_t Esp32Hal::readTpsRaw() { return analogRead(kPinTps); }

}  // namespace ecu::hal
