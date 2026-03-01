// AUTO-GERADO por tools/make_wokwi_sketch.py
// Estrategia consolidada: manual + topologica + heuristica
#include <Arduino.h>
#include <cstdint>
#include <Preferences.h>

// ===== BEGIN MODULE: core/ecu_types.h =====


namespace ecu::core {

struct Calibration {
  uint16_t tpsMinAdc = 180;
  uint16_t tpsMaxAdc = 3900;
};

struct RuntimeState {
  bool sync = false;
  int8_t currentIndex = -1;
  int8_t lastEventIndex = -1;
  bool gJustSeen = false;
  uint32_t neEventCounter = 0;
  uint32_t lastNeMicros = 0;
  uint32_t rpmPeriodUs = 0;

  uint16_t tpsRaw = 0;
  uint8_t tpsPercent = 0;
  uint32_t injectorPulseUs = 2400;
  uint32_t dwellUs = 2500;
  uint16_t vePermille = 700;
  int16_t ignDeg = 10;
};

struct NePulseResult {
  bool shouldChargeIgnition = false;
  uint32_t dwellUs = 2500;
  bool hasNewEvent = false;
};

}  // namespace ecu::core
// ===== END MODULE: core/ecu_types.h =====

// ===== BEGIN MODULE: core/ecu_maps.h =====


namespace ecu::core {

class EcuMaps {
 public:
  static constexpr uint8_t kRpmLen = 7;
  static constexpr uint8_t kTpsLen = 7;

  static uint8_t tpsRawToPercent(uint16_t raw, uint16_t minAdc, uint16_t maxAdc);
  static uint32_t rpmFromPeriodUs(uint32_t periodUs);
  static uint32_t computeDwellUs(uint32_t periodUs);
  static uint32_t computeInjectionPwUs(uint32_t periodUs, uint16_t tpsRaw, uint16_t minAdc, uint16_t maxAdc,
                                       uint16_t &vePermille, int16_t &ignDeg, uint8_t &tpsPercentOut);

 private:
  static float lookupVe(uint16_t rpm, uint8_t tpsPct);
  static float lookupIgnDeg(uint16_t rpm, uint8_t tpsPct);
  static uint8_t findAxisIndexU16(const uint16_t *axis, uint8_t len, uint16_t value);
  static uint8_t findAxisIndexU8(const uint8_t *axis, uint8_t len, uint8_t value);
  static float interp1D(float x0, float x1, float y0, float y1, float x);
  static float interp2D(float x0, float x1, float y0, float y1,
                        float q11, float q21, float q12, float q22,
                        float x, float y);
};

}  // namespace ecu::core
// ===== END MODULE: core/ecu_maps.h =====

// ===== BEGIN MODULE: core/ecu_core.h =====


namespace ecu::core {

class EcuCore {
 public:
  explicit EcuCore(const Calibration &cal);

  void setCalibration(const Calibration &cal);
  const Calibration &calibration() const;
  const RuntimeState &state() const;

  void onPulseG();
  NePulseResult onPulseNE(uint32_t nowUs);
  void onSyncTimeout();

  // Atualiza cadeia de combustível com leitura de TPS (já filtrada externamente)
  uint32_t computeFuelPulseUs(uint16_t tpsRawFiltered);

 private:
  Calibration cal_;
  RuntimeState st_;
};

}  // namespace ecu::core
// ===== END MODULE: core/ecu_core.h =====

// ===== BEGIN MODULE: hal/hal_interfaces.h =====


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
// ===== END MODULE: hal/hal_interfaces.h =====

// ===== BEGIN MODULE: hal/esp32_hal.h =====



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
// ===== END MODULE: hal/esp32_hal.h =====

// ===== BEGIN MODULE: app/ecu_app.h =====

namespace ecu::app {

void setupApp();
void loopApp();

}  // namespace ecu::app
// ===== END MODULE: app/ecu_app.h =====

// ===== BEGIN MODULE: core/ecu_maps.cpp =====

namespace ecu::core {
namespace {
constexpr uint16_t RPM_AXIS[EcuMaps::kRpmLen] = {800, 1200, 1800, 2600, 3600, 4800, 6200};
constexpr uint8_t TPS_AXIS[EcuMaps::kTpsLen] = {0, 10, 25, 40, 60, 80, 100};

constexpr uint8_t VE_TABLE[EcuMaps::kRpmLen][EcuMaps::kTpsLen] = {
    {70, 72, 76, 80, 85, 90, 94},
    {68, 72, 78, 84, 90, 96, 101},
    {66, 72, 80, 88, 96, 103, 108},
    {65, 73, 82, 91, 100, 108, 114},
    {64, 74, 84, 94, 104, 112, 117},
    {62, 72, 82, 92, 102, 109, 114},
    {58, 68, 76, 85, 94, 100, 104}};

constexpr int8_t IGN_TABLE[EcuMaps::kRpmLen][EcuMaps::kTpsLen] = {
    {18, 18, 16, 14, 12, 10, 8},
    {24, 24, 22, 20, 18, 16, 14},
    {30, 30, 28, 26, 24, 22, 20},
    {36, 36, 34, 31, 28, 25, 22},
    {40, 40, 37, 34, 30, 27, 24},
    {38, 38, 35, 32, 29, 26, 23},
    {34, 34, 31, 28, 26, 24, 22}};
}  // namespace

uint8_t EcuMaps::tpsRawToPercent(uint16_t raw, uint16_t minAdc, uint16_t maxAdc) {
  if (maxAdc <= minAdc + 10) return 0;
  if (raw <= minAdc) return 0;
  if (raw >= maxAdc) return 100;
  const uint32_t span = static_cast<uint32_t>(maxAdc - minAdc);
  return static_cast<uint8_t>((static_cast<uint32_t>(raw - minAdc) * 100U) / span);
}

uint32_t EcuMaps::rpmFromPeriodUs(uint32_t periodUs) {
  if (periodUs == 0) return 0;
  return 30000000UL / periodUs;
}

uint32_t EcuMaps::computeDwellUs(uint32_t periodUs) {
  if (periodUs == 0) return 2500;
  uint32_t dwell = periodUs / 7;
  if (dwell < 1500) dwell = 1500;
  if (dwell > 4500) dwell = 4500;
  return dwell;
}

uint32_t EcuMaps::computeInjectionPwUs(uint32_t periodUs, uint16_t tpsRaw, uint16_t minAdc, uint16_t maxAdc,
                                       uint16_t &vePermille, int16_t &ignDeg, uint8_t &tpsPercentOut) {
  const auto rpm = static_cast<uint16_t>(rpmFromPeriodUs(periodUs));
  const auto tpsPct = tpsRawToPercent(tpsRaw, minAdc, maxAdc);
  tpsPercentOut = tpsPct;

  const float ve = lookupVe(rpm, tpsPct);
  float correction = 1.0f;
  if (rpm < 1000 && tpsPct > 35) correction += 0.06f;

  uint32_t pw = static_cast<uint32_t>(2400.0f * ve * correction);
  if (pw < 800) pw = 800;
  if (pw > 8000) pw = 8000;

  vePermille = static_cast<uint16_t>(ve * 1000.0f);
  ignDeg = static_cast<int16_t>(lookupIgnDeg(rpm, tpsPct));
  return pw;
}

float EcuMaps::lookupVe(uint16_t rpm, uint8_t tpsPct) {
  const uint8_t r = findAxisIndexU16(RPM_AXIS, kRpmLen, rpm);
  const uint8_t t = findAxisIndexU8(TPS_AXIS, kTpsLen, tpsPct);
  return interp2D(static_cast<float>(RPM_AXIS[r]), static_cast<float>(RPM_AXIS[r + 1]),
                  static_cast<float>(TPS_AXIS[t]), static_cast<float>(TPS_AXIS[t + 1]),
                  static_cast<float>(VE_TABLE[r][t]) / 100.0f,
                  static_cast<float>(VE_TABLE[r + 1][t]) / 100.0f,
                  static_cast<float>(VE_TABLE[r][t + 1]) / 100.0f,
                  static_cast<float>(VE_TABLE[r + 1][t + 1]) / 100.0f,
                  static_cast<float>(rpm), static_cast<float>(tpsPct));
}

float EcuMaps::lookupIgnDeg(uint16_t rpm, uint8_t tpsPct) {
  const uint8_t r = findAxisIndexU16(RPM_AXIS, kRpmLen, rpm);
  const uint8_t t = findAxisIndexU8(TPS_AXIS, kTpsLen, tpsPct);
  return interp2D(static_cast<float>(RPM_AXIS[r]), static_cast<float>(RPM_AXIS[r + 1]),
                  static_cast<float>(TPS_AXIS[t]), static_cast<float>(TPS_AXIS[t + 1]),
                  static_cast<float>(IGN_TABLE[r][t]), static_cast<float>(IGN_TABLE[r + 1][t]),
                  static_cast<float>(IGN_TABLE[r][t + 1]), static_cast<float>(IGN_TABLE[r + 1][t + 1]),
                  static_cast<float>(rpm), static_cast<float>(tpsPct));
}

uint8_t EcuMaps::findAxisIndexU16(const uint16_t *axis, uint8_t len, uint16_t value) {
  if (value <= axis[0]) return 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    if (value < axis[i + 1]) return i;
  }
  return len - 2;
}

uint8_t EcuMaps::findAxisIndexU8(const uint8_t *axis, uint8_t len, uint8_t value) {
  if (value <= axis[0]) return 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    if (value < axis[i + 1]) return i;
  }
  return len - 2;
}

float EcuMaps::interp1D(float x0, float x1, float y0, float y1, float x) {
  if (x1 <= x0) return y0;
  const float t = (x - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

float EcuMaps::interp2D(float x0, float x1, float y0, float y1,
                       float q11, float q21, float q12, float q22,
                       float x, float y) {
  const float r1 = interp1D(x0, x1, q11, q21, x);
  const float r2 = interp1D(x0, x1, q12, q22, x);
  return interp1D(y0, y1, r1, r2, y);
}

}  // namespace ecu::core
// ===== END MODULE: core/ecu_maps.cpp =====

// ===== BEGIN MODULE: core/ecu_core.cpp =====


namespace ecu::core {

EcuCore::EcuCore(const Calibration &cal) : cal_(cal) {}

void EcuCore::setCalibration(const Calibration &cal) { cal_ = cal; }

const Calibration &EcuCore::calibration() const { return cal_; }

const RuntimeState &EcuCore::state() const { return st_; }

void EcuCore::onPulseG() {
  st_.sync = true;
  st_.currentIndex = 0;
  st_.gJustSeen = true;
}

NePulseResult EcuCore::onPulseNE(uint32_t nowUs) {
  NePulseResult out{};

  if (st_.lastNeMicros > 0) {
    st_.rpmPeriodUs = nowUs - st_.lastNeMicros;
    st_.dwellUs = EcuMaps::computeDwellUs(st_.rpmPeriodUs);
  }
  st_.lastNeMicros = nowUs;

  if (st_.sync) {
    if (st_.gJustSeen) {
      st_.gJustSeen = false;
    } else {
      st_.currentIndex++;
      if (st_.currentIndex > 3) st_.currentIndex = 0;
    }
    st_.lastEventIndex = st_.currentIndex;
    out.shouldChargeIgnition = true;
    out.dwellUs = st_.dwellUs;
  } else {
    st_.lastEventIndex = -1;
  }

  st_.neEventCounter++;
  out.hasNewEvent = true;
  return out;
}

void EcuCore::onSyncTimeout() {
  st_.sync = false;
  st_.currentIndex = -1;
  st_.lastEventIndex = -1;
  st_.rpmPeriodUs = 0;
}

uint32_t EcuCore::computeFuelPulseUs(uint16_t tpsRawFiltered) {
  st_.tpsRaw = tpsRawFiltered;

  st_.injectorPulseUs = EcuMaps::computeInjectionPwUs(st_.rpmPeriodUs, tpsRawFiltered, cal_.tpsMinAdc, cal_.tpsMaxAdc,
                                                      st_.vePermille, st_.ignDeg, st_.tpsPercent);

  if (st_.rpmPeriodUs > 600 && st_.injectorPulseUs > (st_.rpmPeriodUs - 400)) {
    st_.injectorPulseUs = st_.rpmPeriodUs - 400;
  }
  if (st_.injectorPulseUs < 800) st_.injectorPulseUs = 800;
  if (st_.injectorPulseUs > 8000) st_.injectorPulseUs = 8000;

  return st_.injectorPulseUs;
}

}  // namespace ecu::core
// ===== END MODULE: core/ecu_core.cpp =====

// ===== BEGIN MODULE: hal/esp32_hal.cpp =====

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
// ===== END MODULE: hal/esp32_hal.cpp =====

// ===== BEGIN MODULE: app/ecu_app.cpp =====



namespace ecu::app {
namespace {
using ecu::core::Calibration;
using ecu::core::EcuCore;
using ecu::core::NePulseResult;

Preferences prefs;
Calibration calib;
EcuCore core(calib);
ecu::hal::Esp32Hal hal;
TaskHandle_t fuelTaskHandle = nullptr;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool injectorOn = false;
bool fuelTaskAlive = false;

Calibration loadCalibration() {
  Calibration c;
  prefs.begin("ecu", true);
  c.tpsMinAdc = prefs.getUShort("tpsMin", 180);
  c.tpsMaxAdc = prefs.getUShort("tpsMax", 3900);
  prefs.end();
  if (c.tpsMaxAdc <= c.tpsMinAdc + 100) {
    c.tpsMinAdc = 180;
    c.tpsMaxAdc = 3900;
  }
  return c;
}

void saveCalibration(const Calibration &c) {
  prefs.begin("ecu", false);
  prefs.putUShort("tpsMin", c.tpsMinAdc);
  prefs.putUShort("tpsMax", c.tpsMaxAdc);
  prefs.end();
}

void IRAM_ATTR onIgnTimer() {
  portENTER_CRITICAL_ISR(&mux);
  hal.setIgnitionPin(false);
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onInjTimer() {
  portENTER_CRITICAL_ISR(&mux);
  hal.setInjectorPin(false);
  injectorOn = false;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR onPulseG() {
  core.onPulseG();
}

void IRAM_ATTR onPulseNE() {
  const NePulseResult res = core.onPulseNE(hal.microsNow());
  if (res.shouldChargeIgnition) {
    hal.setIgnitionPin(true);
    hal.armIgnitionTimerUs(res.dwellUs);
  }

  if (fuelTaskHandle != nullptr) {
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(fuelTaskHandle, &hp);
    if (hp == pdTRUE) portYIELD_FROM_ISR();
  }
}

void fuelTask(void *) {
  fuelTaskAlive = true;
  uint16_t tpsFiltered = 0;

  while (true) {
    const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    if (notified == 0) continue;

    const auto &st = core.state();
    if (!st.sync || st.rpmPeriodUs == 0) continue;

    const uint16_t adcRaw = hal.readTpsRaw();
    tpsFiltered = static_cast<uint16_t>((3U * tpsFiltered + adcRaw) / 4U);

    const uint32_t pwUs = core.computeFuelPulseUs(tpsFiltered);
    hal.setInjectorPin(true);
    injectorOn = true;
    hal.armInjectorTimerUs(pwUs);
  }
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  const String cmd = Serial.readStringUntil('\n');
  Calibration c = core.calibration();

  if (cmd.startsWith("tpsmin")) {
    c.tpsMinAdc = hal.readTpsRaw();
    saveCalibration(c);
    core.setCalibration(c);
    Serial.printf("TPS_MIN salvo: %u\n", c.tpsMinAdc);
  } else if (cmd.startsWith("tpsmax")) {
    c.tpsMaxAdc = hal.readTpsRaw();
    saveCalibration(c);
    core.setCalibration(c);
    Serial.printf("TPS_MAX salvo: %u\n", c.tpsMaxAdc);
  } else if (cmd.startsWith("tpsreset")) {
    c.tpsMinAdc = 180;
    c.tpsMaxAdc = 3900;
    saveCalibration(c);
    core.setCalibration(c);
    Serial.println("TPS calibration reset");
  } else if (cmd.startsWith("tpsshow")) {
    Serial.printf("TPS_MIN=%u TPS_MAX=%u\n", c.tpsMinAdc, c.tpsMaxAdc);
  }
}

}  // namespace

void setupApp() {
  Serial.begin(115200);
  hal.begin();
  hal.attachTimerCallbacks(onIgnTimer, onInjTimer);

  calib = loadCalibration();
  core.setCalibration(calib);

  const BaseType_t created = xTaskCreatePinnedToCore(fuelTask, "FuelTask", 4096, nullptr, 2, &fuelTaskHandle, 1);
  if (created != pdPASS) {
    Serial.println("[ERRO] FuelTask nao criada");
  }

  hal.attachSignalInterrupts(onPulseG, onPulseNE);

  Serial.println("--- ECU 7A-FE START ---");
  Serial.println("Aguardando sincronia do distribuidor...");
  Serial.printf("TPS calib: min=%u max=%u\n", calib.tpsMinAdc, calib.tpsMaxAdc);
  Serial.println("Comandos: tpsmin | tpsmax | tpsshow | tpsreset");
}

void loopApp() {
  static uint32_t lastPrintedEvent = 0;
  static uint32_t lastHeartbeatMs = 0;

  handleSerialCommands();

  const uint32_t nowUs = hal.microsNow();
  auto st = core.state();
  if (st.lastNeMicros > 0 && (nowUs - st.lastNeMicros) > 500000UL) {
    core.onSyncTimeout();
    hal.setIgnitionPin(false);
    hal.setInjectorPin(false);
    injectorOn = false;
    hal.stopIgnitionTimer();
    hal.stopInjectorTimer();
    st = core.state();
  }

  const bool hasNewEvent = st.neEventCounter != lastPrintedEvent;
  const bool heartbeatDue = (hal.millisNow() - lastHeartbeatMs) > 500;

  if (hasNewEvent || heartbeatDue) {
    if (hasNewEvent) lastPrintedEvent = st.neEventCounter;
    lastHeartbeatMs = hal.millisNow();

    Serial.print("Status: ");
    Serial.print(st.sync ? "[SYNC OK]" : "[NO SYNC]");
    Serial.print(" | RPM: ");
    Serial.print(ecu::core::EcuMaps::rpmFromPeriodUs(st.rpmPeriodUs));
    Serial.print(" | TPSraw: ");
    Serial.print(st.tpsRaw);
    Serial.print(" | TPS%: ");
    Serial.print(st.tpsPercent);
    Serial.print(" | TPScal: ");
    Serial.print(core.calibration().tpsMinAdc);
    Serial.print("-");
    Serial.print(core.calibration().tpsMaxAdc);
    Serial.print(" | Dwell(us): ");
    Serial.print(st.dwellUs);
    Serial.print(" | InjPW(us): ");
    Serial.print(st.injectorPulseUs);
    Serial.print(" | INJ:");
    Serial.print(injectorOn ? "ON" : "OFF");
    Serial.print(" | VE(x1000): ");
    Serial.print(st.vePermille);
    Serial.print(" | IGN(deg): ");
    Serial.print(st.ignDeg);
    Serial.print(" | FuelTask: ");
    Serial.print(fuelTaskAlive ? "ON" : "OFF");
    Serial.print(" | Evento NE: ");
    Serial.print(st.neEventCounter);
    Serial.print(" | Cilindro do evento: ");
    if (st.sync && st.lastEventIndex >= 0 && st.lastEventIndex < 4) {
      static constexpr uint8_t firing[4] = {1, 3, 4, 2};
      Serial.print("Cil #");
      Serial.print(firing[st.lastEventIndex]);
      Serial.print(" (idx ");
      Serial.print(st.lastEventIndex);
      Serial.print(")");
    } else {
      Serial.print("?");
    }
    Serial.println();
  }
}

}  // namespace ecu::app
// ===== END MODULE: app/ecu_app.cpp =====

// ===== BEGIN MODULE: sketch.ino =====

void setup() {
  ecu::app::setupApp();
}

void loop() {
  ecu::app::loopApp();
}
// ===== END MODULE: sketch.ino =====
