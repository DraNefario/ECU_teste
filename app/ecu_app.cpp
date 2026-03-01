#include "ecu_app.h"

#include <Arduino.h>
#include <Preferences.h>

#include "../core/ecu_core.h"
#include "../core/ecu_maps.h"
#include "../hal/esp32_hal.h"

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
