#pragma once

#include "ecu_types.h"

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
