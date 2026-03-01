#include "ecu_core.h"

#include "ecu_maps.h"

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
