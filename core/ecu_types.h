#pragma once

#include <cstdint>

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
