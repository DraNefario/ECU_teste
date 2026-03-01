#pragma once

#include <cstdint>

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
