#include "ecu_maps.h"

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
