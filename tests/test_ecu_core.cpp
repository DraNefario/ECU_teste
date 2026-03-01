#include <cassert>
#include <cstdint>
#include <iostream>

#include "../core/ecu_core.h"
#include "../core/ecu_maps.h"

int main() {
  using namespace ecu::core;

  Calibration cal{180, 3900};
  EcuCore core(cal);

  // Sem sync, NE não gera ignição
  auto r0 = core.onPulseNE(1000);
  assert(!r0.shouldChargeIgnition);

  // Sync por G e sequência 1-3-4-2
  core.onPulseG();
  core.onPulseNE(8000);
  assert(core.state().lastEventIndex == 0);
  core.onPulseNE(15000);
  assert(core.state().lastEventIndex == 1);
  core.onPulseNE(22000);
  assert(core.state().lastEventIndex == 2);
  core.onPulseNE(29000);
  assert(core.state().lastEventIndex == 3);

  // Conversão TPS calibrada
  assert(EcuMaps::tpsRawToPercent(180, 180, 3900) == 0);
  assert(EcuMaps::tpsRawToPercent(3900, 180, 3900) == 100);
  const auto mid = EcuMaps::tpsRawToPercent(2040, 180, 3900);
  assert(mid > 45 && mid < 55);

  // Combustível >0 com motor girando
  uint32_t pw = core.computeFuelPulseUs(2000);
  assert(pw >= 800);

  std::cout << "OK\n";
  return 0;
}
