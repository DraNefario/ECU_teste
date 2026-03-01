#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "wokwi" / "sketch.ino"

ORDER = [
    "core/ecu_types.h",
    "core/ecu_maps.h",
    "core/ecu_core.h",
    "hal/hal_interfaces.h",
    "hal/esp32_hal.h",
    "app/ecu_app.h",
    "core/ecu_maps.cpp",
    "core/ecu_core.cpp",
    "hal/esp32_hal.cpp",
    "app/ecu_app.cpp",
    "sketch.ino",
]


def clean_module(text: str) -> list[str]:
    out: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped == "#pragma once":
            continue
        if stripped.startswith('#include "') or stripped.startswith('#include <'):
            continue
        out.append(line)
    return out


def main() -> None:
    system_includes: list[str] = []
    for rel in ORDER:
        for line in (ROOT / rel).read_text().splitlines():
            s = line.strip()
            if s.startswith("#include <") and s not in system_includes:
                system_includes.append(s)

    if "#include <Arduino.h>" in system_includes:
        system_includes.remove("#include <Arduino.h>")
    system_includes.insert(0, "#include <Arduino.h>")

    lines: list[str] = []
    lines.append("// AUTO-GERADO por tools/make_wokwi_sketch_manual.py")
    lines.append("// Nao editar manualmente: altere arquivos em core/, hal/, app/ e regenere.")
    lines.extend(system_includes)
    lines.append("")

    for rel in ORDER:
        src = ROOT / rel
        lines.append(f"// ===== BEGIN MODULE: {rel} =====")
        lines.extend(clean_module(src.read_text()))
        lines.append(f"// ===== END MODULE: {rel} =====")
        lines.append("")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(lines).rstrip() + "\n")
    print(f"Generated: {OUT}")


if __name__ == "__main__":
    main()
