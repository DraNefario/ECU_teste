#!/usr/bin/env python3
"""
Amalgamador consolidado para Wokwi.

Combina os melhores pontos das 3 abordagens anteriores:
1) Ordem manual preferencial para módulos críticos (determinismo)
2) Resolução por dependências locais (#include "...") com ordenação topológica
3) Fallback heurístico para arquivos não mapeados/ciclos

Saída: wokwi/sketch.ino
"""

from __future__ import annotations

from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "wokwi" / "sketch.ino"

SRC_DIRS = [ROOT / "core", ROOT / "hal", ROOT / "app"]
ENTRY = ROOT / "sketch.ino"

LOCAL_RE = re.compile(r'^\s*#include\s+"([^"]+)"')
SYS_RE = re.compile(r'^\s*#include\s+<([^>]+)>')

# Ordem manual para os módulos principais: garante previsibilidade e leitura
MANUAL_PREFERRED = [
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


def collect_sources() -> dict[str, Path]:
    files: dict[str, Path] = {}
    for d in SRC_DIRS:
        for p in sorted(d.glob("*.h")) + sorted(d.glob("*.cpp")):
            files[str(p.relative_to(ROOT))] = p
    files[str(ENTRY.relative_to(ROOT))] = ENTRY
    return files


def parse_file(path: Path) -> tuple[list[str], list[str], list[str]]:
    deps_local: list[str] = []
    includes_sys: list[str] = []
    body: list[str] = []

    for line in path.read_text().splitlines():
        s = line.strip()
        if s == "#pragma once":
            continue

        m_local = LOCAL_RE.match(line)
        if m_local:
            # Resolve include local por sufixo de arquivo no conjunto coletado
            deps_local.append(Path(m_local.group(1)).name)
            continue

        m_sys = SYS_RE.match(line)
        if m_sys:
            includes_sys.append(f"#include <{m_sys.group(1)}>")
            continue

        body.append(line)

    return deps_local, includes_sys, body


def heuristic_score(path: Path) -> tuple[int, int, int, str]:
    txt = path.read_text()
    local_count = len(LOCAL_RE.findall(txt))
    sys_count = len(SYS_RE.findall(txt))
    ext_priority = 0 if path.suffix == ".h" else 1
    # Mais includes locais primeiro (tende a ter mais dependências)
    return (ext_priority, -local_count, -sys_count, str(path))


def topological_order(files: dict[str, Path]) -> list[str]:
    name_to_rel = {p.name: rel for rel, p in files.items()}

    dep_map: dict[str, set[str]] = {}
    for rel, p in files.items():
        deps, _, _ = parse_file(p)
        resolved: set[str] = set()
        for d in deps:
            if d in name_to_rel:
                resolved.add(name_to_rel[d])
        dep_map[rel] = resolved

    ordered: list[str] = []
    ready = [k for k, v in dep_map.items() if not v]

    while ready:
        ready.sort()
        n = ready.pop(0)
        ordered.append(n)
        for k in list(dep_map.keys()):
            if n in dep_map[k]:
                dep_map[k].remove(n)
                if not dep_map[k] and k not in ordered and k not in ready:
                    ready.append(k)

    # Sobrou ciclo? usa fallback heurístico
    if len(ordered) != len(files):
        remaining = [k for k in files.keys() if k not in ordered]
        remaining.sort(key=lambda r: heuristic_score(files[r]))
        ordered.extend(remaining)

    return ordered


def consolidate_order(files: dict[str, Path]) -> list[str]:
    topo = topological_order(files)
    topo_set = set(topo)

    final: list[str] = []

    # Primeiro, ordem manual preferencial (se existir no repo)
    for rel in MANUAL_PREFERRED:
        if rel in files and rel not in final:
            final.append(rel)

    # Depois, respeita topológico para qualquer arquivo não contemplado manualmente
    for rel in topo:
        if rel not in final:
            final.append(rel)

    # Segurança
    assert set(final) == set(files.keys()) == topo_set
    return final


def build_output(order: list[str], files: dict[str, Path]) -> list[str]:
    includes: list[str] = []
    chunks: list[tuple[str, list[str]]] = []

    for rel in order:
        # Regra: não incorporar testes no Wokwi
        if rel.startswith("tests/"):
            continue

        deps, sys_inc, body = parse_file(files[rel])
        _ = deps  # dependências já usadas no ordenamento

        for inc in sys_inc:
            if inc not in includes:
                includes.append(inc)

        chunks.append((rel, body))

    # Arduino.h exatamente uma vez, no topo
    includes = [i for i in includes if i != "#include <Arduino.h>"]
    includes.insert(0, "#include <Arduino.h>")

    out: list[str] = []
    out.append("// AUTO-GERADO por tools/make_wokwi_sketch.py")
    out.append("// Estrategia consolidada: manual + topologica + heuristica")
    out.extend(includes)
    out.append("")

    for rel, body in chunks:
        out.append(f"// ===== BEGIN MODULE: {rel} =====")
        out.extend(body)
        out.append(f"// ===== END MODULE: {rel} =====")
        out.append("")

    return out


def main() -> None:
    files = collect_sources()
    order = consolidate_order(files)
    out_lines = build_output(order, files)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(out_lines).rstrip() + "\n")

    print(f"Generated: {OUT}")
    print("Order used:")
    for rel in order:
        print(f" - {rel}")


if __name__ == "__main__":
    main()
