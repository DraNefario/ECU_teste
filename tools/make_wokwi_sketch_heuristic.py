#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "wokwi" / "sketch.ino"

INCLUDE_LOCAL = re.compile(r'^\s*#include\s+"([^"]+)"')
INCLUDE_SYS = re.compile(r'^\s*#include\s+<([^>]+)>')


def strip_local_includes_and_pragma(path: Path) -> tuple[list[str], list[str]]:
    sys_inc: list[str] = []
    body: list[str] = []
    for line in path.read_text().splitlines():
        s = line.strip()
        if s == "#pragma once":
            continue
        if INCLUDE_LOCAL.match(line):
            continue
        m = INCLUDE_SYS.match(line)
        if m:
            sys_inc.append(f"#include <{m.group(1)}>")
            continue
        body.append(line)
    return sys_inc, body


def score_file(path: Path) -> tuple[int, int, int]:
    txt = path.read_text()
    local_count = len(INCLUDE_LOCAL.findall(txt))
    sys_count = len(INCLUDE_SYS.findall(txt))
    # headers primeiro, depois cpp, dentro disso mais dependencias primeiro
    ext_priority = 0 if path.suffix == ".h" else 1
    return (ext_priority, -local_count, -sys_count)


def gather_order() -> list[Path]:
    candidates: list[Path] = []
    for base in ["core", "hal", "app"]:
        d = ROOT / base
        candidates.extend(sorted(d.glob("*.h")))
        candidates.extend(sorted(d.glob("*.cpp")))

    # Ordenacao heuristica independente de parser topologico completo
    candidates.sort(key=score_file)

    # garante sketch por ultimo
    candidates.append(ROOT / "sketch.ino")
    return candidates


def main() -> None:
    order = gather_order()

    includes: list[str] = []
    chunks: list[tuple[str, list[str]]] = []

    for p in order:
        if "tests" in p.parts:
            continue
        sys_inc, body = strip_local_includes_and_pragma(p)
        for i in sys_inc:
            if i not in includes:
                includes.append(i)
        chunks.append((str(p.relative_to(ROOT)), body))

    if "#include <Arduino.h>" in includes:
        includes.remove("#include <Arduino.h>")
    includes.insert(0, "#include <Arduino.h>")

    out: list[str] = []
    out.append("// AUTO-GERADO por tools/make_wokwi_sketch_heuristic.py")
    out.append("// Estrategia: ordenacao heuristica por tipo/complexidade de includes")
    out.extend(includes)
    out.append("")

    for name, body in chunks:
        out.append(f"// ===== BEGIN MODULE: {name} =====")
        out.extend(body)
        out.append(f"// ===== END MODULE: {name} =====")
        out.append("")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(out).rstrip() + "\n")
    print(f"Generated: {OUT}")


if __name__ == "__main__":
    main()
