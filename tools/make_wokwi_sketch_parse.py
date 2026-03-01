#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SRC_DIRS = [ROOT / "core", ROOT / "hal", ROOT / "app"]
ENTRY = ROOT / "sketch.ino"
OUT = ROOT / "wokwi" / "sketch.ino"

LOCAL_RE = re.compile(r'^\s*#include\s+"([^"]+)"')
SYS_RE = re.compile(r'^\s*#include\s+<([^>]+)>')


def collect_files() -> dict[str, Path]:
    files: dict[str, Path] = {}
    for d in SRC_DIRS:
        for p in sorted(d.glob("*.h")) + sorted(d.glob("*.cpp")):
            files[p.name] = p
    files[ENTRY.name] = ENTRY
    return files


def parse_deps(path: Path) -> tuple[list[str], list[str], list[str]]:
    deps: list[str] = []
    sys_inc: list[str] = []
    body: list[str] = []

    for line in path.read_text().splitlines():
        l = line.strip()
        if l == "#pragma once":
            continue

        m_local = LOCAL_RE.match(line)
        if m_local:
            deps.append(Path(m_local.group(1)).name)
            continue

        m_sys = SYS_RE.match(line)
        if m_sys:
            sys_inc.append(f"#include <{m_sys.group(1)}>")
            continue

        body.append(line)
    return deps, sys_inc, body


def topo_sort(files: dict[str, Path]) -> list[str]:
    dep_map: dict[str, set[str]] = {}
    for name, path in files.items():
        deps, _, _ = parse_deps(path)
        dep_map[name] = {d for d in deps if d in files}

    out: list[str] = []
    no_deps = [k for k, v in dep_map.items() if not v]

    while no_deps:
        no_deps.sort()
        n = no_deps.pop(0)
        out.append(n)
        for k in list(dep_map.keys()):
            if n in dep_map[k]:
                dep_map[k].remove(n)
                if not dep_map[k] and k not in out and k not in no_deps:
                    no_deps.append(k)

    if len(out) != len(files):
        # fallback deterministico
        remaining = [k for k in files.keys() if k not in out]
        out.extend(sorted(remaining))
    return out


def main() -> None:
    files = collect_files()
    order = topo_sort(files)

    includes: list[str] = []
    modules: list[tuple[str, list[str]]] = []

    for name in order:
        deps, sys_inc, body = parse_deps(files[name])
        for inc in sys_inc:
            if inc not in includes:
                includes.append(inc)
        modules.append((str(files[name].relative_to(ROOT)), body))

    if "#include <Arduino.h>" in includes:
        includes.remove("#include <Arduino.h>")
    includes.insert(0, "#include <Arduino.h>")

    output: list[str] = [
        "// AUTO-GERADO por tools/make_wokwi_sketch_parse.py",
        "// Estrategia: parser de includes locais + ordenacao topologica",
    ]
    output.extend(includes)
    output.append("")

    for mod_name, body in modules:
        if mod_name.startswith("tests/"):
            continue
        output.append(f"// ===== BEGIN MODULE: {mod_name} =====")
        output.extend(body)
        output.append(f"// ===== END MODULE: {mod_name} =====")
        output.append("")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text("\n".join(output).rstrip() + "\n")
    print(f"Generated: {OUT}")


if __name__ == "__main__":
    main()
