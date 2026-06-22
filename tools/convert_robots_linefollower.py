#!/usr/bin/env python3
"""Convert Robots/LineFollower Arduino sketches to PlatformIO projects under Robots/LineFollower/."""

from __future__ import annotations

import argparse
import re
import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = Path(r"D:\Workspaces\Arduino\Robots\LineFollower")
DEST = REPO / "Robots" / "LineFollower"

RP2040_PLATFORM = "https://github.com/maxgerhardt/platform-raspberrypi.git"

PS3_PLATFORM_PACKAGES = [
    "platform_packages =",
    "    platformio/framework-arduinoespressif32 @ https://github.com/espressif/arduino-esp32.git#3.2.0",
    "board_build.arduino.upstream_packages = no",
]

GITIGNORE = """.pio
.vscode/.browse.c_cpp.db*
.vscode/c_cpp_properties.json
.vscode/launch.json
.vscode/ipch
"""

TCS34725 = ["https://github.com/adafruit/Adafruit_BusIO.git"]  # TCS34725 vendored in libraries/

PROJECTS: dict[str, dict] = {
    "LineFollower_ESP32_OSOYOO": {
        "env": "esp32dev",
        "platform": "espressif32",
        "board": "esp32dev",
        "lib_deps": [],
    },
    "DragRacer_ESP32_OSOYOO": {
        "env": "esp32dev",
        "platform": "espressif32",
        "board": "esp32dev",
        "lib_deps": [],
        "ps3": True,
    },
    "DragRacer_ThingPlus_RP2040_TCS34725": {
        "env": "sparkfun-thing-plus-rp2040",
        "platform": RP2040_PLATFORM,
        "board": "sparkfun_thingplusrp2040",
        "lib_deps": TCS34725,
    },
    "DragRacer_ThingPlus_RP2040_OSOYOO": {
        "env": "sparkfun-thing-plus-rp2040",
        "platform": RP2040_PLATFORM,
        "board": "sparkfun_thingplusrp2040",
        "lib_deps": [],
    },
    "DragRacer_ThingPlus_ProMicroRP2040_HUSKYLENS_SCMD": {
        "env": "sparkfun-promicro-rp2040",
        "platform": RP2040_PLATFORM,
        "board": "sparkfun_promicrorp2040",
        "lib_deps": ["https://github.com/adafruit/Adafruit_NeoPixel.git"],
    },
    "DragRacer_Inland_OSOYOO": {
        "env": "esp32dev",
        "platform": "espressif32",
        "board": "esp32dev",
        "lib_deps": [],
    },
    "CupGrabber_Acebott_OSOYOO": {
        "env": "esp32dev",
        "platform": "espressif32",
        "board": "esp32dev",
        "lib_deps": [],
        "ps3": True,
    },
    "CupGrabber_Acebott_I2C5Line": {
        "env": "esp32dev",
        "platform": "espressif32",
        "board": "esp32dev",
        "lib_deps": [],
    },
}

FUNC_DEF_RE = re.compile(
    r"^((?:void|bool|boolean|int|long|unsigned long|float|double|uint8_t|uint16_t|uint32_t|String)\s+\w+\s*\([^;{}]*\))\s*\{",
    re.MULTILINE,
)


def strip_anonymous_namespaces(content: str) -> str:
    lines: list[str] = []
    in_namespace = False
    depth = 0
    for line in content.splitlines():
        if not in_namespace and re.match(r"^\s*namespace\s*\{", line):
            in_namespace = True
            depth = line.count("{") - line.count("}")
            continue
        if in_namespace:
            depth += line.count("{") - line.count("}")
            if depth <= 0:
                in_namespace = False
            continue
        lines.append(line)
    return "\n".join(lines)


def add_forward_declarations(content: str) -> str:
    scan_content = strip_anonymous_namespaces(content)
    prototypes: list[str] = []
    seen: set[str] = set()
    for match in FUNC_DEF_RE.finditer(scan_content):
        signature = match.group(1).strip()
        name_match = re.search(r"\s+(\w+)\s*\(", signature)
        if not name_match:
            continue
        name = name_match.group(1)
        if name in {"setup", "loop"}:
            continue
        if name in seen:
            continue
        if re.search(rf"^\s*{re.escape(signature)}\s*;", content, re.MULTILINE):
            continue
        seen.add(name)
        prototypes.append(f"{signature};")

    if not prototypes:
        return content

    block = "\n".join(prototypes) + "\n\n"
    lines = content.splitlines(keepends=True)
    insert_at = 0
    in_block_comment = False
    for index, line in enumerate(lines):
        stripped = line.lstrip()
        if in_block_comment:
            if "*/" in stripped:
                in_block_comment = False
            continue
        if stripped.startswith("/*"):
            in_block_comment = "*/" not in stripped
            continue
        if stripped.startswith("#include"):
            insert_at = index + 1

    lines.insert(insert_at, block)
    return "".join(lines)


def normalize_main_cpp(content: str) -> str:
    if "#include <Arduino.h>" not in content and '#include "Arduino.h"' not in content:
        content = "#include <Arduino.h>\n\n" + content
    return add_forward_declarations(content)


def write_platformio_ini(project_dir: Path, name: str, cfg: dict) -> None:
    env = cfg["env"]
    lines = [
        f"; {name}",
        f"; Source: Robots/LineFollower/{name}",
        "; https://github.com/dancingevilgenius/Robots.git",
        "",
        "[platformio]",
        "extra_configs = ../../../platformio.shared.ini",
        f"default_envs = {env}",
        "",
        f"[env:{env}]",
        "extends = common",
        f"platform = {cfg['platform']}",
        f"board = {cfg['board']}",
        "framework = arduino",
    ]

    if cfg["platform"] == "espressif32":
        lines.append("upload_speed = 921600")
        if cfg.get("ps3"):
            lines.append("")
            lines.extend(PS3_PLATFORM_PACKAGES)

    lib_deps = cfg.get("lib_deps", [])
    if lib_deps:
        lines.append("")
        lines.append("lib_deps =")
        for dep in lib_deps:
            lines.append(f"    {dep}")

    project_dir.joinpath("platformio.ini").write_text("\n".join(lines) + "\n", encoding="utf-8")


def remove_project_dir(dst_dir: Path) -> None:
    if not dst_dir.exists():
        return
    for child in dst_dir.iterdir():
        if child.name == ".pio":
            continue
        if child.is_dir():
            shutil.rmtree(child, ignore_errors=True)
        else:
            child.unlink(missing_ok=True)


def convert_project(source: Path, name: str, cfg: dict) -> None:
    src_dir = source / name
    dst_dir = DEST / name
    if not src_dir.is_dir():
        raise FileNotFoundError(f"Missing source folder: {src_dir}")

    dst_dir.mkdir(parents=True, exist_ok=True)
    remove_project_dir(dst_dir)

    ino_files = list(src_dir.glob("*.ino"))
    if not ino_files:
        raise FileNotFoundError(f"No .ino in {src_dir}")
    ino = ino_files[0].read_text(encoding="utf-8", errors="replace")
    main_cpp = normalize_main_cpp(ino)
    (dst_dir / "src").mkdir()
    (dst_dir / "src" / "main.cpp").write_text(main_cpp, encoding="utf-8")

    data_src = src_dir / "data"
    if data_src.is_dir():
        shutil.copytree(data_src, dst_dir / "data")

    for extra in src_dir.glob("*.txt"):
        shutil.copy2(extra, dst_dir / extra.name)

    write_platformio_ini(dst_dir, name, cfg)
    (dst_dir / ".gitignore").write_text(GITIGNORE, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=DEFAULT_SOURCE,
        help=f"LineFollower Arduino folder (default: {DEFAULT_SOURCE})",
    )
    args = parser.parse_args()
    source = args.source.resolve()
    if not source.is_dir():
        raise SystemExit(f"Source folder not found: {source}")

    DEST.mkdir(parents=True, exist_ok=True)
    for name, cfg in PROJECTS.items():
        convert_project(source, name, cfg)
        print(f"Converted {name}")

    readme = DEST / "README.md"
    readme.write_text(
        """# LineFollower (PlatformIO)

PlatformIO ports of [Robots/LineFollower](https://github.com/dancingevilgenius/Robots/tree/main/LineFollower) sketches.

Each subfolder is a standalone PlatformIO project (`platformio.ini`, `src/main.cpp`).

Shared local libraries live in the repo-root `libraries/` folder:

| Library | Used by |
|---------|---------|
| `Fork_of_PS3_Controller_Host` | DragRacer/CupGrabber PS3 sketches |
| `emakefun_line_tracker_v3` | CupGrabber I2C five-line tracker |
| `HUSKYLENS` | ProMicro RP2040 HUSKYLENS + SCMD |
| `Serial_Controlled_Motor_Driver` | SCMD motor driver |

Open a project folder in VS Code and use PlatformIO **Build** / **Upload**.
""",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
