#!/usr/bin/env python3
"""Convert Robots/MiniSumo Arduino sketches to PlatformIO projects under Robots/MiniSumo/."""

from __future__ import annotations

import re
import shutil
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
SOURCE = REPO.parent / "_Robots_import_temp" / "MiniSumo"
DEST = REPO / "Robots" / "MiniSumo"

GITIGNORE = """.pio
.vscode/.browse.c_cpp.db*
.vscode/c_cpp_properties.json
.vscode/launch.json
.vscode/ipch
"""

LIDAR = [
    "https://github.com/DFRobot/DFRobot_MatrixLidar.git",
    "adafruit/Adafruit IS31FL3741 Library",
    "adafruit/Adafruit BusIO",
]

ESP32_WEB = [
    "bblanchon/ArduinoJson @ ^7.4.3",
    "https://github.com/ESP32Async/AsyncTCP.git#v3.4.10",
    "https://github.com/ESP32Async/ESPAsyncWebServer.git#v3.11.0",
]

SCMD = []  # vendored: libraries/Serial_Controlled_Motor_Driver (custom fork)

MUX_MOTOR = [
    "adafruit/Adafruit NeoPixel @ ^1.12.0",
    "pololu/QTRSensors @ ^3.0.0",
    "sparkfun/SparkFun VL53L1X VL53L4 CX VL53L3 CX Distance Sensor @ ^1.2.5",
    "sparkfun/SparkFun I2C Mux Arduino Library @ ^1.0.3",
]

VL53L5_AN = [
    "sparkfun/SparkFun VL53L5CX Library @ ^1.0.3",
    "sparkfun/SparkFun Qwiic Alphanumeric Display @ ^1.0.3",
]

PROJECTS: dict[str, dict] = {
    "ESP32_QT_PY_DFR8x8_LedMatrix": {
        "env": "adafruit_qtpy_esp32",
        "platform": "espressif32",
        "board": "adafruit_qtpy_esp32",
        "lib_deps": LIDAR,
    },
    "MiniSumo_ESP32_C3_8x8_LedMatrix_Web_SCMD": {
        "env": "esp32-c3-minisumo-web",
        "platform": "espressif32",
        "board": "esp32-c3-devkitm-1",
        "lib_deps": ESP32_WEB
        + LIDAR
        + SCMD
        + ["adafruit/Adafruit NeoPixel @ ^1.12.0"],
        "build_flags": [
            "-DARDUINO_USB_MODE=1",
            "-DARDUINO_USB_CDC_ON_BOOT=1",
        ],
        "network_api_v2": True,
    },
    "MiniSumo_LonelyBinary_S3_DFR8x8_LedMatrix": {
        "env": "esp32-s3-lonelybinary",
        "platform": "espressif32",
        "board": "esp32-s3-devkitc-1",
        "lib_deps": LIDAR,
    },
    "MiniSumo_Metro_S3_8x8_SCMD_BLE": {
        "env": "adafruit-metro-esp32s3",
        "platform": "espressif32",
        "board": "adafruit_metro_esp32s3",
        "lib_deps": LIDAR + SCMD + VL53L5_AN[1:],
    },
    "MiniSumo_Propmaker_MUX": {
        "env": "adafruit-feather-prop-maker",
        "platform": "https://github.com/maxgerhardt/platform-raspberrypi.git",
        "board": "adafruit_feather_prop_maker",
        "lib_deps": MUX_MOTOR,
    },
    "MiniSumo_QT_PY_8x8_AN_SCMD": {
        "env": "adafruit_qtpy_esp32s3",
        "platform": "espressif32",
        "board": "adafruit_qtpy_esp32s3_n4r2",
        "lib_deps": VL53L5_AN + SCMD,
    },
    "MiniSumo_QT_PY_8x8_LedMatrix_PS3": {
        "env": "adafruit_qtpy_esp32",
        "platform": "espressif32",
        "board": "adafruit_qtpy_esp32",
        "lib_deps": LIDAR[:2] + VL53L5_AN[:1],
    },
    "MiniSumo_QT_PY_Pico_8x8_13x9_SCMD": {
        "env": "adafruit_qtpy_esp32pico",
        "platform": "espressif32",
        "board": "adafruit_qtpy_esp32",
        "lib_deps": ESP32_WEB + LIDAR + SCMD,
        "board_build": {
            "flash_size": "8MB",
            "psram": "enabled",
            "filesystem": "littlefs",
            "partitions": "default_8MB.csv",
        },
        "has_data": True,
    },
    "MiniSumo_QT_PY_Pico_DFR8x8_LedMatrix": {
        "env": "adafruit_qtpy_esp32pico",
        "platform": "espressif32",
        "board": "adafruit_qtpy_esp32",
        "lib_deps": ESP32_WEB + LIDAR,
        "board_build": {
            "flash_size": "8MB",
            "psram": "enabled",
            "filesystem": "littlefs",
            "partitions": "default_8MB.csv",
        },
        "has_data": True,
    },
    "MiniSumo_QT_PY_S3_DFR8x8_LedMatrix": {
        "env": "adafruit_qtpy_esp32s3",
        "platform": "espressif32",
        "board": "adafruit_qtpy_esp32s3_n4r2",
        "lib_deps": LIDAR,
    },
    "MiniSumo_RP2040_8x8_SCMD_BLE": {
        "env": "sparkfun-thing-plus-rp2040",
        "platform": "https://github.com/maxgerhardt/platform-raspberrypi.git",
        "board": "sparkfun_thingplusrp2040",
        "lib_deps": LIDAR + SCMD + [VL53L5_AN[1]],
    },
    "MiniSumo_ThingPlus_RP2040_DFR8x8_LedMatrix": {
        "env": "sparkfun-thing-plus-rp2040",
        "platform": "https://github.com/maxgerhardt/platform-raspberrypi.git",
        "board": "sparkfun_thingplusrp2040",
        "lib_deps": LIDAR,
    },
    "MiniSumo_Thing_Plus_RP2040_MUX": {
        "env": "sparkfun-thing-plus-rp2040",
        "platform": "https://github.com/maxgerhardt/platform-raspberrypi.git",
        "board": "sparkfun_thingplusrp2040",
        "lib_deps": MUX_MOTOR,
    },
    "RCSumo_ESP32_PS3": {
        "env": "adafruit-feather-esp32",
        "platform": "espressif32",
        "board": "adafruit_feather_esp32_v2",
        "lib_deps": MUX_MOTOR,
    },
}


def normalize_main_cpp(content: str, network_api_v2: bool) -> str:
    if network_api_v2:
        content = content.replace("NetworkClient", "WiFiClient")
        content = content.replace("NetworkServer", "WiFiServer")
    if "#include <Arduino.h>" not in content and '#include "Arduino.h"' not in content:
        content = "#include <Arduino.h>\n\n" + content
    return content


def write_platformio_ini(project_dir: Path, name: str, cfg: dict) -> None:
    env = cfg["env"]
    lines = [
        f"; {name}",
        f"; Source: Robots/MiniSumo/{name}",
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

    board_build = cfg.get("board_build", {})
    for key, value in board_build.items():
        lines.append(f"board_build.{key} = {value}")

    if cfg.get("has_data") and "filesystem" not in board_build:
        lines.append("board_build.filesystem = littlefs")

    lib_deps = cfg.get("lib_deps", [])
    if lib_deps:
        lines.append("")
        lines.append("lib_deps =")
        for dep in lib_deps:
            lines.append(f"    {dep}")

    build_flags = cfg.get("build_flags", [])
    if build_flags or cfg.get("has_data"):
        lines.append("")
        lines.append("build_flags =")
        lines.append("    -DCORE_DEBUG_LEVEL=0")
        for flag in build_flags:
            lines.append(f"    {flag}")

    project_dir.joinpath("platformio.ini").write_text("\n".join(lines) + "\n", encoding="utf-8")


def convert_project(name: str, cfg: dict) -> None:
    src_dir = SOURCE / name
    dst_dir = DEST / name
    if not src_dir.is_dir():
        raise FileNotFoundError(f"Missing source folder: {src_dir}")

    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    dst_dir.mkdir(parents=True)

    ino_files = list(src_dir.glob("*.ino"))
    if not ino_files:
        raise FileNotFoundError(f"No .ino in {src_dir}")
    ino = ino_files[0].read_text(encoding="utf-8", errors="replace")
    main_cpp = normalize_main_cpp(ino, cfg.get("network_api_v2", False))
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
    if not SOURCE.is_dir():
        raise SystemExit(f"Clone Robots repo to {SOURCE.parent} first.")

    DEST.mkdir(parents=True, exist_ok=True)
    for name, cfg in PROJECTS.items():
        convert_project(name, cfg)
        print(f"Converted {name}")

    readme = DEST / "README.md"
    readme.write_text(
        """# MiniSumo (PlatformIO)

PlatformIO ports of [Robots/MiniSumo](https://github.com/dancingevilgenius/Robots/tree/main/MiniSumo) sketches.

Each subfolder is a standalone PlatformIO project (`platformio.ini`, `src/main.cpp`).

Shared local libraries used by some projects live in the repo-root `libraries/` folder (`FS_MX1508`, `MedianFilterLib2`).

Open a project folder in VS Code and use PlatformIO **Build** / **Upload**.
""",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
