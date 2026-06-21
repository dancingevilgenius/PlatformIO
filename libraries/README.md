# PlatformIO shared libraries

Central library folder for this repository (similar to Arduino IDE `sketchbook/libraries`).

Place libraries here when they are:

- Custom forks or local patches (not suitable for `lib_deps` as-is)
- Not available on the PlatformIO Registry
- Shared by multiple projects under `Examples/`, `Arduino/`, or `Tutorials and Books/`

## Layout

Each library is one subdirectory with a standard Arduino layout:

```
libraries/
└── MyLibrary/
    ├── library.properties
    ├── library.json          (optional; PlatformIO)
    └── src/
        └── ...
```

## Usage in a project

Include the shared config from the project `platformio.ini`, then extend the shared environment:

```ini
[platformio]
extra_configs = ../../platformio.shared.ini

[env:my_board]
extends = common
platform = espressif32
board = esp32dev
framework = arduino

; Registry libraries still go in lib_deps when possible:
; lib_deps =
;     bblanchon/ArduinoJson @ ^7.4.3
```

Adjust the `extra_configs` path for project depth:

| Project location | `extra_configs` path |
|------------------|----------------------|
| `Examples/MyProject/` | `../../platformio.shared.ini` |
| `Examples/ESP32/MyProject/` | `../../../platformio.shared.ini` |
| `Examples/RP2040/MyProject/` | `../../../platformio.shared.ini` |
| `Examples/STM32/MyProject/` | `../../../platformio.shared.ini` |
| `Examples/Rev4/MyProject/` | `../../../platformio.shared.ini` |
| `Arduino/ESP32/MyProject/` | `../../../platformio.shared.ini` |
| `Tutorials and Books/.../MyProject/` | `../../../platformio.shared.ini` |

Project-specific libraries can still live in that project's `lib/` folder.
