# platformio-projects

PlatformIO firmware projects for robots, examples, tutorials, and Arduino ports.

This repository is **not** the [PlatformIO](https://platformio.org) toolchain — it is a collection of projects that *use* PlatformIO.

## Layout

- `Robots/` — robot sketches (MiniSumo, LineFollower, ConeStalker, …)
- `Examples/` — reference and demo projects
- `Arduino/` — Arduino-style project ports
- `libraries/` — shared vendored libraries (see `libraries/README.md`)
- `platformio.shared.ini` — shared `[common]` config (`lib_extra_dirs`, monitor, etc.)
- `tools/` — conversion scripts from Arduino sketch repos

## Open a project

Each project folder has its own `platformio.ini`. Open that folder in VS Code/Cursor and use PlatformIO **Build** / **Upload**.

## Clone

```bash
git clone https://github.com/dancingevilgenius/platformio-projects.git
```
