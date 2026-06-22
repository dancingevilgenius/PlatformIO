# ConeStalker (PlatformIO)

PlatformIO ports of [Robots/ConeStalker](https://github.com/dancingevilgenius/Robots/tree/main/ConeStalker) sketches.

Each subfolder is a standalone PlatformIO project (`platformio.ini`, `src/main.cpp`).

Target board: **SparkFun RedBoard Turbo** (`sparkfun_redboard_turbo`, SAMD51) — matches Rev4/ConeStalker hardware with `Wire1` and `D0`–style pins.

Shared local libraries in the repo-root `libraries/` folder:

| Library | Used by |
|---------|---------|
| `Serial_Controlled_Motor_Driver` | SCMD motor driver tests |
| `SparkFun_VL53L1X_4m_Laser_Distance_Sensor` | VL53L1X distance sketches |

Open a project folder in VS Code and use PlatformIO **Build** / **Upload**.
