# ConeStalker (PlatformIO)

PlatformIO ports of [Robots/ConeStalker](https://github.com/dancingevilgenius/Robots/tree/main/ConeStalker) sketches.

Each subfolder is a standalone PlatformIO project (`platformio.ini`, `src/main.cpp`).

Target board: **Arduino Uno R4 WiFi** (`uno_r4_wifi`, Renesas RA) — Qwiic sensors on `Wire` (SDA/SCL).

Shared local libraries in the repo-root `libraries/` folder:

| Library | Used by |
|---------|---------|
| `Serial_Controlled_Motor_Driver` | SCMD motor driver tests |
| `SparkFun_VL53L1X_4m_Laser_Distance_Sensor` | VL53L1X distance sketches |
| `NewPing` | NewPing3Sensors |
| `Bas.Button` / `Bas.CallbackCaller` | BasButtonArray |
| `Adafruit_GFX_Library` / `Adafruit_SSD1306` / `Adafruit_BusIO` | ZioUltrasonicSensor |

Open a project folder in VS Code and use PlatformIO **Build** / **Upload**.
