# LineFollower (PlatformIO)

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
