# UGV02 Serial Protocol (what our bridge speaks)

Source: Waveshare wiki https://www.waveshare.com/wiki/UGV02 (read 2026-10-13).
Every line is marked **[확인됨]** (stated in the wiki) or **[가정]** (not documented — must be verified on the real robot, day 1).

## Physical link
- UART on the 40-pin header (to Pi) or the driver board USB (CP2102). [확인됨]
- 115200 baud (Waveshare Python demo uses `baudrate=115200`). [확인됨]
- One JSON object per line, terminated by `\n`. [가정 — demo writes JSON + newline; verify]

## Commands we send (Pi → UGV02)

| T | Name | Example | Meaning | |
|---|---|---|---|---|
| 13 | ROS control | `{"T":13,"X":0.1,"Z":0.3}` | X = linear m/s, Z = angular rad/s. **We use this.** | [확인됨] |
| 1 | Wheel speed | `{"T":1,"L":0.5,"R":0.5}` | each wheel m/s, −0.5 … +0.5, closed loop (encoders) | [확인됨] |
| 130 | Feedback once | `{"T":130}` | request one feedback message | [확인됨, 응답 형식 미기재] |
| 131 | Feedback stream | `{"T":131,"cmd":1}` | 1 = chassis sends feedback continuously, 0 = off (default) | [확인됨] |
| 142 | Feedback interval | `{"T":142,"cmd":50}` | feedback period (probably ms) | [가정, Waveshare app 사용] |
| 143 | Serial echo | `{"T":143,"cmd":0}` | echo received commands back (default off) | [확인됨] |

Sign convention: Z > 0 = turn left (counter-clockwise) — matches ROS REP 103. [가정 — verify with a slow left turn]

## Feedback we receive (UGV02 → Pi)
Not documented in the wiki. **Best evidence so far**: Waveshare's own Raspberry Pi app
(github.com/waveshareteam/ugv_rpi, `app.py`) has this comment and reads `v` as battery voltage:
```
{'T':1001,'L':0,'R':0,'r':0,'p':0,'v': 11,'pan':0,'tilt':0}
```
and turns the stream on at boot with:
```
{"T":142,"cmd":50}    # feedback interval (probably ms)  [가정]
{"T":131,"cmd":1}     # continuous serial feedback on    [확인됨]
```
| key | our reading | status |
|---|---|---|
| T | 1001 = base feedback | [근거 있음: Waveshare app] |
| L, R | left/right wheel speed (unit unknown, probably m/s) | [가정] |
| r, p | roll, pitch from IMU (unit unknown, probably degrees) | [가정] |
| v | battery voltage, V | [근거 있음: app shows it as base_voltage] |
| pan, tilt | gimbal (we have none) | ignore |
- No yaw and no encoder counts in this sample → odom must integrate L/R wheel speeds (yaw drift expected). [가정]
- The fake ESP32 uses this format. Keep parsing in **one function** in the bridge so day-1 changes stay local.

## Safety behavior
- Firmware stops the motors if no movement command arrives for **3 s**. [확인됨]
- 3 s at 0.5 m/s = 1.5 m → too long indoors. **Our bridge stops after 0.5 s without /cmd_vel** (bridge v2).

## Day-1 checklist (real robot)
1. `{"T":131,"cmd":1}` → record 10 s of raw feedback to `docs/ugv02-feedback-sample.txt`. Update the [가정] lines above.
2. Slow left turn `{"T":13,"X":0,"Z":0.3}` → confirm counter-clockwise.
3. Stop sending → confirm stop after ~3 s.
4. Charging port spec / charge while powered on / voltage field (mission requirement). — 10-28: 12.6V 2A DC jack on the top plate; first charge "wakes" the UPS (no output before). **Powered on while charging works** (OLED + web panel up, voltage 12.46 → 12.54 V while charging). Spec: "charging and power output at the same time", 1.5 h charge. Resting voltage (charger off) and the feedback `v` field still to check.
5. **Which port**: plug UGV02 to the Pi (USB first; GPIO UART later) → `ls /dev/ttyUSB* /dev/ttyACM* /dev/ttyAMA*` before/after → that is `ugv_port` for `ros2 launch scr_bringup robot.launch.py ugv_port:=...`. Wheels in the air.
6. **Measure for the URDF** (`src/scr_description/urdf/ugv02.urdf.xacro`, lines marked ★): wheel radius, wheel width, left↔right wheel center distance, front↔rear wheel distance, LiDAR position (front/back, left/right from body center) and height above the wheel axles.
7. **LiDAR direction** (do after mounting): put a box 50 cm in front of the robot → RViz `/scan` (Fixed Frame `base_link`). Box drawn behind → set `laser_yaw` to `${pi}`.
8. **LiDAR driver**: ✅ 10-28. apt `rplidar_ros` 2.1.0 (SDK 1.12): reads S/N·health but `Cannot start scan: 80008002` → **Slamtec `sllidar_ros2`** (SDK 2.1.0, submodule `src/sllidar_ros2`): Standard mode, 5 kHz, `/scan` 10.0 Hz (std 1 ms), 720 beams (0.5°), angle −180..180°. CP210x on `/dev/ttyUSB0`. Note: the C1 outer cap does not spin (only the inside), motor sound only.
