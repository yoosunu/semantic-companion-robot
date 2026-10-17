# scr_ugv02 — ROS 2 ↔ UGV02 bridge

Translator between ROS topics and the UGV02's ESP32 (JSON lines over serial). Without this node nothing
in ROS can move the real robot. In Gazebo `ros_gz_bridge` does this job; on the real robot it is ours.
Protocol: `docs/ugv02-protocol.md`. Overall plan: `docs/roadmap.md`.

```
teleop / Nav2 ─ /cmd_vel ─> ugv02_bridge ─ JSON ─> serial ─> UGV02 ESP32   (real)
                                                         └─> fake_esp32 ─> Gazebo TB3   (UGV02 mode sim)
```

Ownership: serial I/O (`include/scr_ugv02/serial_port.hpp`) and the sim harness are Claude's boilerplate.
**The bridge logic (`src/ugv02_bridge.cpp`) is Jasper's.**

## Versions and pass criteria

| ver | what | pass (numbers / observable) |
|---|---|---|
| **v0** | `/cmd_vel` → `{"T":13,"X":..,"Z":..}` → serial | see checklist below |
| v1 | feedback JSON → `/odom` + TF `odom→base_link` | RViz: robot follows my /odom; 1 m forward → odom 1.00 ± 0.05 m (sim) |
| v2 | safety: no /cmd_vel for 0.5 s → send stop; send stop on shutdown; `/battery_state` | kill teleop while driving → robot stops < 0.5 s (now: ~3 s) |

## v0 checklist
1. `colcon build --packages-select scr_ugv02` → 0 warnings
2. Without the bridge, teleop does **not** move the robot (proves Gazebo only listens through fake_esp32)
3. With the bridge, forward key → robot moves; `esp32` window shows `recv {"T":13,"X":0.200,"Z":0.000}`
4. Left key → robot turns **counter-clockwise** (sign check, REP 103)
5. Wrong port (`-p port:=/tmp/nope`) → clear error message and exit, not a silent hang
6. Observe (don't fix yet): Ctrl+C the bridge while driving → how many seconds until the robot stops? (fixed in v2)

## v0 result (2026-10-14, UGV02 mode sim)
- teleop drives the TB3 only through the bridge; signs OK (left key → Z +0.9, right → Z −1.0, back → X −0.2)
- wrong port → `failed to open serial port /tmp/nope ... No such file or directory`, process exits
  (exits via uncaught exception "terminate called ..." → clean up with try/catch in main, v2)
- bridge killed while driving (`ros2 topic pub -r 10 /cmd_vel ... x: 0.1`): last command received 1791966698.8,
  firmware stop logged 1791966705.6 → **robot kept driving ~3 s after the bridge died** (firmware 3 s timeout).
  At 0.1 m/s ≈ 0.3 m; at UGV02 max 0.5 m/s it would be ≈ 1.5 m. → motivation for v2 (stop within 0.5 s).
- Test tip: teleop can't be used for this test — focusing another terminal stops key input and safe_teleop
  sends zero after 0.5 s by itself. Use `ros2 topic pub -r 10` as a source that never stops.

## v2 result (2026-10-18, UGV02 mode sim) — written by Jasper
Dead-man rule: no /cmd_vel for `cmd_timeout` (param, 0.5 s) -> send `{"T":13,"X":0,"Z":0}` once; also on shutdown.
Source: `ros2 topic pub -r 10 /cmd_vel ... x: 0.1` (teleop can't be used: it stops itself when it loses focus).

| test | log evidence | result |
|---|---|---|
| command source killed (pub Ctrl+C) | bridge: `sent stop (cmd_vel timeout), 0.505 s after last /cmd_vel` | ✅ stop 0.505 s after last command (was ~3 s) |
| bridge Ctrl+C while driving | bridge SIGINT 1792262783.928 → fake_esp32 `recv X:0.000` 1792262783.948 | ✅ stop sent ~20 ms after Ctrl+C |
| restart after stop | pub again → `recv X:0.100` at 1792262783.012 | ✅ |
| no stop spam | one `sent stop` line per stop | ✅ |

Limit (by design): if the bridge itself is killed hard (`kill -9`, crash) it can't send anything →
only the firmware 3 s timeout remains. Defense in depth: teleop 0.5 s → bridge 0.5 s → firmware 3 s.
Note: after a stop the fake firmware still logs "no command for 3 s (already stopped)" — harmless.

## Run (UGV02 mode sim)
```bash
# [VM] once:   sudo apt install socat python3-serial
# [VM] build:  cd ~/scr_ws && colcon build --symlink-install --packages-select scr_gazebo scr_ugv02
./scripts/ugv02_sim.sh                       # tmux: gazebo / serial(socat) / esp32 windows
ros2 run scr_ugv02 ugv02_bridge              # new terminal (source scripts/env.sh first)
robot-drive                                  # new terminal, teleop
robot-stop                                   # when done
```
