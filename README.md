# Semantic Companion Robot

A ROS 2 wheeled mobile robot that combines LiDAR-based autonomous navigation with iPhone-based semantic perception.

> Status: **simulation phase** — validating the SLAM → Nav2 pipeline in Gazebo before hardware bring-up.

## Hardware (planned)

| Part | Role |
|---|---|
| Waveshare UGV02 | Chassis, motors, built-in ESP32 (encoders, IMU, closed-loop control) |
| Raspberry Pi 5 8GB | Main computer — Ubuntu 24.04, ROS 2 Jazzy |
| RPLIDAR C1 | 2D 360° LiDAR → `/scan` |
| iPhone | Camera, ARKit, CoreML perception |

## Software stack

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · SLAM · Nav2

## Repository layout

```text
src/        ROS 2 packages written for this project
scripts/    Dev helper scripts (start sim, SLAM, teleop, ...)
docs/       Architecture notes and debugging records
logs/       Local runtime logs (not committed)
```

## Roadmap

1. Validate SLAM map quality in simulation
2. Nav2 autonomous navigation end-to-end in simulation
3. Hardware bring-up (UGV02 + Pi 5 + RPLIDAR C1)
4. Real-world SLAM / Nav2
5. iPhone semantic perception; camera-based distance estimation vs LiDAR ground truth
