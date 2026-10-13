# scr_motion

Basic motion nodes. They only use `/cmd_vel` (TwistStamped) and `/odom`, so the same code runs in Gazebo and on the real UGV02.

## simple_mover
```bash
ros2 run scr_motion simple_mover --ros-args -p use_sim_time:=true     # in sim
./scripts/move.sh forward 1.0      # or: left 90 / right 45 / backward 0.5 / stop
```

### Baseline → upgrade targets (Jasper)
| # | Baseline behaviour | Upgrade | How to measure |
|---|---|---|---|
| 1 | Constant speed, overshoots the target | Slow down near the target (P control) | final error vs. target, e.g. 10 runs of `forward 1.0` |
| 2 | Ignores obstacles | Stop if `/scan` shows something closer than X m ahead | drive toward a pillar |
| 3 | One command at a time | Command queue (e.g. a square: forward/left × 4) | end-pose error after the square |
| 4 | Speed jumps 0 → full | Acceleration limit | smoothness of `/cmd_vel` |
| 5 | Topic-based command, no feedback | ROS 2 action (goal / feedback / result) | — |
