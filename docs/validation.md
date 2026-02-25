# Validation Protocol

## 1. Purpose

This document defines repeatable acceptance checks for the current vertical slice:

- `exo_simulation`
- `exo_state_estimator`
- `exo_gait_phase_detector`
- `exo_torque_controller`
- `exo_watchdog`
- `exo_fault_manager`

## 2. Preconditions

- Workspace built successfully.
- ROS 2 Jazzy environment sourced.
- Launch file available: `exo_bringup.launch.xml`.

Commands:

```bash
cd ~/ros2_projects/ros2-exoskeleton-control-architecture
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 3. Test Case VC-001: Baseline Bringup

Objective:

- Confirm all vertical-slice nodes start cleanly.

Steps:

1. Run:

```bash
ros2 launch exo_bringup exo_bringup.launch.xml
```

2. Verify processes started:
- `simulation_node`
- `state_estimator_node`
- `gait_phase_detector_node`
- `torque_controller_node`
- `watchdog_node`
- `fault_manager_node`

Pass criteria:

- No process exits unexpectedly.
- No startup exceptions.
- Mode sequence observed in logs/topic: `STARTUP -> IDLE`.

## 4. Test Case VC-002: Raw Sensor Rate

Objective:

- Validate simulation publishes raw sensor stream near configured rate.

Steps:

1. In a second terminal:

```bash
ros2 topic hz /exo/sensing/raw
```

Pass criteria:

- Mean rate approximately 200 Hz (expected from `exo_simulation` config).
- Typical acceptable band for this prototype: 190-210 Hz.

## 5. Test Case VC-003: Mode-Gated Torque

Objective:

- Verify torque output is gated by operating mode.

Steps:

1. Monitor torque topic:

```bash
ros2 topic echo /exo/control/torque_command
```

2. Monitor mode topic:

```bash
ros2 topic echo /exo/system/mode
```

3. Transition modes:

```bash
ros2 service call /exo/system/set_mode exo_interfaces/srv/SetOperatingMode "{mode: CALIBRATION}"
ros2 service call /exo/system/set_mode exo_interfaces/srv/SetOperatingMode "{mode: ASSISTIVE}"
```

Pass criteria:

- In `STARTUP`, `IDLE`, and `CALIBRATION`: torque command is zero.
- In `ASSISTIVE`: nonzero torque commands appear.

## 6. Test Case VC-004: Fault Escalation by Sensor Loss

Objective:

- Confirm watchdog fault causes mode transition to `FAULT` and torque suppression.

Steps:

1. While full stack is running and mode is `ASSISTIVE`, stop simulation only:

```bash
pkill -f "/exo_simulation/simulation_node"
```

2. Monitor fault and mode:

```bash
ros2 topic echo /exo/fault/event
ros2 topic echo /exo/system/mode
```

3. Monitor torque:

```bash
ros2 topic echo /exo/control/torque_command
```

Pass criteria:

- Watchdog publishes timeout fault events.
- Fault manager transitions to `FAULT`.
- Torque is forced to zero.
- `safety_ok` becomes `false` in `FAULT`.

## 7. Test Case VC-005: Fault-Injection Launch Profile (No Simulation)

Objective:

- Quickly verify supervision behavior without simulation node.

Steps:

1. Run:

```bash
ros2 launch exo_bringup exo_bringup_no_sim.launch.xml
```

2. Observe:

```bash
ros2 topic echo /exo/fault/event
ros2 topic echo /exo/system/mode
```

Pass criteria:

- Timeout faults appear due to missing upstream sensor feed.
- Mode progression is `STARTUP -> IDLE -> FAULT`.

## 8. Known Limitations (Current Phase)

- Duplicate/near-duplicate torque messages can appear because controller publishes when either phase or fused-state callback arrives.
- Watchdog checks are mode-aware and include a post-mode-change grace period; timeout values may still need tuning for slower machines.
- No lifecycle-node orchestration yet.
