# Interface Contracts

## 1. Purpose

This document defines message and service contracts for the baseline vertical slice:

- `exo_simulation`
- `exo_state_estimator`
- `exo_gait_phase_detector`
- `exo_torque_controller`
- `exo_watchdog`

These contracts are the authoritative integration surface across packages.

## 2. Topic Naming

All package code should use topic names from `exo_system_config/config/topics.yaml`.

Canonical names:

- Raw sensor: `/exo/sensing/raw`
- Fused state: `/exo/state/fused`
- Gait phase: `/exo/gait/phase`
- Adaptive parameters: `/exo/adaptation/parameters`
- Torque command: `/exo/control/torque_command`
- Fault event: `/exo/fault/event`
- Mode state: `/exo/system/mode`
- Mode service: `/exo/system/set_mode`

## 3. Messages

### 3.1 `exo_interfaces/msg/RawSensorData.msg`

Fields:

- `std_msgs/Header header`
- `float64 joint_angle_rad`
- `float64 imu_orientation_pitch_rad`
- `float64 imu_angular_velocity_y_rad_s`
- `bool contact_state`

Contract:

- Published by sensor/simulation layer.
- Units are SI (rad, rad/s).
- `header.stamp` must represent measurement time.

### 3.2 `exo_interfaces/msg/FusedLimbState.msg`

Fields:

- `std_msgs/Header header`
- `float64 joint_angle_rad`
- `float64 joint_velocity_rad_s`
- `float64 joint_acceleration_rad_s2`
- `float64 contact_probability`
- `float64 phase_progress_hint`

Contract:

- Published only by state estimator.
- `contact_probability` in `[0.0, 1.0]`.
- `phase_progress_hint` in `[0.0, 1.0]` when available.

### 3.3 `exo_interfaces/msg/GaitPhase.msg`

Fields:

- `std_msgs/Header header`
- `uint8 STANCE=0`
- `uint8 SWING=1`
- `uint8 TRANSITION=2`
- `uint8 phase_label`
- `float64 phase_continuous`
- `float64 confidence`

Contract:

- Published only by gait phase detector.
- `phase_continuous` in `[0.0, 1.0]`.
- `confidence` in `[0.0, 1.0]`.

### 3.4 `exo_interfaces/msg/AdaptiveParameters.msg`

Fields:

- `std_msgs/Header header`
- `float64 assistance_gain`
- `float64 torque_profile_scale`
- `float64 timing_offset_s`
- `float64 safety_scale`

Contract:

- Produced by adaptation layer.
- Consumed by torque controller.
- Safety scaling must be non-negative.

### 3.5 `exo_interfaces/msg/TorqueCommand.msg`

Fields:

- `std_msgs/Header header`
- `float64 desired_torque_nm`
- `bool saturated`
- `bool safety_ok`

Contract:

- Published only by torque controller.
- This is the single software command path to actuator interface.
- `safety_ok=false` implies downstream hardware interface must hold zero torque.
- Before mode sync (`OFFLINE`) and in non-assistive modes, controller publishes safe zero torque.

### 3.6 `exo_interfaces/msg/FaultEvent.msg`

Fields:

- `std_msgs/Header header`
- `string source_node`
- `string failure_type`
- `uint8 severity`
- `string detail`

Contract:

- Emitted by watchdog and fault manager.
- `severity` follows project-level criticality mapping.

## 4. Services

### 4.1 `exo_interfaces/srv/SetOperatingMode.srv`

Request:

- `string mode`

Response:

- `bool accepted`
- `string message`

Contract:

- Used by supervisory logic or operator tooling.
- Expected requested mode values: `IDLE`, `CALIBRATION`, `ASSISTIVE`, `FAULT`.
- `STARTUP` is an internal mode set by fault manager on bringup and is not accepted through the service.

## 5. QoS and Timing (Baseline)

- Sensor/fused/phase streams: keep-last depth 10, volatile durability.
- Torque and fault streams: reliable delivery preferred.
- Watchdog freshness check compares now vs latest message stamp or arrival time.
- Mode topic uses transient-local QoS so late subscribers receive latest mode.

## 6. Ownership Rules

- Only state estimator writes `FusedLimbState`.
- Only gait detector writes `GaitPhase`.
- Only torque controller writes `TorqueCommand`.
- Watchdog/fault manager write `FaultEvent`.
- No package should publish another layer's contract topic.
