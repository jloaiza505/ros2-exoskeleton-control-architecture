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
- Calibration status: `/exo/system/calibration_status`
- User message: `/exo/system/user_message`
- Mode service: `/exo/system/set_mode`
- Start unit service: `/exo/system/start_unit`
- Shutdown unit service: `/exo/system/shutdown_unit`
- Start simulation service: `/exo/system/start_simulation`
- Stop simulation service: `/exo/system/stop_simulation`
- Full calibration service: `/exo/system/run_full_calibration`
- Calibration validation service: `/exo/system/validate_calibration`

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

### 3.7 `exo_interfaces/msg/CalibrationStatus.msg`

Fields:

- `std_msgs/Header header`
- `uint8 UNKNOWN=0`
- `uint8 MISSING=1`
- `uint8 VALID=2`
- `uint8 INVALID=3`
- `uint8 status`
- `string detail`

Contract:

- Published by calibration workflow/status owner.
- `status=VALID` is required to allow transition to `ASSISTIVE`.
- `detail` provides operator-facing reason when not valid.

### 3.8 `exo_interfaces/msg/UserMessage.msg`

Fields:

- `std_msgs/Header header`
- `uint8 INFO=0`
- `uint8 WARNING=1`
- `uint8 ERROR=2`
- `uint8 severity`
- `string code`
- `string text`

Contract:

- Published by supervisory/workflow nodes for operator-facing state.
- Intended as the GUI-facing message channel for readiness, calibration requirements, and actionable warnings.

## 4. Services

### 4.1 `exo_interfaces/srv/SetOperatingMode.srv`

Request:

- `string mode`

Response:

- `bool accepted`
- `string message`

Contract:

- Used by supervisory logic or operator tooling.
- Expected requested mode values: `OFFLINE`, `STARTUP`, `IDLE`, `CALIBRATION`, `ASSISTIVE`, `FAULT`.
- `OFFLINE` and `STARTUP` are used by startup/shutdown orchestration flow.

### 4.2 `/exo/system/run_full_calibration` (`std_srvs/srv/Trigger`)

Request:

- Empty

Response:

- `bool success`
- `string message`

Contract:

- Runs full calibration against recent gait/state window.
- Expected to be called during `CALIBRATION` mode.
- On success, runs validation and updates calibration status to `VALID`.
- On successful validation, requests transition to `IDLE`.
- Stores baseline metrics for future startup validation.

### 4.3 `/exo/system/start_unit` (`std_srvs/srv/Trigger`)

Request:

- Empty

Response:

- `bool success`
- `string message`

Contract:

- Triggers startup orchestration from UI/operator command.
- Requests system mode transition to configured startup target (`STARTUP` by default).
- Publishes user-facing startup result on `/exo/system/user_message`.

### 4.4 `/exo/system/shutdown_unit` (`std_srvs/srv/Trigger`)

Request:

- Empty

Response:

- `bool success`
- `string message`

Contract:

- Triggers safe shutdown orchestration from UI/operator command.
- Requests system mode transition to `IDLE` and then configured shutdown target (`OFFLINE` by default).
- Publishes user-facing shutdown result on `/exo/system/user_message`.

### 4.5 `/exo/system/start_simulation` (`std_srvs/srv/Trigger`)

Request:

- Empty

Response:

- `bool success`
- `string message`

Contract:

- Starts simulation gait publishing to emulate user walking.
- Emits operator-facing message on `/exo/system/user_message`.

### 4.6 `/exo/system/stop_simulation` (`std_srvs/srv/Trigger`)

Request:

- Empty

Response:

- `bool success`
- `string message`

Contract:

- Stops simulation gait publishing.
- Emits operator-facing message on `/exo/system/user_message`.

### 4.7 `/exo/system/validate_calibration` (`std_srvs/srv/Trigger`)

Request:

- Empty

Response:

- `bool success`
- `string message`

Contract:

- Runs short validation against stored calibration baseline.
- Sets calibration status to `VALID` or `INVALID`/`MISSING` based on result.
- Intended for startup quick-check when a persisted baseline exists.

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
- Calibration manager writes `CalibrationStatus`.
- No package should publish another layer's contract topic.
