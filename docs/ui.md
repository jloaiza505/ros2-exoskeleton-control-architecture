# UI Specification

## 1. Purpose

This document defines the operator UI for `ros2-exoskeleton-control-architecture`.

The UI goals are:

- Control operating modes safely.
- Display real-time state for control/safety decisions.
- Visualize signals for tuning and validation.
- Provide clear fault diagnostics and event traceability.

This document is implementation-agnostic and can be used later to generate interactive elements.

## 2. UI Scope (Phase 1)

Phase 1 UI covers simulation + vertical-slice runtime and includes:

- Unit lifecycle control (`OFFLINE` -> startup checks -> `IDLE`, and shutdown back to `OFFLINE`).
- Mode control (`IDLE`, `CALIBRATION`, `ASSISTIVE`, reset from `FAULT`).
- Calibration workflow controls (full calibration + validation).
- Simulation controls (start/stop walking simulation stream).
- Real-time plots for torque, gait, and key sensor/state signals.
- System health/fault panel with watchdog and fault manager visibility.
- Operator message stream from backend (`/exo/system/user_message`).

Out of scope for Phase 1:

- User account/auth management.
- Cloud telemetry and remote fleet management.
- Full configuration authoring UI.

## 3. Data Contract for UI

### 3.1 Subscribed Topics

- `/exo/system/mode` (`std_msgs/String`)
- `/exo/system/calibration_status` (`exo_interfaces/msg/CalibrationStatus`)
- `/exo/system/user_message` (`exo_interfaces/msg/UserMessage`)
- `/exo/fault/event` (`exo_interfaces/msg/FaultEvent`)
- `/exo/control/torque_command` (`exo_interfaces/msg/TorqueCommand`)
- `/exo/gait/phase` (`exo_interfaces/msg/GaitPhase`)
- `/exo/state/fused` (`exo_interfaces/msg/FusedLimbState`)
- `/exo/sensing/raw` (`exo_interfaces/msg/RawSensorData`)

### 3.2 Called Services

- `/exo/system/set_mode` (`exo_interfaces/srv/SetOperatingMode`)
- `/exo/system/start_unit` (`std_srvs/srv/Trigger`)
- `/exo/system/shutdown_unit` (`std_srvs/srv/Trigger`)
- `/exo/system/start_simulation` (`std_srvs/srv/Trigger`)
- `/exo/system/stop_simulation` (`std_srvs/srv/Trigger`)
- `/exo/system/run_full_calibration` (`std_srvs/srv/Trigger`)
- `/exo/system/validate_calibration` (`std_srvs/srv/Trigger`)

### 3.3 Derived UI Signals

- `is_assistive_active` from mode.
- `is_faulted` from mode and latest fault event.
- `sensor_freshness_ms` from topic timestamps.
- `torque_abs_percent = abs(desired_torque_nm) / max_torque_nm`.

## 4. Information Architecture

Single-page dashboard with five regions:

1. Header / Session Bar
2. Mode & Safety Control Panel
3. Real-Time Plot Area
4. System Status Cards
5. User Message / Event Log

## 5. Panels and Behavior

### 5.1 Header / Session Bar

Shows:

- Current launch profile (`full_stack` or `no_sim`).
- ROS domain/session label.
- Elapsed session time.
- Record marker for validation runs.

Actions:

- Start/stop local recording marker (UI metadata only in Phase 1).
- Add operator note tag.

### 5.2 Mode & Safety Control Panel

Shows:

- Current mode badge (`OFFLINE`, `STARTUP`, `IDLE`, `CALIBRATION`, `ASSISTIVE`, `FAULT`).
- Last mode transition reason (if available).
- Fault state indicator (`NORMAL`, `WARNING`, `FAULT`).
- Calibration readiness (`UNKNOWN`, `MISSING`, `VALID`, `INVALID`) with detail string.

Controls:

- `Start Unit` (calls `/exo/system/start_unit`)
- `Shutdown Unit` (calls `/exo/system/shutdown_unit`)
- `Set CALIBRATION`
- `Set ASSISTIVE`
- `Set IDLE`
- `Reset to IDLE` (enabled only when current mode is `FAULT`)
- `Run Full Calibration + Validate`
- `Validate Calibration`
- `Start Simulation Walk`
- `Stop Simulation Walk`

Service behavior:

- Every mode button calls `/exo/system/set_mode`.
- Unit buttons call `/exo/system/start_unit` and `/exo/system/shutdown_unit`.
- Simulation buttons call `/exo/system/start_simulation` and `/exo/system/stop_simulation`.
- Calibration buttons call full and validate calibration trigger services.
- UI displays service result (`accepted`, `message`).
- Reject transitions in UI without changing local state.

Safety UX rules:

- `ASSISTIVE` button disabled unless current mode is `IDLE` with valid calibration.
- In `FAULT`, action buttons are locked except reset path.
- Display explicit warning before assistive activation.

### 5.3 Real-Time Plot Area

Required plots (live, scrolling window 10-30s selectable):

1. Torque plot:
- `desired_torque_nm`
- saturation marker
- `safety_ok` marker

2. Gait plot:
- `phase_continuous`
- phase label transitions (`STANCE/SWING/TRANSITION`)
- confidence

3. State plot:
- `joint_angle_rad`
- `joint_velocity_rad_s`
- `joint_acceleration_rad_s2`

4. Sensor plot:
- raw joint angle
- IMU angular velocity
- contact state (step signal)

Plot UX requirements:

- Pause/resume stream.
- Autoscale + fixed-scale modes.
- Cursor value inspection.
- Export current window snapshot (image/CSV).

### 5.4 System Status Cards

Cards:

- Node health:
  - simulation, estimator, gait detector, torque controller, watchdog, fault manager
- Topic freshness:
  - raw/fused/gait/torque age in ms
- Rates:
  - observed frequency per key topic
- Controller state:
  - current controller mode input
  - torque output enabled/disabled summary

Color semantics:

- Green: healthy / fresh.
- Amber: stale nearing timeout.
- Red: timeout/fault.

### 5.5 User Message / Event Log

Shows timestamped stream entries:

- Mode transitions
- Fault events (source, type, severity, detail)
- Service responses
- `/exo/system/user_message` entries (`INFO`, `WARNING`, `ERROR`)
- Manual operator notes

Log controls:

- Severity filter
- Topic/source filter
- Export session log

## 6. State Management Rules (UI)

Single source of truth in UI:

- Latest message cache by topic keyed on timestamp.
- Mode source is strictly `/exo/system/mode` (never inferred from button state).
- Fault state derived from mode + recent `/exo/fault/event`.

Consistency rules:

- UI action does not optimistically assume success.
- Mode changes only after topic update confirms transition.
- If topic stream missing, display stale indicator rather than silent zero.

## 7. Performance Requirements

- UI update loop target: 20-30 FPS for rendering.
- Signal ingest target: support at least 500 Hz input streams with decimation for charts.
- End-to-end mode button feedback: < 250 ms to visible result (on local network).
- No UI thread blocking from logging/export tasks.

## 8. Fail-Safe UX Requirements

- On connection loss, disable all control actions and show banner.
- On `FAULT`, highlight red global state and freeze assistive actions.
- If mode topic absent at startup, show `UNKNOWN/OFFLINE` status.
- If torque command is nonzero outside `ASSISTIVE`, show critical UI alert.

## 9. Recommended Implementation Path

Phase A (immediate):

- Dashboard prototype with mode panel + plots + event log.
- Works with current vertical-slice topics/services.

Phase B:

- Add configuration pane (read-only first).
- Add validation-run templates and pass/fail checklist overlay.

Phase C:

- Add hardware interface diagnostics panel.
- Add rosbag integration controls.

## 10. Current Scaffold Status (March 2026)

Implemented in `ui/operator-dashboard`:

- rosbridge connection indicator.
- Subscriptions:
  - `/exo/system/mode`
  - `/exo/system/calibration_status`
  - `/exo/system/user_message`
  - `/exo/gait/phase`
- Controls:
  - `start_unit`, `shutdown_unit`
  - `set_mode`
  - `start_simulation`, `stop_simulation`
  - `run_full_calibration`, `validate_calibration`
- Current gait label/value display.
- User message list for operator-visible readiness and warning/error states.

Still pending for full Phase 1:

- Real-time plots.
- Topic freshness/rate health cards.
- Log export and richer filtering.
- Assistive transition confirmation guardrail UX.

## 11. Acceptance Criteria

The UI is considered Phase 1 complete when:

1. Operator can transition modes through service calls and observe confirmed mode updates.
2. Fault injection (simulation stop) visibly triggers:
- fault events,
- mode transition to `FAULT`,
- zero torque indication.
3. Real-time plots display torque/gait/state/sensor data with stable refresh.
4. System status cards correctly show topic freshness and stale conditions.
5. Event log captures mode transitions and fault details for export.

## 12. Suggested Future Document Split

As UI grows, split into:

- `docs/ui_architecture.md` (transport, state model, dataflow)
- `docs/ui_components.md` (widget-level spec)
- `docs/ui_validation.md` (UI-specific tests)
