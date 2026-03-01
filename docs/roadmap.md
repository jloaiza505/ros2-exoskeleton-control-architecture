# Project Roadmap

## 1. Current Status

This roadmap reflects the project state after architecture definition, workspace scaffolding, and first safety-critical vertical slice.

## 2. Completed Milestones

### M1 - Architecture Baseline

- Defined layered control architecture and safety rules.
- Captured scope, timing targets, and failure assumptions.

Deliverable:

- `docs/architecture.md`

### M2 - Workspace Scaffolding

- Created ROS 2 package structure for sensing, estimation, gait, control, safety, interfaces, config, simulation, and bringup.
- Established C++/XML-first package templates.

Deliverables:

- `src/exo_*` package skeletons
- package manifests and CMake setup

### M3 - Interface Contracts (Initial)

- Defined initial message/service contracts for the vertical slice.

Deliverable:

- `docs/interfaces.md`

### M4 - Vertical Slice v1 (Functional)

- Implemented end-to-end pipeline:
  - simulation -> state estimation -> gait phase -> torque control
- Added watchdog fault monitoring.
- Added fault manager mode state machine.
- Added torque gating by operating mode.
- Added explicit startup/mode visibility and safer startup behavior:
  - `STARTUP -> IDLE` auto transition in fault manager
  - mode-aware watchdog checks to avoid startup race faults
  - torque controller `OFFLINE` pre-mode safe state

Deliverables:

- Vertical slice node implementations
- mode service and mode topic
- full-stack bringup launch

### M5 - Validation Baseline

- Added repeatable validation checklist for bringup, rates, mode gating, and fault escalation.
- Added no-simulation fault-injection launch profile.

Deliverables:

- `docs/validation.md`
- `exo_bringup_no_sim.launch.xml`

### M6 - Adaptation + Calibration Baseline + Startup Orchestration

- Implemented adaptation parameter publisher and controller integration.
- Added calibration status manager with:
  - baseline persistence (load/save),
  - full calibration service,
  - quick validation service,
  - startup validation in `IDLE`.
- Added operator-facing user message topic (`/exo/system/user_message`).
- Added bringup orchestration services:
  - `/exo/system/start_unit`
  - `/exo/system/shutdown_unit`
- Added OFFLINE behavior for power-on waiting state and startup/shutdown flow.
- Added scenario launch files for valid/missing/invalid calibration startup testing.

Deliverables:

- `src/exo_adaptation/`
- `src/exo_calibration/`
- `src/exo_bringup/src/bringup_node.cpp`
- `src/exo_bringup/launch/exo_bringup_scenario_*.launch.xml`

### M7 - Operator UI Scaffold (Phase A, Initial)

- Added React + TypeScript dashboard scaffold with rosbridge integration.
- Added unit control actions (start/shutdown), mode requests, simulation control, and calibration actions.
- Added subscriptions for mode, calibration status, gait estimate, and user messages.

Deliverables:

- `ui/operator-dashboard/`
- `docs/ui.md`

## 3. Next Steps (Recommended Order)

### N1 - Per-User Adaptation Parameters

- Extend adaptation contract to include per-user and per-fit parameters:
  - body mass,
  - user height,
  - configured lever geometry,
  - optional limb-specific scaling terms.
- Define source-of-truth ownership:
  - user-entered metadata,
  - calibration-estimated parameters,
  - runtime-safe derived values consumed by control.
- Keep torque controller integration bounded to validated/safe adaptation outputs.

Success criteria:

- Parameter schema clearly separates user-entered vs calibration-estimated fields.
- Adaptation output remains stable and safe when input values are stale/missing.
- Controller behavior changes only through validated adaptation parameters.

### N1.5 - OpenSim-Driven Simulation Input

- Extend `exo_simulation` to support OpenSim gait file playback (`.mot` / `.sto`) in addition to synthetic sinusoidal generation.
- Add runtime-selectable simulator source mode:
  - `synthetic`
  - `opensim_file`
- Add interpolation-based playback at ROS publish rate.
- Map playback data into existing `RawSensorData` interface:
  - joint angle
  - derived/loaded angular velocity
  - IMU proxy signals
  - contact proxy/event stream

Success criteria:

- Simulator can load and replay a provided OpenSim gait file.
- `/exo/sensing/raw` remains contract-compatible for downstream nodes.
- Playback can loop and support time scaling.
- Demo run shows more realistic gait signal behavior than baseline sinusoid.

### N2 - Deterministic Fault Policy Expansion

- Enforce severity mapping and escalation rules in fault manager.
- Add explicit reset workflow from `FAULT -> IDLE -> CALIBRATION -> ASSISTIVE`.

Success criteria:

- Invalid mode transitions rejected.
- Fault and recovery behavior matches documented policy.

### N2.5 - Operator UI Baseline (Execution + Runtime Tuning)

- Expand current scaffold toward full Phase A dashboard from `docs/ui.md`:
  - mode/safety control panel guardrails,
  - real-time plots (torque, gait, state, sensor),
  - status/freshness cards and event log filtering/export.
- Add basic runtime configuration controls for safe, bounded parameter changes during execution (Phase B subset).
- Keep mode/fault/readiness behavior authoritative from ROS topics/services and user message channel.

Success criteria:

- UI can drive mode transitions through `/exo/system/set_mode` and show confirmed `/exo/system/mode` updates.
- During execution, operator can apply basic runtime config updates and observe effect on published control/state signals.
- Fault events and stale/offline conditions are visible in the dashboard.

### N3 - Hardware Interface Abstraction

- Implement `exo_hardware_interface` mock backend first.
- Route torque command and measured torque/driver status through interface contract.

Success criteria:

- No direct hardware assumptions in torque controller.
- Sim and mock hardware use same command/state contracts.

### N4 - Test and CI Hardening

- Add package-level unit tests.
- Add integration tests for mode transitions and watchdog behavior.
- Expand CI from placeholder to build + test workflows.

Success criteria:

- Automated checks run in CI.
- Regressions are caught before merge.

### N5 - Documentation and Portfolio Readiness

- Add `docs/safety.md`, `docs/validation_results.md`, and ADR entries.
- Add architecture and data-flow diagrams.

Success criteria:

- Project narrative is clear for recruiters/technical reviewers.
- System design decisions are traceable.

## 4. Phase Exit Criteria

### Exit for Foundation Phase

- Stable end-to-end stack in simulation.
- Mode-safe torque gating enforced.
- Fault detection and transition to `FAULT` validated.

### Exit for Integration Phase

- Adaptation integrated.
- Hardware interface abstraction validated with mock backend.
- CI + automated integration tests active.

### Exit for Portfolio Demonstration Phase

- Reproducible demo scenarios (nominal + fault).
- Clear architecture docs and validation evidence.
