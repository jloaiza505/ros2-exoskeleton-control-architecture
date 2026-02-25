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

## 3. Next Steps (Recommended Order)

### N1 - Adaptation Layer Baseline

- Implement `exo_adaptation` publisher for `AdaptiveParameters`.
- Integrate adaptation input in torque controller.

Success criteria:

- Controller consumes adaptation topic.
- Parameter updates visibly change torque profile.

### N2 - Deterministic Fault Policy Expansion

- Enforce severity mapping and escalation rules in fault manager.
- Add explicit reset workflow from `FAULT -> IDLE -> CALIBRATION -> ASSISTIVE`.

Success criteria:

- Invalid mode transitions rejected.
- Fault and recovery behavior matches documented policy.

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
