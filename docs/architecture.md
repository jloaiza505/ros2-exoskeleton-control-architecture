# System Architecture

## 1. Purpose

This document defines the baseline architecture for the `ros2-exoskeleton-control-architecture` project.

Primary goals:

- Provide a modular ROS 2 control stack for lower-limb assistive robotics.
- Keep torque assistance human-led (no trajectory enforcement).
- Ensure deterministic behavior under faults.
- Establish clean boundaries so simulation-to-hardware transition is straightforward.

This architecture is the implementation reference for package scaffolding, interface design, and safety policy.

## 2. Scope and System Definition

### 2.1 Target Device (Initial Abstraction)

- One actuated knee joint (sagittal plane).
- One passive ankle (future extension point).
- Rigid thigh and shank segments.
- Torque-controlled knee actuator.

### 2.2 Human-Robot Interaction Model

- Human leads motion.
- Robot applies assistive torque.
- No position trajectory tracking.
- Transparent behavior when inactive.

### 2.3 Operating Modes

- `STARTUP`: initialization phase, supervision active, torque output disabled.
- `IDLE`: no torque output.
- `CALIBRATION`: sensor/offset initialization.
- `ASSISTIVE`: phase-based torque assistance enabled.
- `FAULT`: controlled shutdown, torque held at zero until manual reset.

## 3. Architectural Principles

- Single writer for torque commands.
- Single fusion authority for sensor state.
- No cyclic dependencies between layers.
- Degrade gracefully when safe; shutdown deterministically when required.
- No silent failures: every critical component has timeout and explicit response.

## 4. Layered Control Architecture

Data and control flow are strictly upstream to downstream:

1. Sensor Layer
2. State Estimation Layer
3. Gait Phase Layer
4. Adaptation Layer
5. Torque Control Layer
6. Hardware Interface Layer
7. Safety Supervision Layer (cross-cutting)

### 4.1 Sensor Layer

Responsibilities:

- Acquire raw encoder, IMU, and contact data.
- Timestamp and validate incoming samples.
- Publish canonical raw sensor topic.

Constraints:

- No filtering or fusion beyond minimal signal sanitation.

### 4.2 State Estimation Layer

Responsibilities:

- Fuse raw measurements into filtered joint state.
- Estimate velocity/acceleration.
- Estimate contact probability and optional phase progress hints.

Constraints:

- This is the only layer allowed to consume raw sensor streams directly.

### 4.3 Gait Phase Layer

Responsibilities:

- Compute discrete phase (`STANCE`, `SWING`, `TRANSITION`).
- Compute continuous phase variable (`0.0` to `1.0`).
- Publish confidence metrics.

Constraints:

- No torque generation logic.

### 4.4 Adaptation Layer

Responsibilities:

- Tune assistance behavior at low rate.
- Publish parameter modifiers (gain scale, profile scale, timing offsets, safety scale).

Constraints:

- Must not command torque directly.
- On failure, parameters freeze at last safe value.

### 4.5 Torque Control Layer

Responsibilities:

- Compute desired torque from phase, state, and adaptive parameters.
- Enforce software limits and saturation logic.
- Publish the only torque command in the system.
- Gate output by operating mode:
  - `OFFLINE` / `STARTUP` / `IDLE` / `CALIBRATION`: zero torque.
  - `ASSISTIVE`: active assistive torque.
  - `FAULT`: zero torque with safety flag indicating fault state.

Control relation:

`torque = f(gait_phase, limb_state, adaptive_parameters)`

### 4.6 Hardware Interface Layer

Responsibilities:

- Convert torque command into actuator-specific command path.
- Read back measured torque and driver status/fault flags.
- Expose hardware state to diagnostics and fault logic.

### 4.7 Safety Supervision Layer (Cross-Cutting)

Components:

- Watchdog (topic freshness, liveliness, jitter, lifecycle state checks).
- Fault manager (criticality-aware escalation and deterministic shutdown).

Current implementation notes:

- Fault manager starts in `STARTUP`, then auto-transitions to `IDLE`.
- Watchdog is mode-aware:
  - Monitoring is suppressed in `STARTUP`.
  - A grace window is applied after mode transitions.
  - Torque-command timeout checks are enforced in `ASSISTIVE`.

## 5. Data Interfaces (Logical)

### 5.1 Raw Sensor Data Topic

Fields include:

- Joint angle.
- IMU orientation.
- IMU angular velocity.
- Contact state.
- Timestamp.

### 5.2 Fused Limb State Topic

Fields include:

- Filtered joint angle.
- Filtered joint velocity.
- Estimated acceleration.
- Contact probability.
- Optional phase-progress estimate.

### 5.3 Gait Phase Topic

Fields include:

- Discrete phase label.
- Continuous phase variable (`0..1`).
- Confidence metric.

### 5.4 Adaptive Parameter Topic

Fields include:

- Assistance gain.
- Torque profile scaling.
- Timing offsets.
- Safety scaling factor.

### 5.5 Torque Command Topic

Fields include:

- Desired joint torque.
- Saturation status.
- Safety flag.

Rule:

- This is the only command path to hardware actuation.

## 6. Timing Targets

| Layer | Target Frequency | Purpose |
| --- | --- | --- |
| Sensing | 500 Hz | Raw acquisition |
| State Estimation | 200-500 Hz | Filtered state generation |
| Gait Phase | 100-200 Hz | Phase detection |
| Torque Control | 500 Hz | Real-time actuation |
| Adaptation | 1-10 Hz | Parameter tuning |

Sensor-specific targets:

- Encoder: `>= 500 Hz`.
- IMU: `200-500 Hz`.
- Contact sensor: `>= 200 Hz`.

These targets drive executor model, callback grouping, and QoS/deadline selection.

## 7. Fault Model and Robustness

### 7.1 Failure Types

- Sensor dropout.
- Delayed messages.
- Node restart/crash.
- Actuator stall.

### 7.2 Criticality Levels

- Critical: encoder, torque controller, hardware interface.
- Medium: IMU, gait detector.
- Low: adaptation and logging.

### 7.3 Escalation Policy

1. Warning.
2. Degraded mode.
3. Torque reduction.
4. Controlled shutdown.
5. Hardware E-stop (if required).

### 7.4 Controlled Shutdown Behavior

Triggered by critical failures such as encoder loss, torque control failure, persistent stall, or watchdog timeout.

Behavior:

- Ramp torque to zero within safe rate.
- Disable controller output.
- Transition to `FAULT` mode.
- Log failure state.
- Require manual reset and recalibration.

### 7.5 Graceful Degradation Examples

- IMU failure: switch to encoder-only estimation and reduce torque gain.
- Contact sensor failure: fallback to kinematic phase cues.
- Adaptation failure: freeze adaptive parameters.

Principle:

- Remove intelligence before removing torque, when safety permits.

## 8. Safety Constraints

Hard constraints:

- Maximum torque limit.
- Maximum torque rate limit.
- Immediate torque disable on critical fault.
- Watchdog timeout enforcement for critical topics/components.

Global rules:

1. No uncontrolled torque.
2. No silent failure.
3. No cyclic dependencies.
4. Every critical topic has timeout.
5. Every failure has deterministic response.
6. Degradation preferred over shutdown when safe.
7. Hardware limits override software decisions.

## 9. Simulation-First Strategy

Initial implementation targets simulation-only validation with:

- Limb kinematics.
- Contact state generation.
- Configurable gait patterns.
- Fault injection scenarios (dropout, delay, stall proxies).

Hardware integration starts only after end-to-end stack behavior is validated.

Planned enhancement:

- Add file-driven simulation mode in `exo_simulation` to replay OpenSim gait datasets (`.mot` / `.sto`) and publish realistic `RawSensorData` while preserving current message contracts.

## 10. Single Source of Truth Strategy

To keep implementation professional and consistent:

- Interface source of truth: one interfaces package for all custom message definitions.
- Behavior source of truth: centralized configuration package for limits, timing, timeouts, and policies.
- Topic/QoS source of truth: central topic names and QoS profiles used by all nodes.
- State-machine source of truth: one fault/mode policy definition consumed by runtime nodes.

Implementation rule:

- Avoid hardcoded constants in node logic; use declared parameters sourced from shared configuration files.

## 11. Planned Package Mapping (Scaffolding Reference)

Planned ROS 2 packages aligned to this architecture:

- `exo_interfaces`
- `exo_system_config`
- `exo_sensor_layer`
- `exo_state_estimator`
- `exo_gait_phase_detector`
- `exo_adaptation`
- `exo_torque_controller`
- `exo_hardware_interface`
- `exo_watchdog`
- `exo_fault_manager`
- `exo_simulation`
- `exo_bringup`
- `exo_utils`

This mapping will be used in the scaffolding phase after architecture documentation is approved.

## 12. Out of Scope for This Version

- Multi-joint active assistance beyond knee joint.
- Human intent prediction beyond phase/state-based adaptation.
- Autonomous gait trajectory generation.
- Automatic return from `FAULT` to `ASSISTIVE`.

---

Status: Draft v1 (baseline architecture for workspace scaffolding)
