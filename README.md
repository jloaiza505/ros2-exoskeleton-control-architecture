# ros2-exoskeleton-control-architecture
Modular ROS2-based human-in-the-loop control architecture integrating sensor fusion, gait phase detection, and adaptive torque generation for assistive robotics systems.

# Problem Statement

Real-world assistive robotics requires integration of sensing, state estimation, control, and user adaptation within a unified architecture. Most academic prototypes demonstrate isolated components rather than full-stack systems. This project addresses the design of a modular ROS2-based control framework for a lower-limb assistive device, integrating sensor fusion, gait phase detection, adaptive torque generation, and real-time parameter tuning. The objective is to demonstrate system-level robustness, modularity, and extensibility toward real hardware deployment.

# System Definition & Architectural Groundwork

This phase defines constraints, interfaces, assumptions, and operating targets.

## 1. System Scope Definition
### 1.1 Target Device Abstraction

The lower-limb assistive system is configured as (initial assumption):

- 1 actuated knee joint (sagittal plane)
- 1 passive ankle (optional future extension)
- Rigid thigh and shank segments
- Torque-controlled actuator at knee

### 1.2 Human–Robot Interaction Model

We assume:

- Human leads motion
- Robot provides assistive torque
- No trajectory enforcement
- Transparent when inactive

In other words, we will not be controlling position but rather modulating torque based on gait phase.

### 1.3 Operating Modes

The system-level operation modes will be:

- Startup (initialization and supervision warm-up)
- Idle (no torque output)
- Calibration
- Assistive mode
- Fault / Emergency stop

## 2. Sensor Architecture Definition

### 2.1 Required Sensors (Minimal Set)

For this projects, the following is the minimal sensor set:

- Joint encoder (position)
- Derived velocity
- Foot contact sensor (binary)
- IMU on shank

### 2.2 Sensor Sampling Targets

Sensors will have the following frequency targets:

- Encoder: ≥ 500 Hz
- IMU: 200–500 Hz
- Contact sensor: ≥ 200 Hz

These rates support 200–500 Hz control loops.

## 3. Control & Estimation Timing Targets


| Layer            | Frequency  | Purpose          |
| ---------------- | ---------- | ---------------- |
| Sensing          | 500 Hz     | Raw acquisition  |
| State Estimation | 200–500 Hz | Filtered state   |
| Gait Phase       | 100–200 Hz | Phase logic      |
| Torque Control   | 500 Hz     | Actuation        |
| Adaptation       | 1–10 Hz    | Parameter tuning |


These frequencies define QoS and executor configuration later.

## 4. Data Interfaces (High-Level)

### 4.1 Raw Sensor Topic

Contains:

- Joint angle
- IMU orientation
- IMU angular velocity
- Contact state
- Timestamp

This is unfiltered data.

### 4.2 Fused Limb State Topic

Contains:

- Joint angle (filtered)
- Joint velocity (filtered)
- Estimated acceleration
- Contact probability
- Optional phase progress estimate

This becomes the authoritative state.

### 4.3 Gait Phase Topic

Contains:

- Discrete phase label (STANCE / SWING / TRANSITION)
- Continuous phase variable (0–1)
- Confidence metric

No torque data allowed here.

### 4.4 Adaptive Parameter Topic

Contains:

- Assistance gain
- Torque profile scaling
- Timing offsets
- Safety scaling factor

This layer modifies controller behavior indirectly.

### 4.5 Torque Command Topic

Contains:

- Desired joint torque
- Saturation status
- Safety flag

This is the only command to hardware.

## 5. Control Strategy Definition

### Primary Strategy:

Phase-based assistive torque profiles

Torque as a function of:
- Gait phase
- Joint states
- Adaptive parameters

      Torque = f(gait_phase, joint_state, adaptive_parameters)

In other words, there will be no trajectory tracking.

### Safety Constraints

Hard limits:

- Maximum torque
- Maximum torque rate
- Disable torque on fault
- Watchdog timeout

Safety must be defined before real implementation.

## 6. Simulation Strategy

Initial implementation will run entirely in simulation.

Simulation must provide:

- Limb kinematics
- Contact state
- Configurable gait pattern

Hardware integration deferred until full stack validated.

## 7. Modularity Principles

Explicit design rules to follow:

- No node reads raw sensor data except state estimation layer.
- Only torque controller publishes torque.
- Each node subscribes only to upstream layer.
- No cyclic dependencies.

## 8. Failure & Robustness Assumptions

### 8.1 Failure Model Definitions
#### 1. Sensor Dropout
##### Definition

A sensor is considered in dropout if:

1. No message received within 2× expected period, or
2. Message timestamp drift exceeds tolerance threshold.

##### Example thresholds:

- Encoder @ 500 Hz → timeout > 4 ms
- IMU @ 500 Hz → timeout > 4 ms
- Contact @ 200 Hz → timeout > 10 ms

##### Severity Levels

Level 1 — Transient Dropout
- Single missed message
- System continues operating

Level 2 — Persistent Dropout
- Timeout exceeds threshold
- Degrade estimation

Level 3 — Critical Dropout
- Encoder missing
- Immediate transition to FAULT mode

#### 2. Delayed Messages
##### Definition

A message is delayed if:

      (current_time - message_timestamp) > latency_budget

##### Example latency budgets:

- State estimate: 5 ms
- Gait phase: 10 ms
- Torque command: 2 ms

##### Handling Strategy

1. If delayed but valid → discard message
2. If delay persistent → degrade to safe mode
3. If control-layer message delayed → zero torque

####  3. Node Restart / Crash
##### Definition

A node is considered failed if:

1. It stops publishing within defined timeout
2. Lifecycle state transitions unexpectedly
3. Process exits

##### Monitoring

Watchdog node monitors:
- State estimator
- Gait detector
- Torque controller

##### Reaction Levels

Non-critical node restart (e.g., adaptation):

- Freeze last known safe parameters
- Continue assistive mode

Critical node restart (state estimator or controller):

- Immediate torque zero 
- Transition to FAULT

#### 4. Actuator Stall
##### Definition

Actuator stall is detected if:

1. Commanded torque ≠ measured torque beyond threshold
2. Velocity ≈ 0 while torque command high
3. Driver fault flag raised

Threshold example:

      |command − measured| > 15% for > 10 ms

##### Response

- First event → torque ramp-down
- Persistent stall → FAULT mode
- Log diagnostic

---
### 8.2 Robustness Behavior Definitions

#### 1. Graceful Degradation

System continues operating with reduced capability when possible.

##### Example Scenarios

IMU Failure:

- Disable orientation-based estimation
- Use encoder-only estimation
- Reduce torque gain by 30%

Contact Sensor Failure:

- Switch to kinematic-based phase detection
- Reduce confidence metric

Adaptation Failure:

- Freeze adaptive parameters
- Continue fixed assistance

Principle:

Remove intelligence before removing torque.

#### 2. Controlled Shutdown

Triggered when:

- Encoder lost
- Torque control failure
- Actuator stall persistent
- Watchdog timeout

Controlled Shutdown Behavior

- Immediately ramp torque to zero within safe rate
- Disable controller output
- Transition to FAULT mode
- Log failure state
- Require manual reset

No abrupt torque cut unless hardware E-stop.

---
### 8.3 Watchdog Specification

Monitors:

1. Topic liveliness
2. Timestamp freshness
3. Control loop period jitter
4. Node lifecycle state

Watchdog timeouts:

| Component       | Timeout |
| --------------- | ------- |
| Sensor layer    | 5 ms    |
| State estimator | 10 ms   |
| Gait detector   | 20 ms   |
| Controller      | 5 ms    |

If timeout exceeded:

- Trigger degradation or shutdown based on criticality.

---
### 8.4 Criticality Classification

Defines system components by level of criticality.

Critical (Failure → Immediate FAULT):
- Encoder
- Torque controller
- Hardware interface

Medium (Failure → Degrade):
- IMU
- Gait detector

Low (Failure → Freeze):
- Adaptation layer
- Logging

---
### 8.5 Fault Escalation Policy

Escalation ladder will follow
1. Warning
2. Degraded mode
3. Torque reduction
4. Controlled shutdown
5. Hardware E-stop

---
### 8.6 Deterministic Recovery Policy

After FAULT:

1. Torque remains zero
2. All nodes must reinitialize
3. Calibration required before returning to Assist mode

No automatic return to assistive mode.

---
### 8.7 Logging & Diagnostics Requirements

On any failure event, logs will include:

- Timestamp
- Failure type
- Node involved
- Last torque command
- Last state estimate

Persistent log storage is required for future analysis.

---
### 8.8 Summary Design Rules

1. **No uncontrolled torque.**
2. **No silent failure.**
3. **No cyclic dependencies.**
4. **Every critical topic has a timeout.**
5. **Every failure has a deterministic response.**
6. **Degradation preferred over shutdown when safe.**
7. **Hardware limits override software decisions.**
