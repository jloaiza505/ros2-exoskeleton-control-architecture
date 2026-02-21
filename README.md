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

Define now:

- Sensor dropout possible
- Delayed messages possible
- Node restart possible
- Actuator stall possible

Architecture must tolerate:
- Graceful degradation
- Controlled shutdown