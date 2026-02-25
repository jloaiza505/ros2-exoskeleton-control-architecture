# Simulation Data Plan (OpenSim Playback)

## 1. Goal

Add a realistic data-source mode to `exo_simulation` so raw sensor output can be driven by OpenSim gait files instead of only synthetic sinusoidal signals.

## 2. Scope

Target package:

- `src/exo_simulation`

Target interface (unchanged):

- `/exo/sensing/raw` using `exo_interfaces/msg/RawSensorData`

## 3. Source Modes

Planned runtime parameter:

- `data_source_mode`:
  - `synthetic` (current default)
  - `opensim_file` (planned)

## 4. Planned Parameters

- `opensim_file_path` (string): path to `.mot` or `.sto` gait file
- `loop_playback` (bool): restart when end of file is reached
- `time_scale` (double): playback speed multiplier
- `publish_rate_hz` (double): ROS publish rate
- `contact_threshold` (double): threshold for contact proxy generation
- `velocity_source` (string): `file` or `numerical_derivative`
- `imu_proxy_scale` (double): scale mapping from kinematics to IMU proxy

## 5. Data Mapping to `RawSensorData`

- `joint_angle_rad` <- knee angle column from OpenSim data
- `imu_orientation_pitch_rad` <- mapped/proxy orientation from angle
- `imu_angular_velocity_y_rad_s` <- mapped/proxy velocity
- `contact_state` <- event/threshold-based contact proxy
- `header.stamp` <- ROS time at publish point

## 6. Processing Pipeline (Planned)

1. Load file and parse time + required columns.
2. Validate monotonic time and required field availability.
3. Build interpolation objects for runtime sampling.
4. At each timer tick:
- sample trajectory by playback clock,
- compute derived signals (if needed),
- publish `RawSensorData`.

## 7. Compatibility Rules

- Keep `RawSensorData` schema unchanged.
- Preserve topic names and parameter conventions.
- Allow switching back to `synthetic` mode with no downstream changes.

## 8. Validation Criteria

- File playback starts with no parser/runtime errors for supported gait files.
- Published stream remains stable at target rate.
- Downstream nodes (`state_estimator`, `gait_phase_detector`, `torque_controller`) operate unchanged.
- Realism improvement is visible in plots versus synthetic mode.

## 9. Risks and Notes

- OpenSim files can vary in column naming; mapping config may be required.
- Contact state may not be directly provided and may need heuristic derivation.
- Unit normalization (degrees vs radians) must be explicit and validated.
