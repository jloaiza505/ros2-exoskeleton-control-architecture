# Operator Dashboard (Scaffold)

Initial scaffold for the exoskeleton operator UI (Phase A).

## Recommended stack

- React + TypeScript + Vite
- `roslibjs` for ROS topic/service integration via `rosbridge_server`
- CSS variables + lightweight custom styles for fast iteration

Why this stack for this project:

- Fast UI iteration while your ROS2 backend evolves.
- Direct mapping to your topics/services without introducing backend middleware yet.
- Easy transition later to charting/state libraries (e.g. Recharts + Zustand) once interactions stabilize.

## Prerequisites

1. `rosbridge_server` running in your ROS environment.
2. This control stack running (`exo_bringup`).

Example rosbridge launch:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Run

```bash
cd ui/operator-dashboard
npm install
npm run dev
```

Default URL is `ws://localhost:9090`.
Override with:

```bash
cp .env.example .env
# edit VITE_ROSBRIDGE_URL
```

## Current scaffold features

- Connection status indicator.
- Subscriptions:
  - `/exo/system/mode`
  - `/exo/system/calibration_status`
  - `/exo/system/user_message`
- Service controls:
  - `/exo/system/start_unit`
  - `/exo/system/shutdown_unit`
  - `/exo/system/start_simulation`
  - `/exo/system/stop_simulation`
  - `/exo/system/set_mode`
  - `/exo/system/run_full_calibration`
  - `/exo/system/validate_calibration`
- User message panel ready for future GUI integration.

## Next UI increments

1. Add live plots (torque, gait, state, sensor).
2. Add stale-topic indicators and rates panel.
3. Add mode transition guardrails + confirm dialog for ASSISTIVE.
4. Add calibration flow wizard (Fit -> Calibrate -> Validate -> Ready).
