# bench/ — hardware-in-the-loop & validation tools

Companion tools for my ESP32-S3 flight-controller firmware,
[Drone_FlightCont](https://github.com/malek23ma/Drone_FlightCont).
They reuse the simulator's plant (`quad_plant.m`, `model_params.m`).
Nothing in the simulator depends on this folder.

## Firmware side (Drone_FlightCont)

- 50 Hz CSV debug stream over USB (`D,...` lines), see `main/debug_stream.h`
- serial commands: `STREAM,0|1`
- HIL mode (`HIL_MODE` in `components/common/nav_state.h`):
  sensor injection `S,ax,ay,az,gx,gy,gz,mx,my,mz`, RC injection
  `R,roll,pitch,thr,yaw,arm` (1000–2000). **Never fly HIL builds.**

## MATLAB scripts (run from this folder, or `addpath bench`)

| Script | Purpose |
|---|---|
| `fc_open` / `fc_parse_line` | serial helpers |
| `live_attitude("COM5")` | live 3D cube from onboard quaternion |
| `fc_record("COM5",120,"name")` | record stream → `logs/name.mat/.csv` |
| `compare_madgwick("name")` | onboard vs offline replay vs MATLAB `imufilter` |
| `tune_beta("static","moves")` | Madgwick β sweep (noise vs recovery) |
| `timing_report("name")` | loop timing / jitter audit |
| `sil_cascade()` | firmware PID+mixer port vs 6-DOF plant, sign check |
| `hil_run("COM5")` | true HIL: real FC flies the simulated plant |
| `hil_fly("COM5")` | piloted HIL with joystick/keyboard + live 3D view |
| `mag_cal("COM5")` / `mag_frame_check("name")` | magnetometer calibration, mag-vs-IMU frame check |
| `telemetry_stress("file")` | C6 link packet-loss analysis |

`logs/` is created by `fc_record`; recorded logs are not included in this repository.

Ports of firmware code: `fw_pid.m`, `fw_pid_init.m`, `fw_mixer.m`,
`madgwick_m.m`. Frame conversions documented in `sensor_synth.m`
(plant NED/FRD ↔ IMU chip z-up) and `pwm_to_forces.m` (firmware motor
numbering FR/FL/RR/RL ≠ sim's M1..M4 — handled explicitly).
