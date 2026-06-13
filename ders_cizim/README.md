# MonsterBorg RGB Line-Following Project

This Webots project simulates a PiBorg/MonsterBorg-style four-wheel robot that follows a common start line and then selects either the red or blue course.

## Current Controller

The active RGB world is `worlds/monsterborg_rgb_rl.wbt`.

The active controller is `controllers/rgb_rl_controller/rgb_rl_controller.py`.

The controller now uses shared modules:

- `control_core.py`: action names, wheel-speed conversion, safety clipping, reward, target-search branch bias.
- `mission.py`: configurable start pose jitter, red/blue target parking zones, optional sequence mode, and terminal episode reasons.
- `robot_config.py`: camera pose, camera sensor settings, speed limits, environment parsing.
- `sim_metrics.py`: repeatable smoke-run summaries with goal-success gates.
- `vision_adapter.py`: Webots-like adapter for RGB arrays from Pi camera frames.
- `hardware_pi.py`: lazy Raspberry Pi camera and TB6612 GPIO motor adapters.

## Algorithm

The runtime is split into three layers:

1. Perception samples the lower camera image, classifies black/red/green/blue pixels, segments the visible line, and emits `visible`, `center_error`, `confidence`, `color_name`, and `matched_target`.
2. Control can be started with `red` or `blue` and selects actions through a tabular Q-learning policy. In training mode, Webots episodes update the Q-table from camera state, action, reward, and next-state transitions. In run mode, the loaded or initialized Q values choose the action, which is converted into clipped differential wheel speeds.
3. Mission supervision is simulation-only: Webots global pose is used to reset randomized starts, score the selected goal-zone visit, and stop the episode when the mission is done or unsafe.

The Raspberry Pi path should reuse layers 1 and 2. Layer 3 should be replaced by a real-world stop rule such as a visible finish marker, a measured distance gate, or a manual/operator stop.

## Mission Goals

Default parking zones on the current red/blue texture:

```text
red  center=(0.80, 0.36) radius=0.18
blue center=(-0.72, 0.52) radius=0.18
```

Mission success requires the robot to stop inside the selected zone with a default `0.02 m` clearance from the zone boundary. This avoids counting first-contact boundary crossings as parked.

Override them without editing code:

```powershell
$env:MONSTERBORG_RL_GOAL_ZONES='red=0.80:0.36:0.18,blue=-0.72:0.52:0.18'
$env:MONSTERBORG_RL_GOAL_REACH_CLEARANCE='0.02'
```

Terminal reasons are written into logs as `reached_goal`, `off_board`, `lost_line`, or `timeout`. Single-color smoke runs require `reached_goal`. Summaries include `final_goal_distance` and `final_goal_margin`.

## Camera Pose

The RGB world camera is front-mounted and angled down-forward:

```text
translation -0.13 0 0
rotation 0 1 0 -1.2
fieldOfView 1.25
near 0.02
width 96
height 96
```

Override in Webots without editing the world:

```powershell
$env:MONSTERBORG_CAMERA_TRANSLATION='-0.13 0 0'
$env:MONSTERBORG_CAMERA_ROTATION='0 1 0 -1.2'
$env:MONSTERBORG_CAMERA_NOISE='0.01'
```

A useful camera frame should show line plus surrounding floor. In the current simulation, the first frame reports `line_width_ratio` around `0.158` and `confidence` `1.0`.

## Run Checks

Run unit tests:

```powershell
python -m unittest discover -s tests -v
```

Capture one camera frame:

```powershell
$env:MONSTERBORG_RL_CAPTURE_PATH='ders_cizim\artifacts\rgb_rl\camera_check.ppm'
$env:MONSTERBORG_RL_CAPTURE_STEP='1'
$env:MONSTERBORG_RL_QUIT_AFTER_CAPTURE='1'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' --mode=fast --stdout --stderr --minimize 'ders_cizim\worlds\monsterborg_rgb_rl.wbt'
```

Run a mission smoke episode:

```powershell
$env:MONSTERBORG_RL_START_COLOR='red'
$env:MONSTERBORG_RL_MAX_STEPS='4600'
$env:MONSTERBORG_RL_STEP_LOG_PATH='ders_cizim\artifacts\rgb_rl\run_red_4600.json'
$env:MONSTERBORG_RL_SUMMARY_PATH='ders_cizim\artifacts\rgb_rl\run_red_4600_summary.json'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' --mode=fast --stdout --stderr --minimize 'ders_cizim\worlds\monsterborg_rgb_rl.wbt'
```

Train or refresh the Q-table:

```powershell
$env:MONSTERBORG_RL_MODE='train'
$env:MONSTERBORG_RL_TRAIN_STEPS='60000'
$env:MONSTERBORG_RL_SAVE_INTERVAL='1000'
$env:MONSTERBORG_RL_Q_TABLE='ders_cizim\artifacts\rgb_rl\q_table.json'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' --mode=fast --stdout --stderr --minimize 'ders_cizim\worlds\monsterborg_rgb_rl.wbt'
```

For deployment or smoke runs, leave `MONSTERBORG_RL_MODE` unset or set it to `run`. If `MONSTERBORG_RL_Q_TABLE` points to a trained table, the controller uses those values; unseen states are initialized as Q values and can be improved by another training pass.

Change `MONSTERBORG_RL_START_COLOR` to `blue` to run the blue branch.

Run the red/blue mission matrix with randomized starts:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures 'ders_cizim\worlds\textures\rgb_training_tracks.png' `
  --colors red blue `
  --steps 4600 `
  --episodes 2 `
  --seed 77 `
  --start-lateral-jitter 0.02 `
  --start-longitudinal-jitter 0.025 `
  --start-heading-jitter 0.035 `
  --camera-noise 0.003 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\red_blue_check'
```

The default Webots world now has a red branch and a blue branch. Use `MONSTERBORG_RL_START_COLOR=red` or `MONSTERBORG_RL_START_COLOR=blue` for one direct run, or `--colors red blue` in the smoke matrix.

Current nominal Webots smoke result on the default texture:

```text
2/2 red-blue runs reached the correct goal zone
red  final goal margin: 0.021 m
blue final goal margin: 0.020 m
```

For later Q-learning runs, `MONSTERBORG_RL_START_COLOR=random` now samples only `red` and `blue`:

```powershell
$env:MONSTERBORG_RL_MODE='train'
$env:MONSTERBORG_RL_START_COLOR='random'
$env:MONSTERBORG_RL_TRAIN_STEPS='60000'
$env:MONSTERBORG_RL_EPISODE_STEPS='4600'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' --mode=fast --stdout --stderr --minimize 'ders_cizim\worlds\monsterborg_rgb_rl.wbt'
```

Summarize logs:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.sim_metrics 'ders_cizim\artifacts\rgb_rl\run_*_4600.json'
```

Run a texture-variant smoke matrix:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures `
    'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_00.png' `
    'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_01.png' `
    'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_02.png' `
    'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_03.png' `
    'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_04.png' `
    'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_05.png' `
  --colors red blue `
  --steps 4600 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\red_blue_variant_matrix_4600'
```

A passing red/blue matrix reports `terminal=reached_goal`, `success=1`, and `pass=1` for each texture-color case.

To watch the same matrix in Webots instead of running minimized, add:

```powershell
  --visible `
  --webots-mode fast
```

Run randomized start episodes:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures 'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_00.png' `
  --colors red blue `
  --steps 4600 `
  --episodes 3 `
  --seed 31 `
  --start-lateral-jitter 0.018 `
  --start-longitudinal-jitter 0.025 `
  --start-heading-jitter 0.08 `
  --speed-scale-jitter 0.04 `
  --camera-noise 0.01 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\red_blue_random_perturbed_variant00'
```

Motor asymmetry can also be set directly:

```powershell
$env:MONSTERBORG_RL_LEFT_SPEED_SCALE='0.96'
$env:MONSTERBORG_RL_RIGHT_SPEED_SCALE='1.03'
```

Add transfer realism without editing code:

```powershell
$env:MONSTERBORG_RL_MOTOR_DEADBAND='0.03'
$env:MONSTERBORG_RL_SPEED_NOISE_STD='0.01'
$env:MONSTERBORG_RL_COMMAND_LATENCY_STEPS='1'
```

These are intentionally off by default. Use small values after the nominal run passes; the expected course should have mild bumps and imperfect tape, not extreme sensor noise. Terminal stops bypass latency so `reached_goal`, `lost_line`, and other stop conditions still command zero speed immediately.

Run a camera-mount perturbation matrix:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures 'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_02.png' `
  --colors red blue `
  --steps 4600 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\red_blue_camera_pose_matrix_4600' `
  --camera-poses `
    'nominal:-0.13 0 0|0 1 0 -1.2' `
    'tilt_low:-0.13 0 0|0 1 0 -1.12' `
    'tilt_high:-0.13 0 0|0 1 0 -1.28' `
    'front_offset:-0.14 0 0|0 1 0 -1.2'
```

## Branch Selection

The default Webots course uses a common start line and then branches to red or blue. The target-search environment variables can be tuned for the fork:

```powershell
$env:MONSTERBORG_RL_TARGET_SEARCH_ACTIONS='blue=left,red=straight'
```

## Track Variants

Generate deterministic texture variants for robustness testing:

```powershell
python ders_cizim\worlds\textures\generate_rgb_training_tracks.py --variants 6 --seed 11 --output-dir ders_cizim\worlds\textures\variants
```

Variants alter line width, brightness, and color intensity while keeping route topology stable.

Generate stress variants with small route perturbations, short tape gaps, dirty floor patches, and speckles:

```powershell
python ders_cizim\worlds\textures\generate_rgb_training_tracks.py `
  --stress-variants 4 `
  --seed 53 `
  --output-dir ders_cizim\worlds\textures\stress_variants
```

Stress variants are for pre-hardware confidence checks. They are intentionally imperfect but still represent the same route and target zones.

Run the stress matrix:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures `
    'ders_cizim\worlds\textures\stress_variants\rgb_training_tracks_stress_00.png' `
    'ders_cizim\worlds\textures\stress_variants\rgb_training_tracks_stress_01.png' `
    'ders_cizim\worlds\textures\stress_variants\rgb_training_tracks_stress_02.png' `
    'ders_cizim\worlds\textures\stress_variants\rgb_training_tracks_stress_03.png' `
  --colors red blue `
  --steps 5000 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\red_blue_stress_variant_matrix_5000'
```

A passing stress run should still report `reached_goal`; failures usually mean the line gap or camera perturbation is too aggressive.

Additional color-transfer checks can use the generated recolor textures:

```text
worlds/textures/color_variants/rgb_training_tracks_green.png
worlds/textures/color_variants/rgb_training_tracks_blue.png
```

When using these recolored single-route textures, set the matching target color and override that color's goal zone to the same route endpoint, for example `green=0.80:0.36:0.18`.

## Raspberry Pi Transfer

The Pi path should use the same perception/control contract:

1. Capture RGB frames with `hardware_pi.PiCameraFrameSource`.
2. Wrap each frame with `vision_adapter.RgbArrayCamera`.
3. Analyze it with `rgb_rl_controller.analyze_rgb_camera(..., RgbArrayCameraApi, target_color, previous_error)`.
4. Convert policy actions with `control_core.action_to_command`.
5. Send clipped commands through `hardware_pi.TB6612GPIOMotorSink`.

Start conservatively:

```powershell
$env:MONSTERBORG_HARDWARE_OUTPUT_LIMIT='0.35'
```

Laptop setup:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
python -m unittest discover -s tests -v
```

On macOS/Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m unittest discover -s tests -v
```

Raspberry Pi setup should also install the OS-provided camera stack and an `RPi.GPIO`-compatible GPIO library. The project package intentionally keeps those hardware libraries lazy so laptop tests can run without them.

The TB6612 code uses BCM GPIO numbers, matching this wiring:

| Motor | Driver channel | PWM GPIO | IN1 GPIO | IN2 GPIO | STBY GPIO |
| --- | --- | ---: | ---: | ---: | ---: |
| Right rear | Driver #1 A01-A02 | 12 | 5 | 6 | 21 |
| Right front | Driver #1 B01-B02 | 13 | 16 | 20 | 21 |
| Left front | Driver #2 A01-A02 | 18 | 23 | 24 | 27 |
| Left rear | Driver #2 B01-B02 | 19 | 25 | 26 | 27 |

Power wiring stays outside the code: TB6612 `VM` goes to main power `IN+`, `VCC` goes to Pi `3.3V`, and every driver/Pi ground must share the same `GND`/`IN-` line. Do not let `VM` touch `VCC`.

On the physical robot, verify these before autonomous driving:

- The camera frame sees line plus floor, not only line.
- `line_width_ratio` is roughly `0.10` to `0.25` on straight segments.
- Left/right motor signs match Webots. If one motor runs backward, swap that motor's two output wires or reverse its `TB6612MotorSigns` entry in code.
- Battery voltage does not sag enough to change steering.
- A manual stop path is available before enabling closed-loop motion.

Run the camera-only probe on the Raspberry Pi before enabling motors:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.hardware_probe `
  --target red `
  --frames 60 `
  --output hardware_probe_red.json
```

Replay a saved image on the laptop:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.hardware_probe `
  --image ders_cizim\artifacts\rgb_rl\camera_pose_default_front_mid.png `
  --target red `
  --frames 5 `
  --output ders_cizim\artifacts\rgb_rl\hardware_probe_replay.json
```

The probe is safe: it reads camera frames only and never sends motor commands.

After the camera probe is ready, run the motor sign calibration in dry-run mode:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.hardware_motor_calibration `
  --power 0.12 `
  --pulse-seconds 0.35
```

Only use real motors when the robot is lifted off the ground and a manual stop is available:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.hardware_motor_calibration `
  --power 0.12 `
  --pulse-seconds 0.35 `
  --armed
```

The calibration script clamps power to `0.25`, inserts stops between pulses, and defaults to dry-run unless `--armed` is provided.

Run the physical closed-loop controller only after the probe and motor-sign calibration pass. It defaults to dry-run motor logging:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.hardware_runner `
  --target red `
  --frames 400 `
  --max-seconds 20 `
  --hardware-output-limit 0.30 `
  --branch-search-after-frames 80 `
  --stop-file stop_robot.txt `
  --output hardware_run_red.json
```

When the robot is on the floor, the camera is ready, and someone is next to it with a manual stop path, add `--armed`:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.hardware_runner `
  --target red `
  --frames 400 `
  --max-seconds 20 `
  --hardware-output-limit 0.30 `
  --branch-search-after-frames 80 `
  --stop-file stop_robot.txt `
  --output hardware_run_red.json `
  --armed
```

If `stop_robot.txt` exists, the runner stops before sending the next command. If the line disappears, it commands stop while waiting for recovery and exits after the configured lost-frame limit. If the warmup camera frames fail the visibility, confidence, or line-width gate, it exits without driving.
