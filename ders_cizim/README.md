# MonsterBorg RGB Line-Following Project

This Webots project simulates a PiBorg/MonsterBorg-style four-wheel robot that follows a common black line and then selects a red, green, or blue branch.

## Current Controller

The active RGB world is `worlds/monsterborg_rgb_rl.wbt`.

The active controller is `controllers/rgb_rl_controller/rgb_rl_controller.py`.

The controller now uses shared modules:

- `control_core.py`: action names, wheel-speed conversion, safety clipping, reward, target-search branch bias.
- `mission.py`: configurable start pose jitter, target parking zones, and terminal episode reasons.
- `robot_config.py`: camera pose, camera sensor settings, speed limits, environment parsing.
- `sim_metrics.py`: repeatable smoke-run summaries with goal-success gates.
- `vision_adapter.py`: Webots-like adapter for RGB arrays from Pi camera frames.
- `hardware_pi.py`: lazy Raspberry Pi camera and ThunderBorg motor adapters.

## Algorithm

The runtime is split into three layers:

1. Perception samples the lower camera image, classifies black/red/green/blue pixels, segments the visible line, and emits `visible`, `center_error`, `confidence`, `color_name`, and `matched_target`.
2. Control follows the common black line until the fork, then either locks onto the selected color or applies a small branch-search bias for green/blue. The selected action is converted into clipped differential wheel speeds.
3. Mission supervision is simulation-only: Webots global pose is used to reset randomized starts and stop the episode when the robot reaches the selected color parking zone, leaves the board, loses the line for too long, or times out.

The Raspberry Pi path should reuse layers 1 and 2. Layer 3 should be replaced by a real-world stop rule such as a visible finish marker, a measured distance gate, or a manual/operator stop.

## Mission Goals

Default parking zones on the current RGB texture:

```text
red   center=(0.80, -0.55) radius=0.18
green center=(0.10, -0.50) radius=0.20
blue  center=(0.32, -0.28) radius=0.20
```

Mission success requires the robot to stop inside the selected zone with a default `0.02 m` clearance from the zone boundary. This avoids counting first-contact boundary crossings as parked.

Override them without editing code:

```powershell
$env:MONSTERBORG_RL_GOAL_ZONES='red=0.80:-0.55:0.18,green=0.10:-0.50:0.20,blue=0.32:-0.28:0.20'
$env:MONSTERBORG_RL_GOAL_REACH_CLEARANCE='0.02'
```

Terminal reasons are written into logs as `reached_goal`, `off_board`, `lost_line`, or `timeout`. The smoke matrix requires `reached_goal` and at least `0.015 m` final goal margin. Summaries include `final_goal_distance` and `final_goal_margin`.

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
$env:MONSTERBORG_RL_START_COLOR='green'
$env:MONSTERBORG_RL_MAX_STEPS='950'
$env:MONSTERBORG_RL_STEP_LOG_PATH='ders_cizim\artifacts\rgb_rl\run_green_950.json'
$env:MONSTERBORG_RL_SUMMARY_PATH='ders_cizim\artifacts\rgb_rl\run_green_950_summary.json'
& 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' --mode=fast --stdout --stderr --minimize 'ders_cizim\worlds\monsterborg_rgb_rl.wbt'
```

Summarize logs:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.sim_metrics 'ders_cizim\artifacts\rgb_rl\run_*_900.json'
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
  --colors red green blue `
  --steps 950 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\mission_variant_matrix_950'
```

Current verified matrix result:

```text
18/18 variant-color runs reached the correct goal zone
visible ratio: 0.974 to 1.000
first target lock: step 482 to 567
final goal margin: 0.020 m to 0.022 m
off-board detections: 0
```

Run randomized start episodes:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures 'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_00.png' `
  --colors red green blue `
  --steps 950 `
  --episodes 3 `
  --seed 31 `
  --start-lateral-jitter 0.018 `
  --start-longitudinal-jitter 0.025 `
  --start-heading-jitter 0.08 `
  --speed-scale-jitter 0.04 `
  --camera-noise 0.01 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\mission_random_perturbed_variant00'
```

Current verified randomized/perturbed result:

```text
9/9 randomized start, motor-asymmetry, and camera-noise runs reached the correct goal zone
final goal margin: 0.020 m to 0.021 m
off-board detections: 0
```

Motor asymmetry can also be set directly:

```powershell
$env:MONSTERBORG_RL_LEFT_SPEED_SCALE='0.96'
$env:MONSTERBORG_RL_RIGHT_SPEED_SCALE='1.03'
```

Run a camera-mount perturbation matrix:

```powershell
python -m ders_cizim.controllers.rgb_rl_controller.smoke_matrix `
  --webots 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe' `
  --world 'ders_cizim\worlds\monsterborg_rgb_rl.wbt' `
  --textures 'ders_cizim\worlds\textures\variants\rgb_training_tracks_variant_02.png' `
  --colors red green blue `
  --steps 950 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\mission_camera_pose_matrix_950' `
  --camera-poses `
    'nominal:-0.13 0 0|0 1 0 -1.2' `
    'tilt_low:-0.13 0 0|0 1 0 -1.12' `
    'tilt_high:-0.13 0 0|0 1 0 -1.28' `
    'front_offset:-0.14 0 0|0 1 0 -1.2'
```

Current verified camera-mount matrix result:

```text
12/12 pose-color runs reached the correct goal zone on dim variant_02
visible ratio: 1.000 in all runs
first target lock: step 472 to 567
final goal margin: 0.020 m to 0.022 m
off-board detections: 0
```

## Branch Selection

With a lower, more realistic front camera, red can enter the camera before green/blue. The controller therefore uses a target-search bias after the fork for non-red targets:

```text
green=hard_left
blue=left
```

Override this for another track layout:

```powershell
$env:MONSTERBORG_RL_TARGET_SEARCH_ACTIONS='green=left,blue=soft_left,red=right'
```

Move the start of target search if your fork is in a different place:

```powershell
$env:MONSTERBORG_RL_BRANCH_SEARCH_MIN_X='0.30'
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
  --colors red green blue `
  --steps 1100 `
  --out-dir 'ders_cizim\artifacts\rgb_rl\stress_variant_matrix_1100'
```

Current verified stress result:

```text
12/12 stress texture runs reached the correct goal zone
visible ratio: 0.978 to 1.000
first target lock: step 482 to 557
final goal margin: 0.020 m to 0.021 m
off-board detections: 0
```

## Raspberry Pi Transfer

The Pi path should use the same perception/control contract:

1. Capture RGB frames with `hardware_pi.PiCameraFrameSource`.
2. Wrap each frame with `vision_adapter.RgbArrayCamera`.
3. Analyze it with `rgb_rl_controller.analyze_rgb_camera(..., RgbArrayCameraApi, target_color, previous_error)`.
4. Convert policy actions with `control_core.action_to_command`.
5. Send clipped commands through `hardware_pi.ThunderBorgMotorSink`.

Start conservatively:

```powershell
$env:MONSTERBORG_HARDWARE_OUTPUT_LIMIT='0.35'
```

On the physical robot, verify these before autonomous driving:

- The camera frame sees line plus floor, not only line.
- `line_width_ratio` is roughly `0.10` to `0.25` on straight segments.
- Left/right motor signs match Webots.
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
