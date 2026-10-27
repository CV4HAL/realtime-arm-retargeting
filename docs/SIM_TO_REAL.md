# Switching from the Lite 6 simulator to the real arm

This guide explains how to point the CV bridge (`src/sim_bridge.py`) at the physical
UFACTORY Lite 6 instead of the Docker simulator, and how to go back. It documents the
current code; no control logic changes are needed to follow it.

> **Status:** the real arm has not been tested with this pipeline yet. Values that depend on
> the hardware (arm IP, subnet, speed and workspace limits, tool load) are not in this repo
> or in the xArm-Python-SDK (v1.18.4, see `requirements.txt`), so each one below says
> **how to find it** instead of giving a number.

## 1. Prerequisites

- **Network:** laptop and Lite 6 controller on the same network (direct Ethernet cable or a
  shared switch/router).
- **Arm IP:** not stored in this repo. Get it from UFACTORY Studio's connection screen or
  the control box (see `SIMULATOR.md`, "Switching to the physical robot"), from your
  router's DHCP leases, or by asking whoever set up the arm. Write it down in the team's
  notes and use it wherever `<lite6-ip>` appears below.
- **Static IP on the laptop:** look at the arm's IP and subnet mask, then give the laptop's
  Ethernet interface a fixed address in the same subnet with a different host number
  (for example with NetworkManager: `nmcli con mod <ethernet-connection> ipv4.method manual
  ipv4.addresses <laptop-ip>/<prefix>`, then reconnect). This keeps DHCP from changing it
  mid-session.
- **Verify the connection** before running any code:
  ```bash
  ping -c 3 <lite6-ip>
  ```
  and open UFACTORY Studio at `http://<lite6-ip>:18333` (same port the simulator uses,
  see `SIMULATOR.md`). If it does not load on the real controller, check the address and
  port shown in the UFACTORY documentation for your controller.
- Python environment ready: `./run.sh check`.
- Test the vision side without the robot first: `./run.sh sim --dry-run`.

## 2. What to change in the code

There is a single place: the default of `--ip`.

| Where | What |
|---|---|
| `src/sim_bridge.py:453-457` (`parse_args`) | Simulator line is active; the real-arm line is commented out right below it. Swap which one is commented. |

```python
# SIM-TO-REAL: Simulator (default). Comment this line out to use the real Lite 6.
parser.add_argument("--ip", default="127.0.0.1", help="robot or simulator IP")
# SIM-TO-REAL: Real Lite 6. Uncomment this line (and comment the one above) ...
# parser.add_argument("--ip", default="192.168.1.XXX", ...)  # replace XXX with the Lite 6 IP (section 1)
```

You can also avoid editing code entirely and pass the IP on the command line:

```bash
./run.sh sim --ip <lite6-ip>
```

The IP flows `args.ip` (`sim_bridge.py:337`) -> `RobotLink.__init__` -> `XArmAPI(self.ip)`
(`sim_bridge.py:158`). Everything else (the Docker container, `./run.sh sim-up`) is only
for the simulator.

Other places in the code marked with `# SIM-TO-REAL:` (comments only, nothing to change
yet, but each matters on the real arm):

| Line | Why it matters |
|---|---|
| `sim_bridge.py:156` | `XArmAPI(self.ip)` opens a connection to the physical controller. |
| `sim_bridge.py:166` | `prepare()` enables motors. Add reduced mode / speed / boundary limits here (see section 3). |
| `sim_bridge.py:179` | `go_home()` moves the arm right after connecting, at `HOME_SPEED_DEG_S`. |
| `sim_bridge.py:188` | `start_servo()` enters servo mode (`set_mode(1)`); step limits are `MAX_STEP_MM` / `MAX_ANGLE_STEP_DEG`. |
| `sim_bridge.py:224` | `set_servo_cartesian(...)` is the call that moves the arm. |
| `sim_bridge.py:239` | `recover()` auto-clears errors and re-enables motion. |
| `sim_bridge.py:254` | `shutdown()` uses `set_state(4)`; it does not replace the physical e-stop. |

Other references to `127.0.0.1` that are documentation only: `run.sh:45` (help text),
`README.md:67`, and `SIMULATOR.md`.

## 3. Safety checklist (before enabling motors)

Tick every item before the first real run.

- [ ] A person stands at the e-stop (physical button) and can reach it at all times.
- [ ] Workspace is clear: nothing and nobody inside the arm's reach, including the path
      to the home pose.
- [ ] Reduced speed and acceleration. In the code today: `HOME_SPEED_DEG_S = 30`,
      `MOVE_SPEED_MM_S = 100`, `MAX_STEP_MM = 8.0` and `MAX_ANGLE_STEP_DEG = 2.0` per tick at
      `SERVO_RATE_HZ = 30`, i.e. roughly 240 mm/s and 60 deg/s peak in servo mode.
      Lower them for the first run. The SDK also offers `set_reduced_mode`,
      `set_reduced_max_tcp_speed` and `set_tcp_maxacc`. Start with the lowest speed you are
      comfortable with and read what the controller currently has with `arm.get_reduced_states()`.
      To check that reduced mode also limits servo-mode commands, turn it on with a low speed,
      run the bridge with small hand motions and watch the arm's real speed before trusting it.
- [ ] Workspace limits. Today the only limit is the software box
      `BOX_CENTER_MM = (250, 0, 200)` +/- `BOX_HALF_SIZE_MM = (100, 150, 100)` mm. The SDK offers
      `set_reduced_tcp_boundary` and `set_fence_mode` for controller-side limits.
      To get the values: jog the arm to the corners of the box in UFACTORY Studio, record those
      coordinates (mm), and pass them to `set_reduced_tcp_boundary([x_max, x_min, y_max, y_min,
      z_max, z_min])`. This also shows whether the box is reachable and collision-free in the cell.
- [ ] No active errors or warnings. `prepare()` calls `clean_error()` and `clean_warn()`, but look at
      UFACTORY Studio first: do not blindly clear an error you have not read.
- [ ] Motion enabled: `motion_enable(enable=True)` (done in `prepare()`).
- [ ] Correct mode and state. SDK values: mode `0` = position control, mode `1` = servo motion
      (required by `set_servo_cartesian`); state `0` = motion, `3` = pause, `4` = stop. The bridge
      uses mode 0 / state 0 to home, then mode 1 / state 0 to servo.
- [ ] Gripper: it opens/closes on the right-hand index pinch. Keep fingers clear, and test open/close once
      with nothing in the gripper's path before the first run.
- [ ] Tool load / collision sensitivity: the bridge does not set them. Weigh the real tool and
      call `set_tcp_load(weight_kg, [x_mm, y_mm, z_mm])`; `set_collision_sensitivity(value)` takes
      `0` to `5` (see the SDK docstrings in `xarm/wrapper/xarm_api.py`).

## 4. First-run procedure

1. Complete the checklist above. Power on the arm and confirm it connects in UFACTORY Studio.
2. Run vision only: `./run.sh sim --dry-run`. Confirm tracking is stable and the
   target stays inside the expected box.
3. Lower the speed constants listed above (start well below the defaults).
4. Before the first full run, move a **single joint** a few degrees to confirm direction,
   enable state and e-stop behavior: use UFACTORY Studio's jog controls, or a short script such as
   `arm.set_servo_angle(servo_id=1, angle=5, speed=10, relative=True, wait=True)` (see the SDK
   docstring for units). The repo has no such script yet.
5. With the e-stop person in place, start the bridge: `./run.sh sim --ip <lite6-ip>`.
   The arm will home first (`HOMING`), then enter `SERVO`. Be ready to hit the e-stop.
6. Move your hand slowly and only a little. Watch the overlay status; anything other than
   `SERVO` (for example `ERROR <code>/<code>`) means stop and investigate.
7. Increase speed and range gradually across sessions, never within the same run.
8. End with `q` or `Esc` in the video window; `shutdown()` stops the arm and disconnects.

## 5. Going back to the simulator

- Restore the default `--ip 127.0.0.1` line (or just stop passing `--ip`).
- Start the simulator: `./run.sh sim-up`, open `http://127.0.0.1:18333`, then
  `./run.sh sim`.
- Stop it with `./run.sh sim-down`.

## 6. Troubleshooting

| Symptom | Things to check |
|---|---|
| `ConnectionError: Could not connect to the robot at <ip>` | `ping <lite6-ip>`; laptop static IP is in the same subnet; cable/switch; the IP is correct; Studio loads at `http://<lite6-ip>:18333`. |
| Overlay shows `ERROR <code>/<code>` | Read the code in UFACTORY Studio or with `arm.get_err_warn_code(show=True)` and consult the UFACTORY manual. `recover()` retries every `RECOVERY_COOLDOWN_S` = 2 s; stop the run if it keeps failing. |
| Arm does not move but no error | Motion not enabled, wrong mode/state, or the e-stop is engaged. Check Studio, then `prepare()` / `start_servo()` order. |
| Arm stops after a collision or e-stop | Release the e-stop, clear the error in Studio, restart the bridge. Avoid relying on `recover()` for this. |
| Jittery or too-fast motion | Lower `MAX_STEP_MM`, `MAX_ANGLE_STEP_DEG` and `EMA_ALPHA`/rate; improve lighting for tracking. |
| Works in the simulator, differs on the arm | The simulator does not model collisions, payload or controller limits; add the differences you find during the first tests to this table. |
