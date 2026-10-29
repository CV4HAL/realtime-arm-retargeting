# Switching from the Lite 6 simulator to the real arm

This guide explains how to point the CV bridge (`src/sim_bridge.py`) at the physical
UFACTORY Lite 6 instead of the Docker simulator, and how to go back. Commands are given
for both **Linux/macOS** (bash) and **Windows** (PowerShell).

> **Status (28 Oct 2026):** first session on the physical Lite 6. The laptop reached the arm
> over a direct Ethernet cable, UFACTORY Studio loaded, and the bridge drove the arm and the
> suction gripper. See section 7. Speed, workspace and tool load values are still not in this
> repo or the SDK, so each one below says **how to find it** instead of giving a number.

> **Fix these before the next live run:** (1) `shutdown()` never turns off the gripper; on
> 28 Oct the suction kept running after exit until reset was pressed. (2) `recover()`
> re-enables motion on its own every 2 s. (3) No controller-side speed or workspace limits.
> Until then, only run with someone at the e-stop and section 3 fully ticked.

## Command reference

Run all commands from the repository root.

| Task | Linux / macOS (bash) | Windows (PowerShell) |
|---|---|---|
| Check environment | `./run.sh check` | `.venv\Scripts\python src\check_setup.py` |
| Vision only (no robot) | `./run.sh sim --dry-run` | `.venv\Scripts\python src\sim_bridge.py --dry-run` |
| Bridge to simulator | `./run.sh sim` | `.venv\Scripts\python src\sim_bridge.py --ip 127.0.0.1` |
| Bridge to real arm | `./run.sh sim --ip <lite6-ip>` | `.venv\Scripts\python src\sim_bridge.py --ip <lite6-ip>` |
| Ping the arm | `ping -c 3 <lite6-ip>` | `ping <lite6-ip>` |
| Stop simulator | `./run.sh sim-down` | `docker stop uf_software` |

`run.sh` is a bash script, so it doesn't work in PowerShell. On Windows, call the venv's Python directly.

**Windows:** open the repo in VS Code and use a **PowerShell** terminal (`PS C:\...>`), not an
Ubuntu/WSL tab (`user@LAPTOP:/mnt/c/...$`). WSL has its own network, so pings and robot
connections from it can fail. The script is `src/sim_bridge.py`; there is no `arm_connect.py`.

## 1. Prerequisites

- **Cable:** Ethernet straight from the port on the **back of the control box** to the laptop.
  No router or switch needed.
- **Arm IP:** `192.168.0.184` on 28 Oct 2026. Confirm in UFACTORY Studio or on the control box
  each session. Use it wherever `<lite6-ip>` appears.
- **Static IP on the laptop:** set the **Ethernet** adapter to `192.168.0.100` (any unused
  `192.168.0.x` except `.184`). This is a new address, **not your laptop's current IP**: your
  Wi-Fi IP is on another network and Windows won't save an address already in use.

  **Linux:** `nmcli con mod "<ethernet-connection>" ipv4.method manual ipv4.addresses 192.168.0.100/24`, then `nmcli con up "<ethernet-connection>"`

  **Windows:** Settings > Network & Internet > Ethernet > IP assignment > Edit > Manual. Turn on IPv4, enter `192.168.0.100` and subnet mask `255.255.255.0`, leave gateway and DNS empty, and save.

  "No Internet" / "Sin Internet" on Ethernet afterwards is normal. Wi-Fi can stay on.

- **Verify the connection** before running any code:
  ```bash
      ping -c 3 <lite6-ip>    # Linux / macOS
      ping <lite6-ip>         # Windows, from PowerShell (not WSL)
  ```
  and open UFACTORY Studio at `http://<lite6-ip>:18333` (no `< >` brackets). Confirmed working on the real controller on 28 Oct.
- Python environment ready: `./run.sh check`.
- Test the vision side without the robot first: `./run.sh sim --dry-run`.

## 2. How to point the bridge at the real arm

Pass the arm's IP on the command line. Don't change the default `--ip` in the code: if that change gets committed, everyone's simulator runs will try to reach the real arm.

```bash
# Linux / macOS
./run.sh sim --ip <lite6-ip>
```

```powershell

# Windows
.venv\Scripts\python src\sim_bridge.py --ip <lite6-ip>
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
| `sim_bridge.py:254` | `shutdown()` uses `set_state(4)`; it does not replace the physical e-stop.  It never stops the gripper, which is why the suction stayed on on 28 Oct.|

Other references to `127.0.0.1` that are documentation only: `run.sh:45` (help text),
`README.md:67`, and `SIMULATOR.md`.

## 3. Safety checklist (before enabling motors)

Tick every item before the first real run.

- [ ] A person stands at the e-stop (physical button) and can reach it at all times.
- [ ] No hands near the arm while it runs. Use a spot with free space on all sides (the 28 Oct
      setup was cramped).
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
- [ ] End effector: set it and the TCP payload in Studio to match what's mounted (suction on
      28 Oct, pincher planned next). The index pinch fires the gripper, with about a 5 s delay.
      Know how to turn it off in Studio and where the reset button is.

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
6. Face the webcam squarely and move your **whole arm** in slow, large sweeps (the arm follows
shoulder-to-wrist, not your hand). Don't pinch unless you mean to: pinches lock axes and fire
the gripper. Watch the overlay status; anything other than
   `SERVO` (for example `ERROR <code>/<code>`) means stop and investigate.
7. Increase speed and range gradually across sessions, never within the same run.
8. End with `q` or `Esc` in the video window; `shutdown()` stops the arm and disconnects.
9. Check the gripper or suction has stopped. If not, turn it off in Studio or press reset.

## 5. Going back to the simulator

- Restore the default `--ip 127.0.0.1` line (or just stop passing `--ip`).
- Start the simulator: `./run.sh sim-up`, open `http://127.0.0.1:18333`, then
  `./run.sh sim`.
- Stop it with `./run.sh sim-down`.

## 6. Troubleshooting

| Symptom | Things to check |
|---|---|
| `ConnectionError: Could not connect to the robot at <ip>` | `ping <lite6-ip>`; laptop static IP is in the same subnet; cable/switch; the IP is correct; Studio loads at `http://<lite6-ip>:18333`. |
| Windows won't save the Ethernet IP | Address already in use (usually your Wi-Fi IP). Use `192.168.0.100`. |
| Ping "Destination Host Unreachable" | Ethernet is still on another network. Check `ipconfig` and set the static IP. |
| Ping from a `172.x.x.x` address | You're in WSL. Use PowerShell or `ping.exe`. |
| Arm moves much less than the operator | Use whole-arm sweeps; targets are clamped to `BOX_HALF_SIZE_MM`. |
| Arm freezes along one direction | A pinch is locking an axis. Relax the right hand. |
| Shaky forward/back motion | MediaPipe depth is noisy. Face the camera, improve front lighting, try `EMA_ALPHA` 0.15. |
| Wrong direction or big wrist twists | Arm folded or near reach limit. Use more space; try `--swap-hands` if L/R is reversed. |
| Suction keeps running after exit | Known `shutdown()` bug. Turn it off in Studio or press reset. |
| Overlay shows `ERROR <code>/<code>` | Read the code in UFACTORY Studio or with `arm.get_err_warn_code(show=True)` and consult the UFACTORY manual. `recover()` retries every `RECOVERY_COOLDOWN_S` = 2 s; stop the run if it keeps failing. |
| Arm does not move but no error | Motion not enabled, wrong mode/state, or the e-stop is engaged. Check Studio, then `prepare()` / `start_servo()` order. |
| Arm stops after a collision or e-stop | Release the e-stop, clear the error in Studio, restart the bridge. Avoid relying on `recover()` for this. |
| Jittery or too-fast motion | Lower `MAX_STEP_MM`, `MAX_ANGLE_STEP_DEG` and `EMA_ALPHA`/rate; improve lighting for tracking. |
| Works in the simulator, differs on the arm | The simulator does not model collisions, payload or controller limits; add the differences you find during the first tests to this table. |
| `./run.sh` fails on Windows | `run.sh` is bash only. Use the PowerShell commands in the command reference. |
| Ping fails on Windows | Windows may be routing over Wi-Fi. Turn Wi-Fi off and check the Ethernet adapter has the static IP. |


## 7. Session log

| Date | Result |
|---|---|
| 28 Oct 2026 | Connected over direct Ethernet (ping 4/4, 0 to 2 ms); Studio loaded. Bridge drove the arm but range was small and the cell was cramped. Suction worked with about 5 s delay and stayed on after exit until reset. |