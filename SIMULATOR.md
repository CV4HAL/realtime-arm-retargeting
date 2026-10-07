# UFACTORY Lite 6 Simulator (Docker)

Guide to running the official UFACTORY simulator (UFACTORY Studio + Lite 6
firmware) in Docker, so you can test the pipeline before running against the
real arm.

> **IP note:** every `127.0.0.1` below points at the simulated robot running
> inside this Docker container on your own machine. Once we have the
> physical Lite 6, every one of those `127.0.0.1` occurrences must be
> replaced with the controller's real IP address on the network (check it
> on the robot's control box or in UFACTORY Studio's connection screen). See
> "Switching to the physical robot" at the end of this file.

## Requirements

- Docker Engine installed and running (`docker --version`, `systemctl status docker`).
- Your user in the `docker` group so you don't need `sudo`:

  ```bash
  sudo usermod -aG docker $USER
  ```

  Log out and back in (or run `newgrp docker` / `sg docker -c "<command>"` in
  the current session) for the group change to take effect.

## Starting the simulator (first time)

```bash
# 1. Pull the official UFACTORY image
docker pull danielwang123321/uf-ubuntu-docker

# 2. Create and run the container with the required ports
docker run -d --name uf_software \
  -p 18333:18333 \
  -p 502:502 -p 503:503 -p 504:504 \
  -p 30000:30000 -p 30001:30001 -p 30002:30002 -p 30003:30003 \
  danielwang123321/uf-ubuntu-docker tail -f /dev/null

# 3. Start the firmware and UFACTORY Studio for the Lite 6 (model "6 9")
docker exec -d uf_software /xarm_scripts/xarm_start.sh 6 9
```

Open **http://127.0.0.1:18333** in your browser to access UFACTORY Studio.
If you see "Unable to get robot SN", click "Close" and continue: that's
expected in the simulator.

> `127.0.0.1` here is the simulator running on your own machine. With the
> physical Lite 6, you'll instead open `http://<robot-controller-ip>:18333`
> from a machine on the same network as the robot.

## Starting it again (container already exists)

```bash
docker start uf_software
docker exec -d uf_software /xarm_scripts/xarm_start.sh 6 9
```

## Checking that it's running

```bash
docker ps                       # "uf_software" should show as "Up"
curl -I http://127.0.0.1:18333   # should respond HTTP 200
```

Same as above: `127.0.0.1` is only valid while checking the simulator. Once
the physical robot is in place, run this `curl` against
`http://<robot-controller-ip>:18333` instead.

## Notes

- Port 18333: UFACTORY Studio web interface.
- Ports 502-504 and 30000-30003: SDK/Modbus ports used by xArm-Python-SDK to
  connect to the simulated robot (use `127.0.0.1` as the robot IP in the
  SDK — see the note below, this changes once we're on real hardware).
- The `6 9` parameter in `xarm_start.sh` selects the Lite 6 model.

## Switching to the physical robot

Everything above uses `127.0.0.1` as the robot's IP because the simulator
runs locally in Docker on this same machine. That is **not** the IP of the
physical Lite 6 we'll eventually have.

When the real arm arrives:

1. Find the controller's IP address (shown on the robot's control box, or by
   scanning the network / checking your router's DHCP leases).
2. Replace every `127.0.0.1` used to reach the robot with that IP:
   - In the browser, to open UFACTORY Studio (`http://<robot-ip>:18333`
     instead of `http://127.0.0.1:18333`).
   - In `xArm-Python-SDK` calls, e.g. `XArmAPI('127.0.0.1')` becomes
     `XArmAPI('<robot-ip>')`.
   - In any `curl`/health-check command used to verify connectivity.
3. You do **not** need the Docker container or `xarm_start.sh` for the real
   robot — those only exist to run the simulated firmware. Once you're
   pointed at the physical controller's IP, that traffic goes straight to
   the robot instead of to this container.

## Running the CV bridge

`src/sim_bridge.py` tracks your right arm with MediaPipe and drives the
Lite 6 with Cartesian servo commands. Start the simulator first (see above),
then from the repository root:

```bash
pip install -r requirements.txt

# Tracking only: computes targets, never connects to the robot
python src/sim_bridge.py --dry-run

# Against the simulator (robot homes, then follows your wrist)
python src/sim_bridge.py --ip 127.0.0.1

# Dual-hand mode: right arm moves the TCP, left hand sets its orientation
python src/sim_bridge.py --dual-hand --ip 127.0.0.1
python src/sim_bridge.py --dual-hand --dry-run

# Camera reports the hands the wrong way round: swap left and right
python src/sim_bridge.py --dual-hand --swap-hands --dry-run

# Other camera / servo rate
python src/sim_bridge.py --camera 1 --rate 30
```

With `--dual-hand` the right arm still drives the TCP position and the
gripper value, while the left hand drives the TCP roll, pitch and yaw as
offsets from the home orientation. Tilt and rotate your open left hand in
front of the camera; small movements inside the dead zone are ignored. Close
the left hand into a fist to freeze the orientation, and open it to resume.
If the left hand leaves the frame the last orientation is held. The overlay
shows the roll/pitch/yaw offsets and the clutch state. The angle ranges, dead
zone and fist thresholds are constants at the top of `src/sim_bridge.py`.

Each detected hand is labelled `L` or `R` on its wrist in the mirrored video
window. Raise your left hand and check that the `L` label follows it; if the
labels are reversed, add `--swap-hands`.

Press `q` or `Esc` in the video window to stop the arm, return to position
mode and disconnect. The workspace box and smoothing limits are constants at
the top of `src/sim_bridge.py`. Watch the arm in UFACTORY Studio at
`http://127.0.0.1:18333`.
