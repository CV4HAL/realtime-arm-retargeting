# CV4HAL: Computer Vision for Hybrid Assembly Lines

An operator, a camera, and a UFactory (6Lite, xArm)
that mirrors their movements in real time.

Industrial vision today is an eye it identifies and inspects. This
project turns it into a hand: a low-cost, camera-based teleoperation
interface that lets an operator direct a 6-axis cobot.

> Research project at **FrED Factory** (Tec de Monterrey × MIT), under the
> *Building the Task Force of the Future* mission.
> 

## Project Overview

CV4HAL captures an operator's movements with computer vision (pose
estimation) and maps them to a UFACTORY Lite 6 cobot in real time,
creating an accessible teleoperation interface that needs only a camera;
no motion-capture suit or expensive haptic rig.

The current focus is a robust single-arm system. Extending the approach to
a **line of arms** (one operator directing multiple cobots via a
leader–follower scheme) is the project's next research direction.

## Hardware

| Component        | Model                                  |
| ---------------- | -------------------------------------- |
| Robot arm        | UFACTORY Lite 6 (6-axis cobot)         |
| Camera           | *TBD — to be selected*                 |
| End effector     | *TBD*                                  |
| Compute          | Laptop / mini-PC running the pipeline  |

## Software

- Python 3.x
- [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK) (UFACTORY control)
- OpenCV
- MediaPipe (pose estimation)

Install dependencies:

```bash
pip install -r requirements.txt
```

## Quick start

Everything is launched through `./run.sh` (run `./run.sh --help` for the full list):

```bash
# 1. One-time setup (Python 3.11)
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt

# 2. Check the environment and camera
./run.sh check

# 3. Start the UFACTORY simulator (Docker, see SIMULATOR.md)
./run.sh sim-up

# 4. Run the CV bridge
./run.sh sim --dry-run                  # tracking only, no robot connection
./run.sh sim                            # follow your right wrist in the simulator
./run.sh sim --dual-hand                # left hand also sets the TCP orientation
./run.sh sim --dual-hand --swap-hands   # if the L/R labels are reversed
./run.sh sim --ip 192.168.1.50          # real robot

# Other commands
./run.sh cv --camera 1                  # holistic tracking on camera 1
./run.sh pose                           # basic pose tracking
./run.sh test                           # pytest + ruff
./run.sh sim-down                       # stop the simulator
```

See [SIMULATOR.md](SIMULATOR.md) for the simulator and bridge details.

## Roadmap

- [x] Define target claim, metrics, and repo setup
- [ ] Vision pipeline: real-time operator pose estimation
- [ ] Single-arm teleoperation on the UFACTORY Lite 6
- [ ] Latency & tracking-accuracy characterization
- [ ] *Future:* multi-arm line (leader–follower), e.g. adding an xArm 6
- [ ] *Future:* EMG-based grasp control

## Important Notes

- **Test in the UFACTORY simulator before running on real hardware.**
- **Enable collision detection** and set a conservative sensitivity before
  any live run.
- **Set the correct TCP payload** to avoid false collision triggers.
- Keep the emergency stop within reach during any real-arm session.

## Team

Research team, FrED Factory - Tec de Monterrey:

- Katherine Lucia MacLean - PM
- Santiago Burgueño Ortega
- Carlos Fabián Maldonado Mariño
- Eduardo Mateo Murillio Andrade
- Jasiel Aldana Palacios
- Rodrigo Flores Manríquez
- Armando Javier Flores Salazar

## License

Released under the MIT License — see [LICENSE](LICENSE).

## Acknowledgements

- [FrED Factory](https://fredfactory.mit.edu) — Tec de Monterrey × MIT
- [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
- [MediaPipe](https://github.com/google-ai-edge/mediapipe)
