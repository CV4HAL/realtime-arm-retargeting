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

## Usage

```bash
# 1. Connect to the Lite 6 (or run in UFACTORY simulator first)
# 2. Start the vision pipeline
python src/main.py
```

*(Detailed setup instructions coming as the pipeline is being built.)*

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
