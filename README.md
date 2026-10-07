# RoboVRTele — MuJoCo XR for Meta Quest 2 / 3 / 3S

A native Quest app: MuJoCo 3.3.1 runs **on the headset** (no PC, no streaming, no Unity), OpenXR
handles tracking and stereo, and a small OpenGL ES 3 renderer draws each eye straight from the
simulation.

The scene: 11 robots on an arc around you (the RoboCasa line-up PandaOmron, GR1, G1, Google Robot,
plus Franka Panda, UR5e, Kinova Gen3, ALOHA, Unitree H1, Spot, Stretch 3 from mujoco_menagerie) and
a table with RoboCasa objects. Point at a robot's hand, grab it and drag it: damped least-squares IK
moves the arm and MuJoCo physics with self-collision keeps it from passing through itself.

## Quick start: install the APK
1. Download `MuJoCoXR-<version>.apk` from the [Releases](../../releases) page.
2. Follow **[OCULUS.md](OCULUS.md)**: a step-by-step guide for first-time Quest developers
   (Developer Mode, USB permissions, `adb install`, launching, troubleshooting).

## Controls
| Input | Action |
|---|---|
| Left stick | Walk / strafe (where you look) |
| Click left stick | Toggle WALK / HEIGHT mode (HEIGHT: stick = up/down, left/right = turn) |
| Left ☰ menu button | Menu: show/hide robots and the table, **SETTINGS >** (joint units, self-collisions, stats panel, RoboCasa items) |
| Right ray | Point at a robot: the link turns cyan; a white ball marks the grabbable hand |
| Right trigger | Grab / release a hand (its IK chain turns orange, joint panel opens top left) |
| Right stick, B / A | Move the grabbed point (forward/back, left/right; up / down) |

Physics runs only for the grabbed robot; all others are animated kinematically with no collision
computation. The app holds 72 fps on a Quest 3S with ~570k triangles.

## Build from source
Linux, conda, git, cmake ≥ 3.22. Everything (JDK, Android SDK/NDK, sources) is installed inside this
folder; no Gradle, no Android Studio.

```bash
./setup.sh      # JDK 17, Android SDK + NDK r27c, MuJoCo 3.3.1 source, OpenXR loader, menagerie robots
# scene: download robots.mjb from the release into scenes/, or export it yourself (see below)
mkdir -p scenes && cp ~/Downloads/robots.mjb scenes/
./build.sh      # -> mujocoxr.apk (signed with a local debug key)
adb install -r mujocoxr.apk
```

`./desktop_test.sh` renders the same scene with the same renderer code on your PC (EGL + GLES 3,
needs a GPU driver and `pip install mujoco==3.3.1`): `out/front.ppm`, `out/side.ppm`, `out/table.ppm`.
It also runs a grab + IK + physics check and prints the tracking error.

### Exporting the scene yourself
`export_robots.py` builds the robots exactly as RoboCasa builds them, so it needs a Python env with
`mujoco==3.3.1`, [robosuite](https://github.com/ARISE-Initiative/robosuite),
`robosuite_models`, [robocasa](https://github.com/robocasa/robocasa) (with its kitchen assets
downloaded), `trimesh`, `fast-simplification` and `pillow`:

```bash
PYTHON=/path/to/env/bin/python ./build.sh   # runs export_robots.py if scenes/robots.mjb is missing
```
Edit `ROBOTS` (line-up, mesh simplification per robot) and `TABLE_OBJECTS` in `export_robots.py`.

## Layout
| Path | What |
|---|---|
| `app/main.cpp` | OpenXR + EGL setup, MuJoCo load, physics/IK, GLES renderer, HUD, menu, input |
| `app/AndroidManifest.xml` | NativeActivity, VR intent category, OpenXR broker queries |
| `app/desktop_test.cpp` | desktop render + IK check using the same code |
| `export_robots.py` | builds the scene (robosuite + menagerie + RoboCasa objects) into `scenes/robots.mjb` |
| `setup.sh` / `build.sh` | toolchain setup / MuJoCo arm64 build, app build, aapt2 + zipalign + apksigner |
| `OCULUS.md` | headset setup, install, controls and troubleshooting |

Tuning knobs (speeds, HUD placement, physics rate, PD stiffness, IK damping) are constants at the
top of `app/main.cpp`.

## Known limits
- Flat shading, no textures/shadows/MSAA; objects use the average colour of their texture.
- Physics only for the grabbed robot (computed-torque PD + gravity compensation, model actuators
  disabled, floor contact off); floating bases (Spot, H1, Stretch) are fixed at their home pose.
- Table objects are static.
- Meshes are simplified for the headset; `robots.mjb` must be re-exported if the MuJoCo version changes.
- Controllers are drawn as simple stand-ins (no Meta controller models or hand tracking yet).
