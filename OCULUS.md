# Running MuJoCo XR on a Meta Quest 2 / 3 / 3S — step by step

This guide assumes no previous Meta development experience. It covers:

1. [What you need](#1-what-you-need)
2. [One-time headset setup (Developer Mode)](#2-one-time-headset-setup-developer-mode)
3. [One-time PC setup (USB permissions)](#3-one-time-pc-setup-usb-permissions)
4. [Connect the headset to the PC](#4-connect-the-headset-to-the-pc)
5. [Install the app](#5-install-the-app)
6. [Start the app and what you should see](#6-start-the-app-and-what-you-should-see)
7. [Watch the app's log (optional, very useful)](#7-watch-the-apps-log-optional-very-useful)
8. [Stop, reinstall, uninstall](#8-stop-reinstall-uninstall)
9. [Troubleshooting](#9-troubleshooting)
10. [Optional: install over Wi-Fi instead of USB](#10-optional-install-over-wi-fi-instead-of-usb)

Steps 2 and 3 are done **once**. After that, every test is just steps 4–6.

Commands in grey boxes are typed in a terminal on the PC. Run them from the `android/` folder:

```bash
cd RoboVRTele   # the folder where you cloned or downloaded this repo
```

---

## 1. What you need

- The Quest headset, **charged**, set up and signed in with your Meta account (the normal first-time setup you did when you got it).
- A smartphone with the **Meta Horizon** app (it used to be called "Oculus" / "Meta Quest" app), signed in with the **same** Meta account, and the headset already paired to it (that pairing happens during normal headset setup).
- A **USB-C cable that carries data**, not just power. The cable that came with the Quest works. If the PC never "sees" the headset in step 4, the cable is the first suspect.
- This PC with `adb` installed (done: Android Debug Bridge 34).

---

## 2. One-time headset setup (Developer Mode)

A Quest only accepts apps from the PC (apps that are not from the Meta store) once **Developer Mode** is switched on. Turning it on needs a free "developer organization" on Meta's website.

### 2a. Create a developer organization (free, ~5 minutes)

1. On any computer, open <https://developers.meta.com/horizon/> and log in with the **same Meta account** used on the headset.
2. Meta asks you to **verify your account** before you can create an organization. It accepts either:
   - adding a phone number and confirming the SMS code, **or**
   - adding a payment method (nothing is charged).
   Follow whatever the page asks for.
3. Go to **"Create new organization"** (or open <https://developers.meta.com/horizon/manage/organizations/create/>).
4. Type any name, for example `Robomotic Lab`, and accept the developer agreement.

That's all you need on the website. You do **not** need to create an "app" there.

### 2b. Turn on Developer Mode

1. Put the headset on, make sure it's on and connected to Wi-Fi, then take it off and keep it nearby.
2. On your phone, open the **Meta Horizon** app.
3. Tap **Devices** (or **Menu → Devices**), then select your headset.
4. Tap **Headset settings** → **Developer mode**.
5. Switch **Developer mode** **ON**.
   - If the switch isn't there, the organization from 2a isn't linked yet: wait a minute, close and reopen the app.
6. **Restart the headset**: hold the power button on the side of the headset → choose **Restart**.

> Newer headset software may also show a **Developer** section inside the headset itself
> (**Settings → Advanced → Developer**). If you see it, that's fine, but the phone app is the
> reliable way to turn Developer Mode on.

---

## 3. One-time PC setup (USB permissions)

Linux needs a rule that lets normal users talk to Meta headsets over USB. Without it, adb
shows the headset as **"no permissions"**. This PC's existing Android rules don't include
Meta devices (USB vendor ID `2833`), so add one:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0666", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/51-meta-quest.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Your user is already in the `plugdev` group, so nothing else is needed. If the headset was
plugged in while you ran this, unplug it and plug it back in.

---

## 4. Connect the headset to the PC

1. Turn the headset on and **put it on your head** (the next prompt only appears inside the headset).
2. Connect the USB-C cable from the headset to the PC.
3. Inside the headset a window appears: **"Allow USB debugging?"**
   - Tick **"Always allow from this computer"** (so you don't have to do this every time).
   - Select **Allow** (point with a controller and press the trigger).
   - If a second popup asks about **"Access data"** / file access, you can choose **Deny**; it isn't needed.
4. On the PC, check the connection:

   ```bash
   adb devices
   ```

   Good result: a serial number followed by the word **`device`**:

   ```
   List of devices attached
   1WMHH8XXXXXXXX    device
   ```

   If it says `unauthorized` or `no permissions`, or nothing is listed, see [Troubleshooting](#9-troubleshooting).

---

## 5. Install the app

The ready-made app file is `mujocoxr.apk` in this folder. (If you change the code, rebuild it
first with `./build.sh`; it takes under a minute after the first time.)

```bash
adb install -r mujocoxr.apk
```

- `-r` means "replace if already installed", so the same command works for updates.
- Good result: the last line is **`Success`**. It takes a few seconds (the file is ~7 MB).

---

## 6. Start the app and what you should see

### Before starting
- Stand in an open area of your play space (**Guardian / boundary** set up as usual). The
  robot appears **1.2 m in front of the center of your play space**, standing on the floor.
- It's fine to leave the USB cable connected, but be careful not to trip on it.

### Option A — start it from inside the headset
1. Press the **Meta button** (the flat button on the right controller, with the Meta logo) to open the menu.
2. Open the **App Library** (the grid icon of nine dots in the bottom menu bar).
3. In the top right of the library there's a filter dropdown (it usually says **All** or
   **All apps**). Change it to **Unknown sources**.
   Apps installed from a PC are always listed there, never in the main list.
4. Select **MuJoCo XR**.

### Option B — start it from the PC (handy while testing)
With the headset on your head:
```bash
adb shell am start -n com.robomotic.mujocoxr/android.app.NativeActivity
```

### What you should see
- A **blue sky** with a sun, and a **grey checkered floor** that fades into the haze at the horizon.
- **Eleven robots on an arc** about 4 m around you, all facing you, left to right: Franka Panda,
  UR5e, Kinova Gen3, PandaOmron, ALOHA, GR1, G1, Unitree H1, Spot, Stretch 3, Google Robot. Their
  joints sway gently around their start pose. Some are beside or slightly behind you: look around.
- Simple **controller stand-ins** where your controllers are, and a thin **ray** from the right one.
- A small **status panel** in the upper right of your view: frames per second, number of
  meshes and triangles, and the current movement mode. It follows your head.
- Walking around and moving your head work normally (it's 3D, one image per eye).
- You won't see hands or controller models; that's expected.

### Moving around
| Do this | Result |
|---|---|
| Walk for real | Works anywhere inside your boundary, like any VR app |
| **Left thumbstick** forward / back | WALK mode: glide in the direction you're **looking** (1.5 m/s at full push) |
| **Left thumbstick** left / right | WALK mode: step sideways (strafe) |
| **Click the left thumbstick** (press it down) | Switch between **WALK** and **HEIGHT** mode; the status panel shows which |
| **Left thumbstick** forward / back in HEIGHT mode | Float **up** / **down** (1 m/s), e.g. to look at the robots from above |
| **Left thumbstick** left / right in HEIGHT mode | **Turn** smoothly (60°/s) around where you stand |
| Turning in WALK mode | Turn your body for real |
| **Menu button** on the left controller (☰, small button below the left stick) | Open / close the **robot menu** |

### Robot menu
Press the **☰ menu button** on the left controller. A panel appears in front of you listing the
robots, each with a tick box (`[X]` shown, `[ ]` hidden):

| In the menu | Result |
|---|---|
| Left stick **down / up** | Move the highlight to the next / previous entry |
| **Left trigger** or **X** button | Show / hide the highlighted robot (or the **Table**), or open **SETTINGS >** |
| **☰ menu button** again | Close the menu (you can walk again) |

The last entry, **SETTINGS >**, opens a second page. Trigger / X changes the highlighted setting:

| Setting | What it does |
|---|---|
| JOINT UNITS: DEGREES / RADIANS | Units in the joint panel |
| SELF-COLLISIONS: ON / OFF | Whether a grabbed robot's links collide with each other (OFF: the arm passes through itself) |
| STATS PANEL: ON / OFF | Show / hide the FPS and mesh counts panel (top right) |
| ROBOCASA ITEMS: SHOWN / HIDDEN | Show / hide everything from RoboCasa at once: PandaOmron, GR1, G1, Google Robot and the table with RoboCasa objects |
| < BACK | Back to the robot list |

While the menu is open the left stick doesn't move you. The status panel shows how many entries
are visible (`ROBOTS 11/12`) and the triangle count of what's shown.

### The table
In the middle, about 1.6 m in front of where you start, a table holds real RoboCasa objects (apple,
mug, bowl, banana, water bottle, croissant, can). They're static for now (part of the scene, not
simulated), and coloured with the average colour of their RoboCasa texture.

### Grabbing a robot arm (inverse kinematics + physics)
| Do this | Result |
|---|---|
| Point the **right controller's ray** at any part of a robot | The **link** you point at turns **cyan** |
| ...at an arm, hand or gripper | A **white ball** marks the hand that would be grabbed and the ray turns yellow |
| **Right trigger** | **Grab** it: a yellow ball marks the hand, an orange ball the target point |
| **Right thumbstick** | Move the target forward/back and left/right (relative to where you look) |
| **B** / **A** buttons (hold) | Move the target **up** / **down** |
| **Right trigger** again (or on another hand) | Release (or switch) |

While grabbed, the links the inverse kinematics is moving turn **orange**, and a **joint panel**
appears top left: every IK joint's position, velocity, acceleration (degrees or radians, see
Settings) and applied torque, plus the hand position and how far it is from the target.

That robot becomes a real MuJoCo simulation: its arm follows the target through
inverse kinematics and **cannot pass through itself** (self-collision). Every other robot stays a
simple animation with no collision computation, so only one robot costs physics at a time. The
status panel shows `GRAB <robot>` and the physics cost (`SIM x.xx MS`). If the target is out of
reach, the arm stops at the closest pose it can reach. H1 has no hands, so you grab its elbows.

Gliding with the stick can make some people queasy at first. If it does, push the stick only
part way (slower) and prefer real walking. The speeds are `MOVE_SPEED` and `HEIGHT_SPEED`
near the top of `app/main.cpp`.

Note: you can glide through the robots and outside your real boundary. Your **body** is still
limited by the real room, so keep an eye on the boundary when walking for real.

If the robots aren't in front of you, **look around and behind you**. To re-center: hold the
**Meta button** on the right controller for about 2 seconds, then restart the app.

---

## 7. Watch the app's log (optional, very useful)

The app prints status messages, including how fast it runs. Keep this running in a second
terminal while the app is open:

```bash
adb logcat -s MuJoCoXR
```

Healthy startup looks like:

```
I MuJoCoXR: GL OpenGL ES 3.2 ...
I MuJoCoXR: OpenXR ready: 1680x1760 per eye, STAGE space
I MuJoCoXR: MuJoCo 3.3.1: nq=122 ngeom=240 nmesh=227 visible tris=298273 robots=5
I MuJoCoXR: session state 1 ... session state 5
I MuJoCoXR: 72.0 fps, kin 0.09 ms/frame | mode WALK move (0.00,0.00) pos (0.0,0.0,0.0)
```

(The resolution differs between Quest 2, 3 and 3S.) The last line repeats every 2 seconds:

- **fps**: frames per second. The Quest's normal rate is 72 (sometimes 90 or 120). Much lower
  means the headset is struggling.
- **sim … ms/frame**: time spent on physics per frame. At 72 fps one frame lasts about 14 ms,
  so physics should stay well below that.

Press **Ctrl+C** to stop watching the log. Please send these lines back after a test.

---

## 8. Stop, reinstall, uninstall

- **Quit the app (in the headset):** press the **Meta button** → in the menu that pops up, choose **Quit**.
- **Quit the app (from the PC):**
  ```bash
  adb shell am force-stop com.robomotic.mujocoxr
  ```
- **Install a new version:** run `./build.sh` and then `adb install -r mujocoxr.apk` again.
- **Uninstall:**
  ```bash
  adb uninstall com.robomotic.mujocoxr
  ```
  (or in the headset: App Library → Unknown sources → the three dots on MuJoCo XR → Uninstall).

---

## 9. Troubleshooting

| What you see | What it means / what to do |
|---|---|
| `adb devices` lists **nothing** | Cable is power-only or loose: try the original Quest cable or another USB port. Make sure Developer Mode is ON (step 2b) and you restarted the headset after turning it on. |
| `no permissions (user in plugdev group...)` | The USB rule from step 3 is missing or not loaded yet. Redo step 3, then unplug/replug. |
| `unauthorized` | You haven't accepted the **Allow USB debugging?** prompt. Put the headset on; the prompt waits there. If you never saw it: unplug, run `adb kill-server`, plug in again with the headset on your head. |
| `adb install` says `INSTALL_FAILED_UPDATE_INCOMPATIBLE` | An older copy was signed differently. Run `adb uninstall com.robomotic.mujocoxr`, then install again. |
| `INSTALL_FAILED_NO_MATCHING_ABIS` | You're not installing on a Quest (e.g. an emulator). The app is built for the Quest's ARM processor only. |
| App isn't in the App Library | Switch the filter at the top right to **Unknown sources** (step 6, option A). |
| App opens and **immediately closes**, or shows a black screen | Run `adb logcat -s MuJoCoXR` and start the app again. Lines with `E MuJoCoXR` explain what failed; send them over. |
| Robot is **under the floor or floating** | The log line `OpenXR ready` will say `LOCAL space` instead of `STAGE space`. Set up your boundary (Guardian) again in the headset's settings and restart the app. |
| Robot is too close / too far | Change `ROBOT_DIST` (meters) near the top of `app/main.cpp`, run `./build.sh`, reinstall. |
| Picture is jumpy / low fps | Send the `fps` and `sim ms/frame` log lines; they tell us whether graphics or physics is the bottleneck. |

To see **everything** the headset logs (very noisy, but useful if the app never even starts):

```bash
adb logcat | grep -iE "mujocoxr|openxr|AndroidRuntime|FATAL"
```

---

## 10. Optional: install over Wi-Fi instead of USB

After the USB setup works once, you can drop the cable. The headset and PC must be on the
same Wi-Fi network.

1. With the headset still connected by USB:
   ```bash
   adb tcpip 5555
   adb shell ip -f inet addr show wlan0 | grep inet
   ```
   The second command prints the headset's Wi-Fi address, e.g. `inet 192.168.68.71/22` → the IP is `192.168.68.71`.
2. Unplug the cable, then:
   ```bash
   adb connect 192.168.68.71:5555
   adb devices
   ```
   You should now see `192.168.68.71:5555    device`. `adb install`, `adb logcat` and the
   start/stop commands all work the same way.
3. After the headset restarts, Wi-Fi debugging switches off again: repeat step 1 with the cable.
