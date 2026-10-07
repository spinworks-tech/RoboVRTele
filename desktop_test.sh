#!/usr/bin/env bash
# Render the APK's scene on this PC (same renderer code) -> out/front.ppm, out/side.ppm
set -euo pipefail
cd "$(dirname "$0")"
PYTHON=${PYTHON:-$([ -x ../.env/bin/python ] && echo ../.env/bin/python || echo python3)}  # needs mujoco 3.3.1 (+ robosuite/robocasa to export)
SYS=sdk/ndk/27.2.12479018/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include
MJ=$("$PYTHON" -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")
mkdir -p out/glinc && cp -r $SYS/EGL $SYS/GLES3 $SYS/KHR out/glinc/   # Khronos headers only
g++ -O1 -std=c++17 app/desktop_test.cpp -I app -I out/glinc -I mujoco-src/include \
  -I openxr/prefab/modules/headers/include "$MJ"/libmujoco.so.* \
  /usr/lib/x86_64-linux-gnu/libEGL.so.1 /usr/lib/x86_64-linux-gnu/libGLESv2.so.2 \
  -Wl,-rpath,"$MJ" ${CXXFLAGS:-} -o out/desktop_test
[ -f scenes/robots.mjb ] || "$PYTHON" export_robots.py
out/desktop_test scenes/robots.mjb out
