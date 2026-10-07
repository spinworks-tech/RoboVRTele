#!/usr/bin/env bash
# One-time setup for build.sh: Android toolchain + sources, all inside this folder (no sudo).
# Needs: conda (for JDK 17), git, curl, unzip, cmake >= 3.22, make.
set -euo pipefail
cd "$(dirname "$0")"

# JDK 17 (sdkmanager / apksigner)
[ -d jdk ] || conda create -y -p ./jdk -c conda-forge openjdk=17
export JAVA_HOME=$PWD/jdk/lib/jvm

# Android SDK: command-line tools, platform 32, build-tools 34, NDK r27c
if [ ! -d sdk/cmdline-tools/latest ]; then
  curl -sSL -o cmdtools.zip https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip
  mkdir -p sdk/cmdline-tools && unzip -q cmdtools.zip -d sdk/cmdline-tools
  mv sdk/cmdline-tools/cmdline-tools sdk/cmdline-tools/latest && rm cmdtools.zip
fi
yes | sdk/cmdline-tools/latest/bin/sdkmanager --sdk_root="$PWD/sdk" --licenses > /dev/null || true
sdk/cmdline-tools/latest/bin/sdkmanager --sdk_root="$PWD/sdk" "platforms;android-32" "build-tools;34.0.0" "ndk;27.2.12479018"

# MuJoCo source (the .mjb scene must be loaded by the same version that wrote it: 3.3.1)
[ -d mujoco-src ] || git clone --depth 1 --branch 3.3.1 https://github.com/google-deepmind/mujoco.git mujoco-src

# Khronos OpenXR loader for Android (works on Quest 2 / 3 / 3S)
if [ ! -d openxr ]; then
  curl -sSL -o openxr_loader.aar \
    https://repo1.maven.org/maven2/org/khronos/openxr/openxr_loader_for_android/1.1.63/openxr_loader_for_android-1.1.63.aar
  mkdir -p openxr && unzip -q openxr_loader.aar -d openxr && rm openxr_loader.aar
fi

# mujoco_menagerie robots used by export_robots.py (pinned commit)
if [ ! -d menagerie ]; then
  git clone --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git menagerie
  git -C menagerie checkout f054586a8e90465d49ee5be15335c4a0c7f57caf
  git -C menagerie sparse-checkout set franka_emika_panda universal_robots_ur5e kinova_gen3 aloha \
    hello_robot_stretch_3 boston_dynamics_spot unitree_h1 google_robot
fi

echo "Setup done. Next: get scenes/robots.mjb (download from the GitHub release, or run export_robots.py), then ./build.sh"
