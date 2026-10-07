#!/usr/bin/env bash
# Build MuJoCo XR APK (no Gradle):  ./build.sh   ->  mujocoxr.apk
# Install on a Quest:               adb install -r mujocoxr.apk
set -euo pipefail
cd "$(dirname "$0")"
ROOT=$PWD
SDK=$ROOT/sdk
NDK=$SDK/ndk/27.2.12479018
BT=$SDK/build-tools/34.0.0
ANDROID_JAR=$SDK/platforms/android-32/android.jar
export JAVA_HOME=$ROOT/jdk/lib/jvm
PYTHON=${PYTHON:-$([ -x ../.env/bin/python ] && echo ../.env/bin/python || echo python3)}  # needs mujoco 3.3.1 (+ robosuite/robocasa to export)
export PATH=$JAVA_HOME/bin:$PATH
TOOLCHAIN=(-DCMAKE_TOOLCHAIN_FILE=$NDK/build/cmake/android.toolchain.cmake
           -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29 -DCMAKE_BUILD_TYPE=Release)

# 1. MuJoCo for arm64 (once). Bionic has localtime_r but MuJoCo only uses it under _POSIX_C_SOURCE.
if [ ! -f mujoco-build/lib/libmujoco.so ]; then
  cmake -S mujoco-src -B mujoco-build "${TOOLCHAIN[@]}" -DCMAKE_C_FLAGS="-D_POSIX_C_SOURCE=200809L" \
    -DCMAKE_PLATFORM_NO_VERSIONED_SONAME=ON -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_EXAMPLES=OFF \
    -DMUJOCO_BUILD_TESTS=OFF -DMUJOCO_TEST_PYTHON_UTIL=OFF -DMUJOCO_BUILD_PLUGINS=OFF
  cmake --build mujoco-build --target mujoco -j"$(nproc)"
fi

# 2. app native library
cmake -S app -B app-build "${TOOLCHAIN[@]}" -DMUJOCO_SRC=$ROOT/mujoco-src \
  -DMUJOCO_LIB=$ROOT/mujoco-build/lib/libmujoco.so -DOPENXR_DIR=$ROOT/openxr
cmake --build app-build -j"$(nproc)"

# 3. stage: native libs + scene (all RoboCasa robots, compiled by export_robots.py)
[ -f scenes/robots.mjb ] || "$PYTHON" export_robots.py  # or download robots.mjb from the GitHub release
rm -rf stage && mkdir -p stage/lib/arm64-v8a stage/assets
cp app-build/libmujocoxr.so mujoco-build/lib/libmujoco.so openxr/jni/arm64-v8a/libopenxr_loader.so stage/lib/arm64-v8a/
$NDK/toolchains/llvm/prebuilt/linux-x86_64/bin/llvm-strip --strip-unneeded stage/lib/arm64-v8a/*.so
cp scenes/robots.mjb stage/assets/

# 4. package, align, sign
$BT/aapt2 link -o unaligned.apk --manifest app/AndroidManifest.xml -I "$ANDROID_JAR" -A stage/assets --debug-mode
(cd stage && zip -qr ../unaligned.apk lib)
$BT/zipalign -f -p 4 unaligned.apk aligned.apk
[ -f debug.keystore ] || keytool -genkeypair -keystore debug.keystore -storepass android -keypass android \
  -alias androiddebugkey -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=Android Debug,O=Android,C=US"
$BT/apksigner sign --ks debug.keystore --ks-pass pass:android --out mujocoxr.apk aligned.apk
rm -f unaligned.apk aligned.apk
ls -lh mujocoxr.apk
