cmake_minimum_required(VERSION 3.22)
project(mujocoxr C CXX)

set(CMAKE_CXX_STANDARD 17)
set(GLUE ${ANDROID_NDK}/sources/android/native_app_glue)

add_library(native_app_glue STATIC ${GLUE}/android_native_app_glue.c)

add_library(mujocoxr SHARED main.cpp)
target_include_directories(mujocoxr PRIVATE ${GLUE} ${MUJOCO_SRC}/include ${OPENXR_DIR}/prefab/modules/headers/include)
target_link_libraries(mujocoxr
  native_app_glue
  ${MUJOCO_LIB}
  ${OPENXR_DIR}/jni/arm64-v8a/libopenxr_loader.so
  android log EGL GLESv3)
# keep the NativeActivity entry point from being stripped
target_link_options(mujocoxr PRIVATE -u ANativeActivity_onCreate)
