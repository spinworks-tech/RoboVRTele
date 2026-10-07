// MuJoCo XR: standalone Quest 2/3 app. MuJoCo simulates on the headset, OpenGL ES 3 draws
// each eye from the live mjData, OpenXR handles head tracking and the stereo display.
// Built for the headset, or (without __ANDROID__) included by desktop_test.cpp to check rendering.
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#ifdef __ANDROID__
#include <android/asset_manager.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <jni.h>
#include <sys/stat.h>
#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_OPENGL_ES
#endif
#include <openxr/openxr.h>
#ifdef __ANDROID__
#include <openxr/openxr_platform.h>
#endif

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include <vector>

#ifdef __ANDROID__
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "MuJoCoXR", __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, "MuJoCoXR", __VA_ARGS__)
#else
#define LOGI(...) (fprintf(stderr, __VA_ARGS__), fputc('\n', stderr))
#define LOGE LOGI
#endif
#define XR(call)                                                              \
  do {                                                                        \
    XrResult r_ = (call);                                                     \
    if (XR_FAILED(r_)) {                                                      \
      LOGE("%s failed: %d (line %d)", #call, (int)r_, __LINE__);              \
      return false;                                                           \
    }                                                                         \
  } while (0)

// ---- calibration knobs ----------------------------------------------------------
// MuJoCo is z-up; OpenXR is y-up with -z forward. The robot row stands ROBOT_DIST m in front
// of the play-space origin, robots facing the user.
static const float ROBOT_DIST = 2.0f;
static const float LOCAL_FLOOR_Y = -1.5f;   // floor height if STAGE space is unavailable
static const char* SCENE_ASSET = "robots.mjb";  // compiled model in assets/, see export_robots.py
static const double ANIM_AMPLITUDE = 0.2;   // rad: joints sway around the RoboCasa home pose
static const float MOVE_SPEED = 1.5f;       // m/s at full left-stick deflection (WALK mode)
static const float HEIGHT_SPEED = 1.0f;     // m/s up/down at full deflection (HEIGHT mode)
static const float MIN_HEIGHT = -1.0f, MAX_HEIGHT = 4.0f;  // virtual height offset limits (m)
static const float TURN_SPEED = 60.f;       // deg/s, left stick left/right in HEIGHT mode
// physics runs only for the robot whose effector is grabbed (others stay kinematic, no collisions)
static const double PHYS_TIMESTEP = 0.005;  // s (200 Hz); joints are held by soft PD, so this is plenty
static const int PHYS_ITERATIONS = 20;      // constraint solver iterations
static const double PD_HZ = 8.0;            // joint hold stiffness: natural frequency (computed torque)
// grabbing: right controller ray + right trigger; right stick / A,B move the grabbed point
static const float SELECT_RADIUS = 0.15f;   // m: how close the ray must pass to an effector
static const float EE_SPEED = 0.3f;         // m/s at full right-stick deflection
static const double IK_DAMPING = 0.05;      // damped least squares lambda
static const int IK_ITERS = 10;             // IK iterations per frame
static const float STICK_DEADZONE = 0.2f;
// HUD: head-locked panel, upper right of the view
static const float HUD_DIST = 0.7f, HUD_RIGHT = 0.20f, HUD_UP = 0.13f, HUD_WIDTH = 0.26f;  // m
static const int HUD_W = 192, HUD_H = 48;  // texels: 4 lines of 30 chars in a 6x11 cell grid
// robot menu (left controller menu button): world-locked panel placed in front of you when opened
static const float MENU_DIST = 0.8f, MENU_DROP = 0.15f, MENU_WIDTH = 0.5f;  // m
static const int MENU_W = 256, MENU_H = 224;  // texels: up to 20 lines of 41 chars
// joint panel (top left, while a robot is grabbed): joint position / velocity / acceleration / torque
static const float JOINT_LEFT = 0.24f, JOINT_UP = 0.07f, JOINT_WIDTH = 0.32f;  // m, head-locked like the HUD
static const int JOINT_W = 256, JOINT_H = 176;  // texels: 16 lines of 41 chars

// ---- tiny column-major 4x4 math ---------------------------------------------
struct Mat4 { float m[16]; };

static Mat4 identity() {
  Mat4 r{};
  r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1;
  return r;
}

static Mat4 mul(const Mat4& a, const Mat4& b) {
  Mat4 r{};
  for (int c = 0; c < 4; c++)
    for (int row = 0; row < 4; row++) {
      float s = 0;
      for (int k = 0; k < 4; k++) s += a.m[k * 4 + row] * b.m[c * 4 + k];
      r.m[c * 4 + row] = s;
    }
  return r;
}

static Mat4 scale(float x, float y, float z) {
  Mat4 r = identity();
  r.m[0] = x; r.m[5] = y; r.m[10] = z;
  return r;
}

static Mat4 translate(float x, float y, float z) {
  Mat4 r = identity();
  r.m[12] = x; r.m[13] = y; r.m[14] = z;
  return r;
}

// rigid transform from a MuJoCo row-major 3x3 rotation and position
static Mat4 fromMj(const mjtNum* xmat, const mjtNum* xpos) {
  Mat4 r = identity();
  for (int row = 0; row < 3; row++)
    for (int c = 0; c < 3; c++) r.m[c * 4 + row] = (float)xmat[row * 3 + c];
  r.m[12] = (float)xpos[0]; r.m[13] = (float)xpos[1]; r.m[14] = (float)xpos[2];
  return r;
}

// view matrix = inverse of the eye pose
static Mat4 viewFromPose(const XrPosef& p) {
  float x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
  float R[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
                2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
                2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};  // row-major
  Mat4 v = identity();
  for (int row = 0; row < 3; row++)
    for (int c = 0; c < 3; c++) v.m[c * 4 + row] = R[c * 3 + row];  // transpose
  const float t[3] = {p.position.x, p.position.y, p.position.z};
  for (int row = 0; row < 3; row++)
    v.m[12 + row] = -(R[0 * 3 + row] * t[0] + R[1 * 3 + row] * t[1] + R[2 * 3 + row] * t[2]);
  return v;
}

// rigid transform of a pose (tracking space), e.g. the head
static Mat4 poseMatrix(const XrPosef& p) {
  float x = p.orientation.x, y = p.orientation.y, z = p.orientation.z, w = p.orientation.w;
  Mat4 r = identity();
  r.m[0] = 1 - 2 * (y * y + z * z); r.m[4] = 2 * (x * y - w * z);     r.m[8] = 2 * (x * z + w * y);
  r.m[1] = 2 * (x * y + w * z);     r.m[5] = 1 - 2 * (x * x + z * z); r.m[9] = 2 * (y * z - w * x);
  r.m[2] = 2 * (x * z - w * y);     r.m[6] = 2 * (y * z + w * x);     r.m[10] = 1 - 2 * (x * x + y * y);
  r.m[12] = p.position.x; r.m[13] = p.position.y; r.m[14] = p.position.z;
  return r;
}

static Mat4 rotY(float a) {
  Mat4 r = identity();
  r.m[0] = cosf(a); r.m[2] = -sinf(a); r.m[8] = sinf(a); r.m[10] = cosf(a);
  return r;
}

// inverse of a rigid transform (rotation + translation)
static Mat4 rigidInverse(const Mat4& a) {
  Mat4 r = identity();
  for (int row = 0; row < 3; row++)
    for (int c = 0; c < 3; c++) r.m[c * 4 + row] = a.m[row * 4 + c];
  for (int row = 0; row < 3; row++)
    r.m[12 + row] = -(r.m[row] * a.m[12] + r.m[4 + row] * a.m[13] + r.m[8 + row] * a.m[14]);
  return r;
}

static void xformPoint(const Mat4& a, const float* p, mjtNum* out) {
  for (int r = 0; r < 3; r++) out[r] = a.m[r] * p[0] + a.m[4 + r] * p[1] + a.m[8 + r] * p[2] + a.m[12 + r];
}

static void xformDir(const Mat4& a, const float* v, mjtNum* out) {
  for (int r = 0; r < 3; r++) out[r] = a.m[r] * v[0] + a.m[4 + r] * v[1] + a.m[8 + r] * v[2];
}

static Mat4 projection(const XrFovf& fov, float n, float f) {
  float l = tanf(fov.angleLeft), r = tanf(fov.angleRight);
  float u = tanf(fov.angleUp), d = tanf(fov.angleDown);
  Mat4 p{};
  p.m[0] = 2 / (r - l);
  p.m[5] = 2 / (u - d);
  p.m[8] = (r + l) / (r - l);
  p.m[9] = (u + d) / (u - d);
  p.m[10] = -(f + n) / (f - n);
  p.m[11] = -1;
  p.m[14] = -2 * f * n / (f - n);
  return p;
}

// ---- GL resources ------------------------------------------------------------
struct GpuMesh { GLuint vao = 0; GLsizei count = 0; };

static GpuMesh upload(const float* verts, int nvert, const unsigned* idx, int nidx) {
  GpuMesh g;
  GLuint vbo, ebo;
  glGenVertexArrays(1, &g.vao);
  glBindVertexArray(g.vao);
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, nvert * 3 * sizeof(float), verts, GL_STATIC_DRAW);
  glGenBuffers(1, &ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, nidx * sizeof(unsigned), idx, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
  glBindVertexArray(0);
  g.count = nidx;
  return g;
}

// unit primitives: box [-1,1]^3, sphere r=1, cylinder r=1 z in [-1,1], quad [-1,1]^2 at z=0
static GpuMesh makeBox() {
  float v[24];
  for (int i = 0; i < 8; i++) {
    v[i * 3] = (i & 1) ? 1 : -1; v[i * 3 + 1] = (i & 2) ? 1 : -1; v[i * 3 + 2] = (i & 4) ? 1 : -1;
  }
  unsigned f[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                    2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  return upload(v, 8, f, 36);
}

static GpuMesh makeSphere(int nlat = 16, int nlon = 24) {
  std::vector<float> v;
  std::vector<unsigned> f;
  for (int i = 0; i <= nlat; i++) {
    float th = M_PI * i / nlat;
    for (int j = 0; j <= nlon; j++) {
      float ph = 2 * M_PI * j / nlon;
      v.insert(v.end(), {sinf(th) * cosf(ph), sinf(th) * sinf(ph), cosf(th)});
    }
  }
  for (int i = 0; i < nlat; i++)
    for (int j = 0; j < nlon; j++) {
      unsigned a = i * (nlon + 1) + j, b = a + nlon + 1;
      f.insert(f.end(), {a, b, a + 1, a + 1, b, b + 1});
    }
  return upload(v.data(), v.size() / 3, f.data(), f.size());
}

static GpuMesh makeCylinder(int n = 24) {
  std::vector<float> v = {0, 0, 1, 0, 0, -1};  // cap centers
  std::vector<unsigned> f;
  for (int j = 0; j < n; j++) {
    float ph = 2 * M_PI * j / n;
    v.insert(v.end(), {cosf(ph), sinf(ph), 1, cosf(ph), sinf(ph), -1});
  }
  for (int j = 0; j < n; j++) {
    unsigned t0 = 2 + 2 * j, b0 = t0 + 1, t1 = 2 + 2 * ((j + 1) % n), b1 = t1 + 1;
    f.insert(f.end(), {t0, b0, t1, t1, b0, b1, 0, t0, t1, 1, b1, b0});
  }
  return upload(v.data(), v.size() / 3, f.data(), f.size());
}

static GpuMesh makeQuad() {
  float v[12] = {-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0};
  unsigned f[6] = {0, 1, 2, 0, 2, 3};
  return upload(v, 4, f, 6);
}

static const char* VS = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
uniform mat4 uModel;  // geom -> MuJoCo world
out highp vec3 vMj;
void main() {
  vMj = (uModel * vec4(aPos, 1.0)).xyz;
  gl_Position = uMVP * vec4(aPos, 1.0);
})";

// flat shading from screen-space derivatives: no per-vertex normals needed
static const char* FS = R"(#version 300 es
precision highp float;
in highp vec3 vMj;  // MuJoCo world position: shading is anchored to the scene, not the player
uniform vec4 uColor;
uniform int uChecker;
out vec4 oColor;
void main() {
  vec3 n = normalize(cross(dFdx(vMj), dFdy(vMj)));
  vec3 c = uColor.rgb;
  float fog = 0.0;
  if (uChecker == 1) {
    float k = mod(floor(vMj.x * 2.0) + floor(vMj.y * 2.0), 2.0);
    c = mix(vec3(0.46, 0.48, 0.50), vec3(0.38, 0.40, 0.43), k);
    fog = smoothstep(6.0, 45.0, length(vMj.xy));  // fade into the horizon haze
  }
  c = pow(c, vec3(2.2));  // authored colors are sRGB; swapchain is sRGB
  float light = 0.3 + 0.55 * abs(dot(n, normalize(vec3(0.5, 0.3, 1.0))))   // z up
                    + 0.15 * abs(dot(n, normalize(vec3(-1.0, -0.5, 0.3))));
  oColor = vec4(mix(c * light, pow(vec3(0.72, 0.82, 0.92), vec3(2.2)), fog), 1.0);
})";

static const char* HUD_VS = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
out vec2 vUV;
void main() {
  vUV = aPos.xy * 0.5 + 0.5;
  gl_Position = uMVP * vec4(aPos, 1.0);
})";

static const char* HUD_FS = R"(#version 300 es
precision mediump float;
in vec2 vUV;
uniform sampler2D uTex;
out vec4 oColor;
void main() { oColor = texture(uTex, vec2(vUV.x, 1.0 - vUV.y)); })";

// sky dome: unit sphere around the head; color from the view direction (tracking space, y up)
static const char* SKY_VS = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
out vec3 vDir;
void main() {
  vDir = aPos;
  gl_Position = uMVP * vec4(aPos, 1.0);
})";

static const char* SKY_FS = R"(#version 300 es
precision mediump float;
in vec3 vDir;
out vec4 oColor;
void main() {
  vec3 d = normalize(vDir);
  vec3 horizon = vec3(0.72, 0.82, 0.92), zenith = vec3(0.22, 0.45, 0.80), ground = vec3(0.60, 0.66, 0.72);
  vec3 c = d.y > 0.0 ? mix(horizon, zenith, pow(d.y, 0.6)) : mix(horizon, ground, min(-d.y * 4.0, 1.0));
  float sun = max(dot(d, normalize(vec3(0.4, 0.55, -0.75))), 0.0);
  c += vec3(1.0, 0.92, 0.75) * (pow(sun, 400.0) * 1.5 + pow(sun, 12.0) * 0.18);
  oColor = vec4(pow(c, vec3(2.2)), 1.0);
})";

// 5x7 pixel font (bit 4 = leftmost column), generated from readable bit patterns
struct Glyph { char c; unsigned char rows[7]; };
static const Glyph FONT[] = {
    {'0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}},
    {'1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},
    {'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},
    {'3', {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},
    {'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},
    {'5', {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},
    {'6', {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},
    {'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
    {'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},
    {'9', {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},
    {'A', {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'B', {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E}},
    {'C', {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E}},
    {'D', {0x1C,0x12,0x11,0x11,0x11,0x12,0x1C}},
    {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},
    {'F', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10}},
    {'G', {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F}},
    {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},
    {'J', {0x07,0x02,0x02,0x02,0x02,0x12,0x0C}},
    {'K', {0x11,0x12,0x14,0x18,0x14,0x12,0x11}},
    {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'M', {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}},
    {'N', {0x11,0x11,0x19,0x15,0x13,0x11,0x11}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'P', {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},
    {'Q', {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D}},
    {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'S', {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}},
    {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
    {'U', {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'V', {0x11,0x11,0x11,0x11,0x11,0x0A,0x04}},
    {'W', {0x11,0x11,0x11,0x15,0x15,0x15,0x0A}},
    {'X', {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11}},
    {'Y', {0x11,0x11,0x11,0x0A,0x04,0x04,0x04}},
    {'Z', {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F}},
    {'.', {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C}},
    {':', {0x00,0x0C,0x0C,0x00,0x0C,0x0C,0x00}},
    {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
    {'/', {0x01,0x02,0x02,0x04,0x08,0x08,0x10}},
    {'(', {0x02,0x04,0x08,0x08,0x08,0x04,0x02}},
    {'[', {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E}},
    {']', {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E}},
    {'>', {0x08,0x04,0x02,0x01,0x02,0x04,0x08}},
    {')', {0x08,0x04,0x02,0x02,0x02,0x04,0x08}},
};

static const unsigned char* glyph(char c) {
  for (const Glyph& g : FONT)
    if (g.c == c) return g.rows;
  return nullptr;  // space and unknown characters stay blank
}

static GLuint compile(GLenum type, const char* src) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[1024];
    glGetShaderInfoLog(s, sizeof log, nullptr, log);
    LOGE("shader: %s", log);
  }
  return s;
}

// ---- app state -----------------------------------------------------------------
struct App {
#ifdef __ANDROID__
  android_app* android = nullptr;
#endif
  // EGL
  EGLDisplay display = EGL_NO_DISPLAY;
  EGLConfig config = nullptr;
  EGLContext context = EGL_NO_CONTEXT;
  EGLSurface surface = EGL_NO_SURFACE;
  // OpenXR
  XrInstance instance = XR_NULL_HANDLE;
  XrSystemId system = XR_NULL_SYSTEM_ID;
  XrSession session = XR_NULL_HANDLE;
  XrSpace space = XR_NULL_HANDLE;
  XrActionSet actionSet = XR_NULL_HANDLE;
  XrAction moveAction = XR_NULL_HANDLE, modeAction = XR_NULL_HANDLE;
  // virtual locomotion: where the tracking space sits in the world
  float playerX = 0, playerY = 0, playerZ = 0, playerYaw = 0;
  bool heightMode = false;  // left-stick click toggles WALK / HEIGHT
  XrAction menuAction = XR_NULL_HANDLE, selectAction = XR_NULL_HANDLE;
  // right hand: grab (trigger), move grabbed point (stick, A down / B up); controller poses
  XrAction grabAction = XR_NULL_HANDLE, eeMoveAction = XR_NULL_HANDLE, eeUpAction = XR_NULL_HANDLE,
           eeDownAction = XR_NULL_HANDLE, gripPoseAction[2] = {}, aimPoseAction = XR_NULL_HANDLE;
  XrSpace gripSpace[2] = {}, aimSpace = XR_NULL_HANDLE;
  XrPosef handPose[2] = {}, aimPose = {};  // tracking space
  bool handValid[2] = {}, aimValid = false;
  bool menuOpen = false, navLatched = false;
  int menuPage = 0;  // 0: robots, 1: settings
  // settings (menu page 1)
  bool useDegrees = true, collisionsOn = true, statsOn = true;
  std::vector<int> robocasaItems;  // menu entries that are RoboCasa items (model text "robocasa")
  int menuCursor = 0;
  Mat4 menuModel = identity();  // tracking space, set when the menu opens
  XrVector2f lastMove{};    // for the stats log
  double lastFrameT = 0;
  bool stage = false;
  bool running = false, quit = false;
  XrSwapchain swapchain[2] = {};
  int width[2] = {}, height[2] = {};
  std::vector<GLuint> fbo[2];
  // GL
  GLuint prog = 0;
  GLint uMVP = -1, uModel = -1, uColor = -1, uChecker = -1;
  GLuint hudProg = 0, hudTex = 0, menuTex = 0, jointTex = 0, skyProg = 0;
  double jointT = 0;
  GLint hudMVP = -1, skyMVP = -1;
  std::vector<unsigned char> pixels;  // scratch for text panels
  long visibleTris = 0;
  // robots from the model's "robots" text (export_robots.py): names, visibility, geom -> robot
  std::vector<std::string> robotNames;
  std::vector<char> robotShown;
  std::vector<int> geomRobot;  // -1: not part of a robot (floor)
  std::vector<int> bodyRobot, jointRobot;
  std::vector<std::vector<int>> robotJoints;  // hinge/slide joints per robot
  // physics for one robot at a time; collisions of all other geoms switched off
  std::vector<int> contype0, conaffinity0;
  int simRobot = -1;
  double lastSimT = -1;
  std::vector<mjtNum> acc, macc, jacp;
  // grabbable end effectors and IK
  std::vector<int> effBody, effRobot;
  int hoverEff = -1, selEff = -1;
  int hoverBody = -1;          // link under the ray (highlighted)
  std::vector<char> chainBody;  // links moved by IK for the grabbed effector (tinted)
  float hoverDist = 0;
  mjtNum ikTarget[3] = {};
  std::vector<int> ikJoints;
  std::vector<char> ikJoint;    // per joint: held by IK
  std::vector<mjtNum> qTarget;  // per qpos: IK joint targets
  mjData* dIK = nullptr;        // scratch data for IK so it never disturbs the simulation
  GpuMesh box, sphere, cylinder, quad;
  std::vector<GpuMesh> meshes;
  // MuJoCo
  mjModel* m = nullptr;
  mjData* d = nullptr;
  double animStart = -1;  // wall time when the animation clock started
  // stats (log every 2 s, HUD every 0.5 s)
  double statT = 0, simCost = 0, hudT = 0, hudCost = 0;
  int frames = 0, hudFrames = 0;
};

static double now() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return t.tv_sec + t.tv_nsec * 1e-9;
}

#ifdef __ANDROID__
static std::vector<char> readAsset(android_app* app, const char* name) {
  std::vector<char> buf;
  AAsset* a = AAssetManager_open(app->activity->assetManager, name, AASSET_MODE_BUFFER);
  if (!a) {
    LOGE("missing asset %s", name);
    return buf;
  }
  buf.resize(AAsset_getLength(a));
  AAsset_read(a, buf.data(), buf.size());
  AAsset_close(a);
  return buf;
}
#endif

static bool initEgl(App& s) {
  s.display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  eglInitialize(s.display, nullptr, nullptr);
  const EGLint cfg[] = {EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
                        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR, EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                        EGL_NONE};
  EGLint n = 0;
  if (!eglChooseConfig(s.display, cfg, &s.config, 1, &n) || n == 0) {
    LOGE("eglChooseConfig failed");
    return false;
  }
  const EGLint ctx[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
  s.context = eglCreateContext(s.display, s.config, EGL_NO_CONTEXT, ctx);
  const EGLint pb[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
  s.surface = eglCreatePbufferSurface(s.display, s.config, pb);
  if (s.context == EGL_NO_CONTEXT || !eglMakeCurrent(s.display, s.surface, s.surface, s.context)) {
    LOGE("EGL context failed");
    return false;
  }
  LOGI("GL %s / %s", glGetString(GL_VERSION), glGetString(GL_RENDERER));
  return true;
}

#ifdef __ANDROID__
static bool initXr(App& s) {
  PFN_xrInitializeLoaderKHR initLoader = nullptr;
  XR(xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction*)&initLoader));
  XrLoaderInitInfoAndroidKHR li{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
  li.applicationVM = s.android->activity->vm;
  li.applicationContext = s.android->activity->clazz;
  XR(initLoader((XrLoaderInitInfoBaseHeaderKHR*)&li));

  const char* exts[] = {XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME, XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME};
  XrInstanceCreateInfoAndroidKHR ai{XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
  ai.applicationVM = s.android->activity->vm;
  ai.applicationActivity = s.android->activity->clazz;
  XrInstanceCreateInfo ci{XR_TYPE_INSTANCE_CREATE_INFO};
  ci.next = &ai;
  snprintf(ci.applicationInfo.applicationName, XR_MAX_APPLICATION_NAME_SIZE, "MuJoCo XR");
  ci.applicationInfo.applicationVersion = 1;
  ci.applicationInfo.apiVersion = XR_API_VERSION_1_0;
  ci.enabledExtensionCount = 2;
  ci.enabledExtensionNames = exts;
  XR(xrCreateInstance(&ci, &s.instance));

  XrSystemGetInfo sg{XR_TYPE_SYSTEM_GET_INFO};
  sg.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  XR(xrGetSystem(s.instance, &sg, &s.system));

  PFN_xrGetOpenGLESGraphicsRequirementsKHR glReq = nullptr;
  XR(xrGetInstanceProcAddr(s.instance, "xrGetOpenGLESGraphicsRequirementsKHR", (PFN_xrVoidFunction*)&glReq));
  XrGraphicsRequirementsOpenGLESKHR req{XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
  XR(glReq(s.instance, s.system, &req));  // required before xrCreateSession

  XrGraphicsBindingOpenGLESAndroidKHR gb{XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
  gb.display = s.display;
  gb.config = s.config;
  gb.context = s.context;
  XrSessionCreateInfo sc{XR_TYPE_SESSION_CREATE_INFO};
  sc.next = &gb;
  sc.systemId = s.system;
  XR(xrCreateSession(s.instance, &sc, &s.session));

  // left stick = move (WALK) or up/down (HEIGHT); left stick click = toggle mode (Touch / Touch Plus)
  XrActionSetCreateInfo asci{XR_TYPE_ACTION_SET_CREATE_INFO};
  strcpy(asci.actionSetName, "gameplay");
  strcpy(asci.localizedActionSetName, "Gameplay");
  XR(xrCreateActionSet(s.instance, &asci, &s.actionSet));
  struct { const char* name; XrActionType type; XrAction* out; } acts[] = {
      {"move", XR_ACTION_TYPE_VECTOR2F_INPUT, &s.moveAction}, {"mode", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.modeAction},
      {"menu", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.menuAction}, {"select", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.selectAction},
      {"grab", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.grabAction}, {"ee_move", XR_ACTION_TYPE_VECTOR2F_INPUT, &s.eeMoveAction},
      {"ee_up", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.eeUpAction}, {"ee_down", XR_ACTION_TYPE_BOOLEAN_INPUT, &s.eeDownAction},
      {"left_grip", XR_ACTION_TYPE_POSE_INPUT, &s.gripPoseAction[0]},
      {"right_grip", XR_ACTION_TYPE_POSE_INPUT, &s.gripPoseAction[1]},
      {"right_aim", XR_ACTION_TYPE_POSE_INPUT, &s.aimPoseAction}};
  for (auto& a : acts) {
    XrActionCreateInfo ac{XR_TYPE_ACTION_CREATE_INFO};
    ac.actionType = a.type;
    strcpy(ac.actionName, a.name);
    strcpy(ac.localizedActionName, a.name);
    XR(xrCreateAction(s.actionSet, &ac, a.out));
  }
  const char* paths[] = {"/user/hand/left/input/thumbstick", "/user/hand/left/input/thumbstick/click",
                         "/user/hand/left/input/menu/click", "/user/hand/left/input/trigger/value",
                         "/user/hand/left/input/x/click", "/user/hand/right/input/trigger/value",
                         "/user/hand/right/input/thumbstick", "/user/hand/right/input/b/click",
                         "/user/hand/right/input/a/click", "/user/hand/left/input/grip/pose",
                         "/user/hand/right/input/grip/pose", "/user/hand/right/input/aim/pose"};
  XrAction targets[] = {s.moveAction, s.modeAction, s.menuAction, s.selectAction, s.selectAction,
                        s.grabAction, s.eeMoveAction, s.eeUpAction, s.eeDownAction,
                        s.gripPoseAction[0], s.gripPoseAction[1], s.aimPoseAction};
  const int NB = sizeof(targets) / sizeof(targets[0]);
  XrActionSuggestedBinding bindings[NB];
  for (int i = 0; i < NB; i++) {
    bindings[i].action = targets[i];
    XR(xrStringToPath(s.instance, paths[i], &bindings[i].binding));
  }
  XrPath profile;
  XR(xrStringToPath(s.instance, "/interaction_profiles/oculus/touch_controller", &profile));
  XrInteractionProfileSuggestedBinding sb{XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
  sb.interactionProfile = profile;
  sb.suggestedBindings = bindings;
  sb.countSuggestedBindings = NB;
  XR(xrSuggestInteractionProfileBindings(s.instance, &sb));
  XrSessionActionSetsAttachInfo at{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
  at.countActionSets = 1;
  at.actionSets = &s.actionSet;
  XR(xrAttachSessionActionSets(s.session, &at));
  XrSpace* spaces[] = {&s.gripSpace[0], &s.gripSpace[1], &s.aimSpace};
  XrAction poses[] = {s.gripPoseAction[0], s.gripPoseAction[1], s.aimPoseAction};
  for (int i = 0; i < 3; i++) {
    XrActionSpaceCreateInfo asi{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    asi.action = poses[i];
    asi.poseInActionSpace.orientation.w = 1;
    XR(xrCreateActionSpace(s.session, &asi, spaces[i]));
  }

  // floor-level STAGE space if available, else LOCAL (head height at start)
  XrReferenceSpaceCreateInfo rs{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
  rs.poseInReferenceSpace.orientation.w = 1;
  rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_STAGE;
  s.stage = XR_SUCCEEDED(xrCreateReferenceSpace(s.session, &rs, &s.space));
  if (!s.stage) {
    rs.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    XR(xrCreateReferenceSpace(s.session, &rs, &s.space));
  }

  uint32_t nv = 0;
  XR(xrEnumerateViewConfigurationViews(s.instance, s.system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0, &nv, nullptr));
  std::vector<XrViewConfigurationView> views(nv, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
  XR(xrEnumerateViewConfigurationViews(s.instance, s.system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, nv, &nv, views.data()));

  uint32_t nf = 0;
  XR(xrEnumerateSwapchainFormats(s.session, 0, &nf, nullptr));
  std::vector<int64_t> formats(nf);
  XR(xrEnumerateSwapchainFormats(s.session, nf, &nf, formats.data()));
  int64_t format = formats[0];
  for (int64_t f : formats)
    if (f == GL_SRGB8_ALPHA8) format = f;

  for (int eye = 0; eye < 2; eye++) {
    s.width[eye] = views[eye].recommendedImageRectWidth;
    s.height[eye] = views[eye].recommendedImageRectHeight;
    XrSwapchainCreateInfo ci2{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    ci2.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    ci2.format = format;
    ci2.sampleCount = 1;
    ci2.width = s.width[eye];
    ci2.height = s.height[eye];
    ci2.faceCount = ci2.arraySize = ci2.mipCount = 1;
    XR(xrCreateSwapchain(s.session, &ci2, &s.swapchain[eye]));

    uint32_t ni = 0;
    XR(xrEnumerateSwapchainImages(s.swapchain[eye], 0, &ni, nullptr));
    std::vector<XrSwapchainImageOpenGLESKHR> imgs(ni, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR});
    XR(xrEnumerateSwapchainImages(s.swapchain[eye], ni, &ni, (XrSwapchainImageBaseHeader*)imgs.data()));

    GLuint depth;
    glGenRenderbuffers(1, &depth);
    glBindRenderbuffer(GL_RENDERBUFFER, depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, s.width[eye], s.height[eye]);
    s.fbo[eye].resize(ni);
    glGenFramebuffers(ni, s.fbo[eye].data());
    for (uint32_t i = 0; i < ni; i++) {
      glBindFramebuffer(GL_FRAMEBUFFER, s.fbo[eye][i]);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, imgs[i].image, 0);
      glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
    }
  }
  LOGI("OpenXR ready: %dx%d per eye, %s space", s.width[0], s.height[0], s.stage ? "STAGE" : "LOCAL");
  return true;
}
#endif

static GLuint link(const char* vsrc, const char* fsrc) {
  GLuint p = glCreateProgram();
  glAttachShader(p, compile(GL_VERTEX_SHADER, vsrc));
  glAttachShader(p, compile(GL_FRAGMENT_SHADER, fsrc));
  glLinkProgram(p);
  return p;
}

static GLuint makePanelTexture(int w, int h) {
  GLuint t;
  glGenTextures(1, &t);
  glBindTexture(GL_TEXTURE_2D, t);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  return t;
}

// "Name=rootBody,rootBody;Name2=..." (model text "robots") -> which robot each geom belongs to
static void loadRobots(App& s) {
  const mjModel* m = s.m;
  s.geomRobot.assign(m->ngeom, -1);
  int t = mj_name2id(m, mjOBJ_TEXT, "robots");
  if (t < 0) return;
  std::string spec(m->text_data + m->text_adr[t]);
  std::vector<int> bodyRobot(m->nbody, -1);
  size_t pos = 0;
  while (pos < spec.size()) {
    size_t end = spec.find(';', pos);
    if (end == std::string::npos) end = spec.size();
    std::string item = spec.substr(pos, end - pos);
    size_t eq = item.find('=');
    if (eq != std::string::npos) {
      int r = (int)s.robotNames.size();
      s.robotNames.push_back(item.substr(0, eq));
      std::string roots = item.substr(eq + 1) + ",";
      for (size_t a = 0, b; (b = roots.find(',', a)) != std::string::npos; a = b + 1) {
        int id = mj_name2id(m, mjOBJ_BODY, roots.substr(a, b - a).c_str());
        if (id >= 0) bodyRobot[id] = r;
      }
    }
    pos = end + 1;
  }
  s.robotShown.assign(s.robotNames.size(), 1);
  int rc = mj_name2id(m, mjOBJ_TEXT, "robocasa");
  if (rc >= 0) {
    std::string list = std::string(m->text_data + m->text_adr[rc]) + ",";
    for (size_t a = 0, b; (b = list.find(',', a)) != std::string::npos; a = b + 1)
      for (size_t r = 0; r < s.robotNames.size(); r++)
        if (s.robotNames[r] == list.substr(a, b - a)) s.robocasaItems.push_back((int)r);
  }
  s.chainBody.assign(m->nbody, 0);
  for (int i = 0; i < m->ngeom; i++) s.geomRobot[i] = bodyRobot[m->body_rootid[m->geom_bodyid[i]]];
  s.bodyRobot.assign(m->nbody, -1);
  for (int b = 0; b < m->nbody; b++) s.bodyRobot[b] = bodyRobot[m->body_rootid[b]];
  s.jointRobot.assign(m->njnt, -1);
  s.robotJoints.assign(s.robotNames.size(), {});
  for (int j = 0; j < m->njnt; j++) {
    s.jointRobot[j] = s.bodyRobot[m->jnt_bodyid[j]];
    if (s.jointRobot[j] >= 0 && (m->jnt_type[j] == mjJNT_HINGE || m->jnt_type[j] == mjJNT_SLIDE))
      s.robotJoints[s.jointRobot[j]].push_back(j);
  }
  // grabbable effectors: model text "effectors" = comma-separated body names (export_robots.py)
  int e = mj_name2id(m, mjOBJ_TEXT, "effectors");
  if (e >= 0) {
    std::string list = std::string(m->text_data + m->text_adr[e]) + ",";
    for (size_t a = 0, b; (b = list.find(',', a)) != std::string::npos; a = b + 1) {
      int id = mj_name2id(m, mjOBJ_BODY, list.substr(a, b - a).c_str());
      if (id >= 0) {
        s.effBody.push_back(id);
        s.effRobot.push_back(s.bodyRobot[id]);
      }
    }
  }
}

// switch physics + collisions to robot r (or -1: nothing simulated, no collision computation)
static void setSimRobot(App& s, int r) {
  mjModel* m = s.m;
  s.simRobot = r;
  for (int i = 0; i < m->ngeom; i++) {
    bool on = s.collisionsOn && r >= 0 && s.geomRobot[i] == r;  // floor stays off: self-collision is the test
    m->geom_contype[i] = on ? s.contype0[i] : 0;
    m->geom_conaffinity[i] = on ? s.conaffinity0[i] : 0;
  }
  if (r >= 0) {
    mju_zero(s.d->qvel, m->nv);
    mj_forward(m, s.d);  // fresh mass matrix and bias forces for the controller
  }
  s.lastSimT = -1;
}

static bool geomShown(const App& s, int i) {
  return s.m->geom_group[i] <= 2 && (s.geomRobot[i] < 0 || s.robotShown[s.geomRobot[i]]);
}

static void countTris(App& s) {
  s.visibleTris = 0;
  for (int i = 0; i < s.m->ngeom; i++)
    if (geomShown(s, i) && s.m->geom_type[i] == mjGEOM_MESH) s.visibleTris += s.m->mesh_facenum[s.m->geom_dataid[i]];
}

// compiled model (.mjb) from memory, so nothing has to be copied to storage first
static bool initScene(App& s, const std::vector<char>& mjb) {
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, SCENE_ASSET, mjb.data(), (int)mjb.size());
  s.m = mj_loadModel(SCENE_ASSET, &vfs);
  mj_deleteVFS(&vfs);
  if (!s.m) {
    LOGE("mj_loadModel(%s) failed (%zu bytes)", SCENE_ASSET, mjb.size());
    return false;
  }
  s.d = mj_makeData(s.m);  // qpos = qpos0 = home poses (baked by export_robots.py)
  mj_kinematics(s.m, s.d);
  loadRobots(s);
  countTris(s);
  s.m->opt.disableflags |= mjDSBL_ACTUATION;  // joints are driven by the app's PD, not model actuators
  s.m->opt.timestep = PHYS_TIMESTEP;
  s.m->opt.iterations = PHYS_ITERATIONS;
  s.m->opt.ls_iterations = 10;
  s.contype0.assign(s.m->geom_contype, s.m->geom_contype + s.m->ngeom);
  s.conaffinity0.assign(s.m->geom_conaffinity, s.m->geom_conaffinity + s.m->ngeom);
  s.acc.assign(s.m->nv, 0);
  s.macc.assign(s.m->nv, 0);
  s.jacp.assign(3 * s.m->nv, 0);
  s.ikJoint.assign(s.m->njnt, 0);
  s.qTarget.assign(s.m->qpos0, s.m->qpos0 + s.m->nq);
  s.dIK = mj_makeData(s.m);
  setSimRobot(s, -1);
  LOGI("MuJoCo %s: nq=%d ngeom=%d nmesh=%d visible tris=%ld robots=%zu", mj_versionString(), s.m->nq,
       s.m->ngeom, s.m->nmesh, s.visibleTris, s.robotNames.size());

  s.prog = link(VS, FS);
  s.uMVP = glGetUniformLocation(s.prog, "uMVP");
  s.uModel = glGetUniformLocation(s.prog, "uModel");
  s.uColor = glGetUniformLocation(s.prog, "uColor");
  s.uChecker = glGetUniformLocation(s.prog, "uChecker");

  s.hudProg = link(HUD_VS, HUD_FS);
  s.hudMVP = glGetUniformLocation(s.hudProg, "uMVP");
  s.skyProg = link(SKY_VS, SKY_FS);
  s.skyMVP = glGetUniformLocation(s.skyProg, "uMVP");
  s.hudTex = makePanelTexture(HUD_W, HUD_H);
  s.menuTex = makePanelTexture(MENU_W, MENU_H);
  s.jointTex = makePanelTexture(JOINT_W, JOINT_H);

  s.box = makeBox();
  s.sphere = makeSphere();
  s.cylinder = makeCylinder();
  s.quad = makeQuad();
  for (int i = 0; i < s.m->nmesh; i++) {
    int va = s.m->mesh_vertadr[i], fa = s.m->mesh_faceadr[i];
    s.meshes.push_back(upload(s.m->mesh_vert + 3 * va, s.m->mesh_vertnum[i],
                              (const unsigned*)(s.m->mesh_face + 3 * fa), 3 * s.m->mesh_facenum[i]));
  }
  return true;
}

// demo motion: hinge joints sway around the home pose, slide joints stay at home
static double jointTarget(const App& s, int j, double t) {
  const mjModel* m = s.m;
  double q = m->qpos0[m->jnt_qposadr[j]];
  if (m->jnt_type[j] == mjJNT_HINGE) q += ANIM_AMPLITUDE * sin(0.8 * t + j);
  if (m->jnt_limited[j]) q = mju_clip(q, m->jnt_range[2 * j], m->jnt_range[2 * j + 1]);
  return q;
}

// robots that are not simulated are posed kinematically (velocity zero)
static void poseKinematic(App& s, double t) {
  const mjModel* m = s.m;
  for (int j = 0; j < m->njnt; j++) {
    if (s.jointRobot[j] == s.simRobot || (m->jnt_type[j] != mjJNT_HINGE && m->jnt_type[j] != mjJNT_SLIDE)) continue;
    s.d->qpos[m->jnt_qposadr[j]] = jointTarget(s, j, t);
    s.d->qvel[m->jnt_dofadr[j]] = 0;
  }
}

// one frame of motion up to wall time t: everything kinematic, plus real physics (self-collision,
// computed-torque PD with gravity compensation) for the grabbed robot only
static void simulate(App& s, double t) {
  double t0 = now();
  const mjModel* m = s.m;
  mjData* d = s.d;
  if (s.simRobot < 0) {
    poseKinematic(s, t);
    d->time = t;
    mj_kinematics(m, d);
    s.simCost += now() - t0;
    return;
  }
  if (s.lastSimT < 0 || t - s.lastSimT > 0.25) s.lastSimT = t - m->opt.timestep;  // (re)start, no catch-up storm
  const double wn = 2 * M_PI * PD_HZ, kp = wn * wn, kd = 2 * wn;
  const std::vector<int>& joints = s.robotJoints[s.simRobot];
  for (int steps = 0; s.lastSimT + m->opt.timestep <= t && steps < 10; steps++) {
    s.lastSimT += m->opt.timestep;
    poseKinematic(s, s.lastSimT);
    mju_zero(s.acc.data(), m->nv);
    for (int j : joints) {
      int qa = m->jnt_qposadr[j], dof = m->jnt_dofadr[j];
      double target = s.ikJoint[j] ? s.qTarget[qa] : jointTarget(s, j, s.lastSimT);
      s.acc[dof] = kp * (target - d->qpos[qa]) - kd * d->qvel[dof];
    }
    mj_mulM(m, d, s.macc.data(), s.acc.data());  // full mass matrix: stable for every robot (diagonal is not)
    mju_zero(d->qfrc_applied, m->nv);
    for (int j : joints) {
      int dof = m->jnt_dofadr[j];
      d->qfrc_applied[dof] = d->qfrc_bias[dof] + s.macc[dof];
    }
    mj_step(m, d);
  }
  poseKinematic(s, t);
  mj_kinematics(m, d);  // positions for rendering
  s.simCost += now() - t0;
}

// damped least-squares IK (position only) on the grabbed effector's joint chain -> s.qTarget
static void solveIK(App& s) {
  if (s.selEff < 0) return;
  const mjModel* m = s.m;
  mjData* k = s.dIK;
  int body = s.effBody[s.selEff], n = (int)s.ikJoints.size();
  mju_copy(k->qpos, s.d->qpos, m->nq);
  for (int j : s.ikJoints) k->qpos[m->jnt_qposadr[j]] = s.qTarget[m->jnt_qposadr[j]];
  for (int it = 0; it < IK_ITERS; it++) {
    mj_kinematics(m, k);
    mj_comPos(m, k);
    mjtNum err[3];
    mju_sub3(err, s.ikTarget, k->xpos + 3 * body);
    if (mju_norm3(err) < 1e-3) break;
    mj_jacBody(m, k, s.jacp.data(), nullptr, body);
    mjtNum A[9] = {}, y[3];
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++) {
        for (int j : s.ikJoints) A[3 * r + c] += s.jacp[r * m->nv + m->jnt_dofadr[j]] * s.jacp[c * m->nv + m->jnt_dofadr[j]];
        if (r == c) A[3 * r + c] += IK_DAMPING * IK_DAMPING;
      }
    mju_cholFactor(A, 3, 0);
    mju_cholSolve(y, A, err, 3);
    for (int i = 0; i < n; i++) {
      int j = s.ikJoints[i], qa = m->jnt_qposadr[j], dof = m->jnt_dofadr[j];
      mjtNum dq = s.jacp[dof] * y[0] + s.jacp[m->nv + dof] * y[1] + s.jacp[2 * m->nv + dof] * y[2];
      k->qpos[qa] += mju_clip(dq, -0.1, 0.1);
      if (m->jnt_limited[j]) k->qpos[qa] = mju_clip(k->qpos[qa], m->jnt_range[2 * j], m->jnt_range[2 * j + 1]);
    }
  }
  for (int j : s.ikJoints) s.qTarget[m->jnt_qposadr[j]] = k->qpos[m->jnt_qposadr[j]];
}

// grab effector e (its robot starts simulating, its arm follows IK), or release with e = -1 / same e
static void grab(App& s, int e) {
  const mjModel* m = s.m;
  for (int j : s.ikJoints) s.ikJoint[j] = 0;
  s.ikJoints.clear();
  std::fill(s.chainBody.begin(), s.chainBody.end(), 0);
  if (e < 0 || e == s.selEff) {
    s.selEff = -1;
    setSimRobot(s, -1);
    LOGI("released");
    return;
  }
  s.selEff = e;
  setSimRobot(s, s.effRobot[e]);
  // chain: joints from the effector up to the robot root, excluding mobile bases and wheels
  for (int b = s.effBody[e]; b > 0; b = m->body_parentid[b])
    for (int k = 0; k < m->body_jntnum[b]; k++) {
      int j = m->body_jntadr[b] + k;
      const char* name = mj_id2name(m, mjOBJ_JOINT, j);
      if ((m->jnt_type[j] == mjJNT_HINGE || m->jnt_type[j] == mjJNT_SLIDE) &&
          !(name && (strstr(name, "mobile") || strstr(name, "wheel")))) {
        s.ikJoints.push_back(j);
        s.ikJoint[j] = 1;
        s.qTarget[m->jnt_qposadr[j]] = s.d->qpos[m->jnt_qposadr[j]];
      }
    }
  for (int j : s.ikJoints) s.chainBody[m->jnt_bodyid[j]] = 1;
  s.chainBody[s.effBody[e]] = 1;
  mju_copy3(s.ikTarget, s.d->xpos + 3 * s.effBody[e]);
  LOGI("grabbed %s (%s), %zu IK joints", mj_id2name(m, mjOBJ_BODY, s.effBody[e]),
       s.robotNames[s.effRobot[e]].c_str(), s.ikJoints.size());
}

// effector controlling link b: b is the effector or below it (fingers), else the nearest effector
// whose chain passes through b; -1 if none (e.g. a robot base shared by nothing grabbable)
static int effectorForBody(const App& s, int b) {
  const mjModel* m = s.m;
  int best = -1, bestScore = 1 << 30;
  for (size_t e = 0; e < s.effBody.size(); e++) {
    for (int x = b; x > 0; x = m->body_parentid[x])
      if (x == s.effBody[e]) return (int)e;
    int k = 0;
    for (int x = s.effBody[e]; x > 0; x = m->body_parentid[x], k++)
      if (x == b && k < bestScore) {
        bestScore = k;
        best = (int)e;
      }
  }
  return best;
}

// effector closest to the ray (MuJoCo world), within SELECT_RADIUS; -1 if none
static int pickEffector(App& s, const mjtNum* o, const mjtNum* dir, float* along) {
  int best = -1;
  mjtNum bestDist = SELECT_RADIUS;
  for (size_t e = 0; e < s.effBody.size(); e++) {
    if (!s.robotShown[s.effRobot[e]]) continue;
    mjtNum v[3];
    mju_sub3(v, s.d->xpos + 3 * s.effBody[e], o);
    mjtNum t = mju_dot3(v, dir);
    if (t <= 0) continue;
    mjtNum perp[3] = {v[0] - t * dir[0], v[1] - t * dir[1], v[2] - t * dir[2]};
    mjtNum dist = mju_norm3(perp);
    if (dist < bestDist) {
      bestDist = dist;
      best = (int)e;
      *along = (float)t;
    }
  }
  return best;
}

// world: geom -> MuJoCo world; vp: MuJoCo world -> clip (projection * view * placement)
static void draw(App& s, const GpuMesh& g, const Mat4& world, const Mat4& vp, const float* rgba, int checker = 0) {
  Mat4 mvp = mul(vp, world);
  glUniformMatrix4fv(s.uMVP, 1, GL_FALSE, mvp.m);
  glUniformMatrix4fv(s.uModel, 1, GL_FALSE, world.m);
  glUniform4fv(s.uColor, 1, rgba);
  glUniform1i(s.uChecker, checker);
  glBindVertexArray(g.vao);
  glDrawElements(GL_TRIANGLES, g.count, GL_UNSIGNED_INT, nullptr);
}

static void renderEye(App& s, const XrView& view, const Mat4& placement) {
  Mat4 vpTracking = mul(projection(view.fov, 0.05f, 100.f), viewFromPose(view.pose));
  Mat4 vp = mul(vpTracking, placement);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // sky dome centred on the eye, drawn behind everything
  const XrVector3f& e = view.pose.position;
  Mat4 sky = mul(vpTracking, mul(translate(e.x, e.y, e.z), mul(rotY(-s.playerYaw), scale(50, 50, 50))));
  glDisable(GL_DEPTH_TEST);
  glUseProgram(s.skyProg);
  glUniformMatrix4fv(s.skyMVP, 1, GL_FALSE, sky.m);
  glBindVertexArray(s.sphere.vao);
  glDrawElements(GL_TRIANGLES, s.sphere.count, GL_UNSIGNED_INT, nullptr);

  glEnable(GL_DEPTH_TEST);  // no culling: mesh winding is not guaranteed
  glUseProgram(s.prog);

  const mjModel* m = s.m;
  for (int i = 0; i < m->ngeom; i++) {
    if (!geomShown(s, i)) continue;  // collision group 3+ (like MuJoCo's viewer) or robot hidden in menu
    float rgba[4];
    const float* src = m->geom_matid[i] >= 0 ? m->mat_rgba + 4 * m->geom_matid[i] : m->geom_rgba + 4 * i;
    for (int k = 0; k < 4; k++) rgba[k] = src[k];
    if (rgba[3] == 0) continue;
    int b = m->geom_bodyid[i];
    if (b == s.hoverBody || s.chainBody[b]) {  // pointed-at link: cyan; links IK is moving: orange
      const float tint[3] = {b == s.hoverBody ? 0.2f : 1.0f, b == s.hoverBody ? 0.9f : 0.55f, b == s.hoverBody ? 1.0f : 0.1f};
      for (int k = 0; k < 3; k++) rgba[k] = 0.4f * rgba[k] + 0.6f * tint[k];
    }
    Mat4 world = fromMj(s.d->geom_xmat + 9 * i, s.d->geom_xpos + 3 * i);
    const mjtNum* sz = m->geom_size + 3 * i;
    switch (m->geom_type[i]) {
      case mjGEOM_MESH:
        draw(s, s.meshes[m->geom_dataid[i]], world, vp, rgba);
        break;
      case mjGEOM_PLANE:
        draw(s, s.quad, mul(world, scale(sz[0] > 0 ? sz[0] : 5, sz[1] > 0 ? sz[1] : 5, 1)), vp, rgba, 1);
        break;
      case mjGEOM_BOX:
        draw(s, s.box, mul(world, scale(sz[0], sz[1], sz[2])), vp, rgba);
        break;
      case mjGEOM_SPHERE:
        draw(s, s.sphere, mul(world, scale(sz[0], sz[0], sz[0])), vp, rgba);
        break;
      case mjGEOM_ELLIPSOID:
        draw(s, s.sphere, mul(world, scale(sz[0], sz[1], sz[2])), vp, rgba);
        break;
      case mjGEOM_CYLINDER:
        draw(s, s.cylinder, mul(world, scale(sz[0], sz[0], sz[1])), vp, rgba);
        break;
      case mjGEOM_CAPSULE:
        draw(s, s.cylinder, mul(world, scale(sz[0], sz[0], sz[1])), vp, rgba);
        draw(s, s.sphere, mul(world, mul(translate(0, 0, sz[1]), scale(sz[0], sz[0], sz[0]))), vp, rgba);
        draw(s, s.sphere, mul(world, mul(translate(0, 0, -sz[1]), scale(sz[0], sz[0], sz[0]))), vp, rgba);
        break;
      default:
        break;  // hfield, sdf: not drawn
    }
  }

  // controllers (simple stand-ins), pointing ray, effector markers
  Mat4 inv = rigidInverse(placement);  // tracking space -> MuJoCo world
  const float dark[4] = {0.12f, 0.12f, 0.14f, 1}, light[4] = {0.85f, 0.85f, 0.88f, 1};
  for (int h = 0; h < 2; h++) {
    if (!s.handValid[h]) continue;
    Mat4 g = mul(inv, poseMatrix(s.handPose[h]));
    draw(s, s.cylinder, mul(g, mul(translate(0, -0.01f, 0.03f), scale(0.018f, 0.018f, 0.05f))), vp, dark);
    draw(s, s.sphere, mul(g, mul(translate(0, 0.015f, -0.035f), scale(0.05f, 0.012f, 0.05f))), vp, light);
  }
  if (s.aimValid) {
    float len = (s.hoverEff >= 0 || s.hoverBody >= 0) ? s.hoverDist : 3.0f;
    const float ray[4] = {0.3f, 0.9f, 1, 1}, rayHit[4] = {1, 0.85f, 0.2f, 1};
    draw(s, s.cylinder, mul(inv, mul(poseMatrix(s.aimPose), mul(translate(0, 0, -len / 2), scale(0.003f, 0.003f, len / 2)))),
         vp, s.hoverEff >= 0 ? rayHit : ray);
  }
  auto marker = [&](const mjtNum* p, float r, const float* rgba) {
    draw(s, s.sphere, mul(translate((float)p[0], (float)p[1], (float)p[2]), scale(r, r, r)), vp, rgba);
  };
  const float hoverCol[4] = {1, 1, 1, 1}, selCol[4] = {1, 0.85f, 0.1f, 1}, tgtCol[4] = {1, 0.4f, 0.1f, 1};
  if (s.hoverEff >= 0 && s.hoverEff != s.selEff) marker(s.d->xpos + 3 * s.effBody[s.hoverEff], 0.04f, hoverCol);
  if (s.selEff >= 0) {
    marker(s.d->xpos + 3 * s.effBody[s.selEff], 0.035f, selCol);
    marker(s.ikTarget, 0.025f, tgtCol);
  }
  glBindVertexArray(0);
}

// rasterize text lines into a panel texture: white 5x7 glyphs on a translucent panel,
// optional highlighted row (menu cursor)
static void panelText(App& s, GLuint tex, int w, int h, const std::vector<std::string>& lines, int highlight = -1) {
  std::vector<unsigned char>& px = s.pixels;
  px.assign(w * h * 4, 0);
  for (int y = 0; y < h; y++) {
    bool hl = highlight >= 0 && y >= 1 + 11 * highlight && y < 12 + 11 * highlight;
    for (int x = 0; x < w; x++) {
      unsigned char* p = &px[4 * (y * w + x)];
      p[0] = hl ? 40 : 15; p[1] = hl ? 80 : 18; p[2] = hl ? 140 : 26; p[3] = hl ? 235 : 200;
    }
  }
  for (size_t l = 0; l < lines.size(); l++)
    for (size_t k = 0; k < lines[l].size(); k++) {
      const unsigned char* g = glyph((char)toupper(lines[l][k]));
      if (!g) continue;
      int x0 = 4 + 6 * (int)k, y0 = 3 + 11 * (int)l;
      for (int r = 0; r < 7; r++)
        for (int c = 0; c < 5; c++)
          if ((g[r] >> (4 - c)) & 1 && x0 + c < w && y0 + r < h)
            for (int ch = 0; ch < 4; ch++) px[4 * ((y0 + r) * w + x0 + c) + ch] = 255;
    }
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
}

static void hudText(App& s, const std::vector<std::string>& lines) { panelText(s, s.hudTex, HUD_W, HUD_H, lines); }

// textured quad on top of the scene; model: unit quad [-1,1]^2 -> tracking space
static void drawPanel(App& s, GLuint tex, const Mat4& vpTracking, const Mat4& model) {
  Mat4 mvp = mul(vpTracking, model);
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glUseProgram(s.hudProg);
  glUniformMatrix4fv(s.hudMVP, 1, GL_FALSE, mvp.m);
  glBindTexture(GL_TEXTURE_2D, tex);
  glBindVertexArray(s.quad.vao);
  glDrawElements(GL_TRIANGLES, s.quad.count, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  glDisable(GL_BLEND);
  glEnable(GL_DEPTH_TEST);
}

// head-locked HUD, upper right; head: head pose in tracking space
static void drawHud(App& s, const Mat4& vpTracking, const Mat4& head) {
  float hw = HUD_WIDTH / 2, hh = hw * HUD_H / HUD_W;
  drawPanel(s, s.hudTex, vpTracking, mul(head, mul(translate(HUD_RIGHT, HUD_UP, -HUD_DIST), scale(hw, hh, 1))));
}

static int menuItems(const App& s) { return s.menuPage == 0 ? (int)s.robotNames.size() + 1 : 5; }

static bool robocasaShown(const App& s) {
  for (int r : s.robocasaItems)
    if (!s.robotShown[r]) return false;
  return !s.robocasaItems.empty();
}

static void menuText(App& s) {
  std::vector<std::string> items;
  if (s.menuPage == 0) {
    for (size_t r = 0; r < s.robotNames.size(); r++) items.push_back((s.robotShown[r] ? "[X] " : "[ ] ") + s.robotNames[r]);
    items.push_back("SETTINGS >");
  } else {
    items = {std::string("JOINT UNITS: ") + (s.useDegrees ? "DEGREES" : "RADIANS"),
             std::string("SELF-COLLISIONS: ") + (s.collisionsOn ? "ON" : "OFF"),
             std::string("STATS PANEL: ") + (s.statsOn ? "ON" : "OFF"),
             std::string("ROBOCASA ITEMS: ") + (robocasaShown(s) ? "SHOWN" : "HIDDEN"), "< BACK"};
  }
  std::vector<std::string> lines = {s.menuPage == 0 ? "ROBOTS" : "SETTINGS"};
  for (size_t i = 0; i < items.size(); i++) lines.push_back(((int)i == s.menuCursor ? "> " : "  ") + items[i]);
  lines.push_back("");
  lines.push_back("STICK UP/DOWN: CHOOSE");
  lines.push_back(s.menuPage == 0 ? "TRIGGER OR X: SHOW/HIDE, OPEN" : "TRIGGER OR X: CHANGE");
  lines.push_back("MENU BUTTON: CLOSE");
  panelText(s, s.menuTex, MENU_W, MENU_H, lines, 1 + s.menuCursor);
}

static void setShown(App& s, int r, bool shown) {
  if (!shown && s.selEff >= 0 && s.effRobot[s.selEff] == r) grab(s, -1);  // hiding the grabbed robot
  s.robotShown[r] = shown;
}

static void menuSelect(App& s) {
  int n = (int)s.robotNames.size();
  if (s.menuPage == 0) {
    if (s.menuCursor < n) {
      setShown(s, s.menuCursor, !s.robotShown[s.menuCursor]);
      LOGI("%s %s", s.robotNames[s.menuCursor].c_str(), s.robotShown[s.menuCursor] ? "shown" : "hidden");
    } else {
      s.menuPage = 1;
      s.menuCursor = 0;
    }
  } else {
    switch (s.menuCursor) {
      case 0: s.useDegrees = !s.useDegrees; break;
      case 1:
        s.collisionsOn = !s.collisionsOn;
        if (s.simRobot >= 0) setSimRobot(s, s.simRobot);  // apply to the robot being simulated now
        break;
      case 2: s.statsOn = !s.statsOn; break;
      case 3: {
        bool show = !robocasaShown(s);
        for (int r : s.robocasaItems) setShown(s, r, show);
        break;
      }
      default:
        s.menuPage = 0;
        s.menuCursor = n;
    }
  }
  countTris(s);
  menuText(s);
}

// joint panel: per IK joint position, velocity, acceleration (deg or rad) and applied torque
static void jointText(App& s) {
  if (s.selEff < 0) return;
  const mjModel* m = s.m;
  const mjData* d = s.d;
  double k = s.useDegrees ? 180 / M_PI : 1;
  std::vector<std::string> lines;
  char buf[96];
  snprintf(buf, sizeof buf, "%s  IK JOINTS (%s)", s.robotNames[s.effRobot[s.selEff]].c_str(), s.useDegrees ? "DEG" : "RAD");
  lines.push_back(buf);
  lines.push_back("JOINT        POS    VEL    ACC    TRQ");
  size_t nj = std::min<size_t>(s.ikJoints.size(), 11);
  for (size_t i = 0; i < nj; i++) {
    int j = s.ikJoints[s.ikJoints.size() - 1 - i];  // chain is stored hand-first: list base-first
    int qa = m->jnt_qposadr[j], dof = m->jnt_dofadr[j];
    const char* full = mj_id2name(m, mjOBJ_JOINT, j);
    std::string name = full ? full : "?";
    size_t us = name.find('_');  // drop the robot prefix (robot1_, m4_, gripper2_ ...)
    if (us != std::string::npos && us > 0 && us + 1 < name.size() && isdigit((unsigned char)name[us - 1])) name = name.substr(us + 1);
    double f = m->jnt_type[j] == mjJNT_HINGE ? k : 1;  // slide joints stay in m, m/s
    snprintf(buf, sizeof buf, "%-11.11s%7.*f%7.*f%7.0f%7.1f", name.c_str(), s.useDegrees ? 1 : 3, d->qpos[qa] * f,
             s.useDegrees ? 1 : 2, d->qvel[dof] * f, d->qacc[dof] * f, d->qfrc_applied[dof]);
    lines.push_back(buf);
  }
  const mjtNum* p = d->xpos + 3 * s.effBody[s.selEff];
  snprintf(buf, sizeof buf, "HAND %.2f %.2f %.2f M", p[0], p[1], p[2]);
  lines.push_back(buf);
  snprintf(buf, sizeof buf, "TARGET ERROR %.3f M  CONTACTS %d", mju_dist3(p, s.ikTarget), d->ncon);
  lines.push_back(buf);
  lines.push_back("TRQ: NM (SLIDE: N)  POS/VEL SLIDE: M");
  panelText(s, s.jointTex, JOINT_W, JOINT_H, lines);
}

static void drawJointPanel(App& s, const Mat4& vpTracking, const Mat4& head) {
  if (s.selEff < 0) return;
  float hw = JOINT_WIDTH / 2, hh = hw * JOINT_H / JOINT_W;
  drawPanel(s, s.jointTex, vpTracking, mul(head, mul(translate(-JOINT_LEFT, JOINT_UP, -HUD_DIST), scale(hw, hh, 1))));
}

// world-locked menu: placed in front of the head (yaw only), facing the user
static void openMenu(App& s, const XrPosef& head) {
  const XrQuaternionf& q = head.orientation;
  float fx = -2 * (q.x * q.z + q.w * q.y), fz = -(1 - 2 * (q.x * q.x + q.y * q.y));
  float len = sqrtf(fx * fx + fz * fz);
  if (len < 1e-3f) { fx = 0; fz = -1; } else { fx /= len; fz /= len; }
  Mat4 r = identity();  // columns: right, up, toward the user
  r.m[0] = -fz; r.m[2] = fx;
  r.m[8] = -fx; r.m[10] = -fz;
  r.m[12] = head.position.x + MENU_DIST * fx;
  r.m[13] = head.position.y - MENU_DROP;
  r.m[14] = head.position.z + MENU_DIST * fz;
  float hw = MENU_WIDTH / 2, hh = hw * MENU_H / MENU_W;
  s.menuModel = mul(r, scale(hw, hh, 1));
  s.menuOpen = true;
  menuText(s);
}

static std::vector<std::string> hudLines(const App& s, float fps, float kinMs) {
  char l0[64], l1[64], l2[64], l3[64];
  snprintf(l0, sizeof l0, "FPS %.1f  SIM %.2f MS", fps, kinMs);
  snprintf(l1, sizeof l1, "MESHES %d  GEOMS %d", s.m->nmesh, s.m->ngeom);
  int shown = 0;
  for (char v : s.robotShown) shown += v;
  snprintf(l2, sizeof l2, "TRIS %ld  ROBOTS %d/%zu", s.visibleTris, shown, s.robotNames.size());
  if (s.selEff >= 0)
    snprintf(l3, sizeof l3, "GRAB %s (R STICK A/B)", s.robotNames[s.effRobot[s.selEff]].c_str());
  else
    snprintf(l3, sizeof l3, "MODE %s  MENU: L MENU BTN", s.heightMode ? "HEIGHT" : "WALK");
  return {l0, l1, l2, l3};
}

// MuJoCo (x fwd, y left, z up) -> OpenXR (y up, -z fwd): x->+z, y->+x, z->+y
static Mat4 scenePlacement(bool stage) {
  Mat4 p = identity();
  p.m[0] = 0; p.m[1] = 0; p.m[2] = 1;   // mj x -> xr +z
  p.m[4] = 1; p.m[5] = 0; p.m[6] = 0;   // mj y -> xr +x
  p.m[8] = 0; p.m[9] = 1; p.m[10] = 0;  // mj z -> xr +y
  return mul(translate(0, stage ? 0 : LOCAL_FLOOR_Y, -ROBOT_DIST), p);
}

#ifdef __ANDROID__
static XrVector2f stick(App& s, XrAction action) {
  XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
  gi.action = action;
  XrActionStateVector2f st{XR_TYPE_ACTION_STATE_VECTOR2F};
  if (XR_FAILED(xrGetActionStateVector2f(s.session, &gi, &st)) || !st.isActive) return {0, 0};
  return st.currentState;
}

static bool clicked(App& s, XrAction action) {
  XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
  gi.action = action;
  XrActionStateBoolean st{XR_TYPE_ACTION_STATE_BOOLEAN};
  return XR_SUCCEEDED(xrGetActionStateBoolean(s.session, &gi, &st)) && st.isActive && st.changedSinceLastSync &&
         st.currentState;
}

static bool held(App& s, XrAction action) {
  XrActionStateGetInfo gi{XR_TYPE_ACTION_STATE_GET_INFO};
  gi.action = action;
  XrActionStateBoolean st{XR_TYPE_ACTION_STATE_BOOLEAN};
  return XR_SUCCEEDED(xrGetActionStateBoolean(s.session, &gi, &st)) && st.isActive && st.currentState;
}

// rotate a horizontal (x, z) vector by the player's yaw, tracking space -> world
static void yawRotate(float yaw, float& x, float& z) {
  float c = cosf(yaw), sn = sinf(yaw), x0 = x;
  x = c * x0 + sn * z;
  z = -sn * x0 + c * z;
}

static Mat4 playerInverse(const App& s);

// right hand: ray picks an effector, trigger grabs/releases, stick + A/B move the grabbed point
static void handleGrab(App& s, const XrView* views, float dt) {
  Mat4 inv = rigidInverse(mul(playerInverse(s), scenePlacement(s.stage)));  // tracking -> MuJoCo world
  s.hoverEff = -1;
  s.hoverBody = -1;
  if (s.aimValid) {
    float o[3] = {s.aimPose.position.x, s.aimPose.position.y, s.aimPose.position.z};
    Mat4 a = poseMatrix(s.aimPose);
    float fwd[3] = {-a.m[8], -a.m[9], -a.m[10]};  // aim pose looks along -z
    mjtNum om[3], dm[3];
    xformPoint(inv, o, om);
    xformDir(inv, fwd, dm);
    mju_normalize3(dm);
    const mjtByte groups[mjNGROUP] = {1, 1, 1, 0, 0, 0};  // visual geoms only
    int gid = -1;
    mjtNum dist = mj_ray(s.m, s.d, om, dm, groups, 1, -1, &gid);
    if (gid >= 0 && geomShown(s, gid) && s.geomRobot[gid] >= 0) {
      s.hoverBody = s.m->geom_bodyid[gid];
      s.hoverDist = (float)dist;
      s.hoverEff = effectorForBody(s, s.hoverBody);
      if (s.hoverEff >= 0 && !s.robotShown[s.effRobot[s.hoverEff]]) s.hoverEff = -1;
    } else {
      s.hoverEff = pickEffector(s, om, dm, &s.hoverDist);  // ray missed: forgiving radius around hands
    }
  }
  if (clicked(s, s.grabAction)) grab(s, s.hoverEff >= 0 ? s.hoverEff : -1);
  if (s.selEff < 0) return;
  // move the target relative to where the user looks: stick = forward/back + left/right, A/B = down/up
  XrVector2f st = stick(s, s.eeMoveAction);
  if (fabsf(st.x) < STICK_DEADZONE) st.x = 0;
  if (fabsf(st.y) < STICK_DEADZONE) st.y = 0;
  const XrQuaternionf& q = views[0].pose.orientation;
  float fx = -2 * (q.x * q.z + q.w * q.y), fz = -(1 - 2 * (q.x * q.x + q.y * q.y));
  float len = sqrtf(fx * fx + fz * fz);
  if (len < 1e-3f) return;
  fx /= len; fz /= len;
  float wx = fx * st.y - fz * st.x, wz = fz * st.y + fx * st.x;  // tracking-space horizontal motion
  float up = (held(s, s.eeUpAction) ? 1.f : 0.f) - (held(s, s.eeDownAction) ? 1.f : 0.f);
  float v[3] = {wx, up, wz};
  mjtNum vm[3];
  xformDir(inv, v, vm);
  for (int i = 0; i < 3; i++) s.ikTarget[i] += vm[i] * EE_SPEED * dt;
}

// WALK: left stick walks along the head's horizontal facing.
// HEIGHT: stick forward/back = up/down, left/right = turn (around the head).
static void locomote(App& s, const XrView* views, float dt) {
  XrActiveActionSet active{s.actionSet, XR_NULL_PATH};
  XrActionsSyncInfo si{XR_TYPE_ACTIONS_SYNC_INFO};
  si.countActiveActionSets = 1;
  si.activeActionSets = &active;
  if (xrSyncActions(s.session, &si) != XR_SUCCESS) return;  // e.g. not focused (system menu open)

  if (clicked(s, s.menuAction)) {
    if (s.menuOpen) s.menuOpen = false;
    else openMenu(s, views[0].pose);
  }
  if (s.menuOpen) {  // stick chooses a robot, trigger/X toggles it; no locomotion meanwhile
    float y = stick(s, s.moveAction).y;
    int n = menuItems(s);
    if (fabsf(y) > 0.5f && !s.navLatched) {
      s.menuCursor = (s.menuCursor + (y < 0 ? 1 : n - 1)) % n;  // stick down = next
      s.navLatched = true;
      menuText(s);
    } else if (fabsf(y) < 0.2f) {
      s.navLatched = false;
    }
    if (clicked(s, s.selectAction)) menuSelect(s);
    return;
  }

  handleGrab(s, views, dt);

  if (clicked(s, s.modeAction)) {
    s.heightMode = !s.heightMode;
    LOGI("mode %s", s.heightMode ? "HEIGHT" : "WALK");
  }
  XrVector2f mv = s.lastMove = stick(s, s.moveAction);
  if (fabsf(mv.x) < STICK_DEADZONE) mv.x = 0;
  if (fabsf(mv.y) < STICK_DEADZONE) mv.y = 0;

  if (s.heightMode) {
    s.playerY = fminf(fmaxf(s.playerY + mv.y * HEIGHT_SPEED * dt, MIN_HEIGHT), MAX_HEIGHT);
    if (mv.x != 0) {  // turn, keeping the head's world position fixed
      float hx = 0.5f * (views[0].pose.position.x + views[1].pose.position.x);
      float hz = 0.5f * (views[0].pose.position.z + views[1].pose.position.z);
      float wx = hx, wz = hz;
      yawRotate(s.playerYaw, wx, wz);
      wx += s.playerX; wz += s.playerZ;
      s.playerYaw -= mv.x * TURN_SPEED * (float)M_PI / 180 * dt;  // stick right = turn right
      float rx = hx, rz = hz;
      yawRotate(s.playerYaw, rx, rz);
      s.playerX = wx - rx;
      s.playerZ = wz - rz;
    }
    return;
  }
  const XrQuaternionf& q = views[0].pose.orientation;
  float fx = -2 * (q.x * q.z + q.w * q.y), fz = -(1 - 2 * (q.x * q.x + q.y * q.y));  // head -z, flattened
  float len = sqrtf(fx * fx + fz * fz);
  if (len < 1e-3f) return;  // looking straight up/down
  fx /= len; fz /= len;
  float dx = (fx * mv.y - fz * mv.x) * MOVE_SPEED * dt;  // right = (-fz, fx)
  float dz = (fz * mv.y + fx * mv.x) * MOVE_SPEED * dt;
  yawRotate(s.playerYaw, dx, dz);
  s.playerX += dx;
  s.playerZ += dz;
}

// world -> tracking space: inverse of translate(player) * rotateY(yaw)
static Mat4 playerInverse(const App& s) {
  return mul(rotY(-s.playerYaw), translate(-s.playerX, -s.playerY, -s.playerZ));
}

static void pollXrEvents(App& s) {
  XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
  while (xrPollEvent(s.instance, &ev) == XR_SUCCESS) {
    if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
      auto* sc = (XrEventDataSessionStateChanged*)&ev;
      LOGI("session state %d", (int)sc->state);
      if (sc->state == XR_SESSION_STATE_READY) {
        XrSessionBeginInfo bi{XR_TYPE_SESSION_BEGIN_INFO};
        bi.primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        s.running = XR_SUCCEEDED(xrBeginSession(s.session, &bi));
      } else if (sc->state == XR_SESSION_STATE_STOPPING) {
        xrEndSession(s.session);
        s.running = false;
      } else if (sc->state == XR_SESSION_STATE_EXITING || sc->state == XR_SESSION_STATE_LOSS_PENDING) {
        s.quit = true;
      }
    } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
      s.quit = true;
    }
    ev = {XR_TYPE_EVENT_DATA_BUFFER};
  }
}

static void frame(App& s) {
  XrFrameWaitInfo wi{XR_TYPE_FRAME_WAIT_INFO};
  XrFrameState fs{XR_TYPE_FRAME_STATE};
  if (XR_FAILED(xrWaitFrame(s.session, &wi, &fs))) return;
  XrFrameBeginInfo bi{XR_TYPE_FRAME_BEGIN_INFO};
  xrBeginFrame(s.session, &bi);

  if (s.animStart < 0) s.animStart = now();

  XrCompositionLayerProjectionView pv[2] = {{XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW},
                                            {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW}};
  XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
  const XrCompositionLayerBaseHeader* layers[1] = {(XrCompositionLayerBaseHeader*)&layer};
  uint32_t nlayers = 0;

  XrViewLocateInfo li{XR_TYPE_VIEW_LOCATE_INFO};
  li.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  li.displayTime = fs.predictedDisplayTime;
  li.space = s.space;
  XrViewState vs{XR_TYPE_VIEW_STATE};
  XrView views[2] = {{XR_TYPE_VIEW}, {XR_TYPE_VIEW}};
  uint32_t nv = 0;
  bool located = XR_SUCCEEDED(xrLocateViews(s.session, &li, &vs, 2, &nv, views)) &&
                 (vs.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);

  double tFrame = now();
  float dt = s.lastFrameT > 0 ? (float)fmin(tFrame - s.lastFrameT, 0.1) : 0;
  s.lastFrameT = tFrame;
  XrSpace spaces[3] = {s.gripSpace[0], s.gripSpace[1], s.aimSpace};
  XrPosef* poses[3] = {&s.handPose[0], &s.handPose[1], &s.aimPose};
  bool* valid[3] = {&s.handValid[0], &s.handValid[1], &s.aimValid};
  for (int i = 0; i < 3; i++) {
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    *valid[i] = XR_SUCCEEDED(xrLocateSpace(spaces[i], s.space, fs.predictedDisplayTime, &loc)) &&
                (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
                (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT);
    if (*valid[i]) *poses[i] = loc.pose;
  }
  if (located) locomote(s, views, dt);
  solveIK(s);
  simulate(s, tFrame - s.animStart);

  if (fs.shouldRender && located) {
    Mat4 placement = mul(playerInverse(s), scenePlacement(s.stage));
    XrPosef headPose = views[0].pose;  // HUD follows the head: eye-0 orientation, mid-eye position
    headPose.position.x = 0.5f * (views[0].pose.position.x + views[1].pose.position.x);
    headPose.position.y = 0.5f * (views[0].pose.position.y + views[1].pose.position.y);
    headPose.position.z = 0.5f * (views[0].pose.position.z + views[1].pose.position.z);
    Mat4 head = poseMatrix(headPose);

    for (int eye = 0; eye < 2; eye++) {
      uint32_t img;
      XrSwapchainImageAcquireInfo ai{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
      xrAcquireSwapchainImage(s.swapchain[eye], &ai, &img);
      XrSwapchainImageWaitInfo wi2{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
      wi2.timeout = XR_INFINITE_DURATION;
      xrWaitSwapchainImage(s.swapchain[eye], &wi2);

      glBindFramebuffer(GL_FRAMEBUFFER, s.fbo[eye][img]);
      glViewport(0, 0, s.width[eye], s.height[eye]);
      renderEye(s, views[eye], placement);
      Mat4 vpTracking = mul(projection(views[eye].fov, 0.05f, 100.f), viewFromPose(views[eye].pose));
      if (s.menuOpen) drawPanel(s, s.menuTex, vpTracking, s.menuModel);
      if (s.statsOn) drawHud(s, vpTracking, head);
      drawJointPanel(s, vpTracking, head);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);

      XrSwapchainImageReleaseInfo ri{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
      xrReleaseSwapchainImage(s.swapchain[eye], &ri);

      pv[eye].pose = views[eye].pose;
      pv[eye].fov = views[eye].fov;
      pv[eye].subImage.swapchain = s.swapchain[eye];
      pv[eye].subImage.imageRect.extent = {s.width[eye], s.height[eye]};
    }
    layer.space = s.space;
    layer.viewCount = 2;
    layer.views = pv;
    nlayers = 1;
  }

  XrFrameEndInfo ei{XR_TYPE_FRAME_END_INFO};
  ei.displayTime = fs.predictedDisplayTime;
  ei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  ei.layerCount = nlayers;
  ei.layers = layers;
  xrEndFrame(s.session, &ei);

  // HUD every 0.5 s, stats to logcat every 2 s:  adb logcat -s MuJoCoXR
  s.frames++;
  s.hudFrames++;
  double t = now();
  if (t - s.hudT > 0.5) {
    if (s.hudT > 0 && s.statsOn) hudText(s, hudLines(s, s.hudFrames / (t - s.hudT), 1000 * s.simCost / s.frames));
    s.hudT = t;
    s.hudFrames = 0;
  }
  if (s.selEff >= 0 && t - s.jointT > 0.1) {
    jointText(s);
    s.jointT = t;
  }
  if (t - s.statT > 2) {
    if (s.statT > 0)
      LOGI("%.1f fps, sim %.2f ms/frame (%s) | mode %s pos (%.1f,%.1f,%.1f) yaw %.0f", s.frames / (t - s.statT),
           1000 * s.simCost / s.frames, s.simRobot >= 0 ? s.robotNames[s.simRobot].c_str() : "kinematic",
           s.heightMode ? "HEIGHT" : "WALK", s.playerX, s.playerY, s.playerZ, s.playerYaw * 180 / M_PI);
    s.statT = t;
    s.frames = 0;
    s.simCost = 0;
  }
}

void android_main(android_app* app) {
  App s;
  s.android = app;
  JNIEnv* env;
  app->activity->vm->AttachCurrentThread(&env, nullptr);

  if (!initEgl(s) || !initXr(s) || !initScene(s, readAsset(app, SCENE_ASSET))) {
    LOGE("init failed, exiting");
    ANativeActivity_finish(app->activity);
  }

  while (!app->destroyRequested) {
    int events;
    android_poll_source* src;
    // block briefly when idle; never block while the XR session is running
    while (ALooper_pollOnce(s.running ? 0 : 100, nullptr, &events, (void**)&src) >= 0) {
      if (src) src->process(app, src);
      if (app->destroyRequested) break;
    }
    if (s.session == XR_NULL_HANDLE) continue;  // init failed, waiting for destroy
    pollXrEvents(s);
    if (s.quit) {
      ANativeActivity_finish(app->activity);
      s.quit = false;
      s.running = false;
    }
    if (s.running) frame(s);
  }

  if (s.d) mj_deleteData(s.d);
  if (s.m) mj_deleteModel(s.m);
  if (s.session) xrDestroySession(s.session);
  if (s.instance) xrDestroyInstance(s.instance);
  app->activity->vm->DetachCurrentThread();
}
#endif  // __ANDROID__
