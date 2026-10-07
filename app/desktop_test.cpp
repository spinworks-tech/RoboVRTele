// Desktop check of the headset renderer: same initScene/renderEye code as the APK, rendered
// offscreen with EGL + GLES3 (needs a GPU driver with EGL), written as front.ppm / side.ppm.
//   ./desktop_test.sh   (from android/)
#include "main.cpp"
#ifndef EFF
#define EFF 0  // effector to grab in the IK test (index into the model's "effectors" list)
#endif

static XrView makeView(float px, float py, float pz, float qx, float qy, float qz, float qw) {
  XrView v{XR_TYPE_VIEW};
  v.pose.position = {px, py, pz};
  v.pose.orientation = {qx, qy, qz, qw};
  v.fov = {-0.8f, 0.8f, 0.7f, -0.7f};  // left, right, up, down (rad)
  return v;
}

static void savePpm(const char* path, int w, int h) {
  std::vector<unsigned char> px(w * h * 4);
  glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
  FILE* f = fopen(path, "wb");
  fprintf(f, "P6 %d %d 255\n", w, h);
  for (int y = h - 1; y >= 0; y--)
    for (int x = 0; x < w; x++) fwrite(&px[(y * w + x) * 4], 1, 3, f);
  fclose(f);
}

int main(int argc, char** argv) {
  App s;
  if (argc < 3) return 1;
  FILE* f = fopen(argv[1], "rb");
  if (!f) return 1;
  std::vector<char> mjb;
  char buf[1 << 16];
  size_t n;
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) mjb.insert(mjb.end(), buf, buf + n);
  fclose(f);
  if (!initEgl(s) || !initScene(s, mjb)) return 1;
  simulate(s, 0.0);
  hudText(s, hudLines(s, 72.0f, 0.4f));
  const int W = 800, H = 700;
  GLuint fbo, color, depth;
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glGenRenderbuffers(1, &color);
  glBindRenderbuffer(GL_RENDERBUFFER, color);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, W, H);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, color);
  glGenRenderbuffers(1, &depth);
  glBindRenderbuffer(GL_RENDERBUFFER, depth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, W, H);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth);
  glViewport(0, 0, W, H);
  Mat4 placement = scenePlacement(true);

  // user 4 m behind the play-space origin, head 1.6 m, looking 15 deg down at the robot row; with HUD
  float a = -15 * M_PI / 180 / 2;
  XrView front = makeView(0, 1.6f, 4.0f, sinf(a), 0, 0, cosf(a));
  renderEye(s, front, placement);
  Mat4 vpFront = mul(projection(front.fov, 0.05f, 100.f), viewFromPose(front.pose));
  s.menuCursor = (int)s.robotNames.size();  // on "SETTINGS >"
  s.robotShown[s.robotShown.size() - 1] = 0;  // menu shows one robot unticked
  openMenu(s, front.pose);
  drawPanel(s, s.menuTex, vpFront, s.menuModel);
  drawHud(s, vpFront, poseMatrix(front.pose));
  s.robotShown[s.robotShown.size() - 1] = 1;
  savePpm((std::string(argv[2]) + "/front.ppm").c_str(), W, H);

  // IK + physics: grab effector EFF, move its target, let the simulated arm follow for 4 s
  int eff = EFF;
  grab(s, eff);
  mjtNum start[3];
  mju_copy3(start, s.d->xpos + 3 * s.effBody[eff]);
#ifndef DELTA
#define DELTA {0.15, 0.15, -0.15}
#endif
  mjtNum delta[3] = DELTA;
  for (int i = 0; i < 3; i++) s.ikTarget[i] += delta[i];
  for (int i = 0; i < 288; i++) {
    solveIK(s);
    simulate(s, 1.0 + i / 72.0);
  }
  mjtNum err[3];
  mju_sub3(err, s.ikTarget, s.d->xpos + 3 * s.effBody[eff]);
  // IK's own solution (kinematics of the joint targets) vs where physics actually got
  mju_copy(s.dIK->qpos, s.d->qpos, s.m->nq);
  for (int j : s.ikJoints) s.dIK->qpos[s.m->jnt_qposadr[j]] = s.qTarget[s.m->jnt_qposadr[j]];
  mj_kinematics(s.m, s.dIK);
  LOGI("IK %s: moved %.3f m | error: IK solution %.3f m, physics %.3f m | contacts %d, NaN %d",
       s.robotNames[s.effRobot[eff]].c_str(), mju_dist3(start, s.d->xpos + 3 * s.effBody[eff]),
       mju_dist3(s.ikTarget, s.dIK->xpos + 3 * s.effBody[eff]), mju_norm3(err), s.d->ncon, (int)mju_isBad(s.d->qpos[0]));

  // close-up of the grabbed effector with a controller stand-in and the ray pointing at it
  const mjtNum* pe = s.d->xpos + 3 * s.effBody[eff];
  float px = placement.m[0] * pe[0] + placement.m[4] * pe[1] + placement.m[8] * pe[2] + placement.m[12];
  float pz = placement.m[2] * pe[0] + placement.m[6] * pe[1] + placement.m[10] * pe[2] + placement.m[14];
  float camx = px * 0.7f, camz = pz * 0.7f;  // 30% of the way from the arc centre
  float yaw = atan2f(-(px - camx), -(pz - camz));
  XrView close = makeView(camx, 1.3f, camz, 0, sinf(yaw / 2), 0, cosf(yaw / 2));
  s.handValid[1] = s.aimValid = true;
  s.aimPose = close.pose;
  Mat4 cam = poseMatrix(close.pose);
  float off[3] = {0.12f, -0.25f, -0.35f};  // hand in front, lower right
  s.aimPose.position = {cam.m[12] + cam.m[0] * off[0] + cam.m[4] * off[1] + cam.m[8] * off[2],
                        cam.m[13] + cam.m[1] * off[0] + cam.m[5] * off[1] + cam.m[9] * off[2],
                        cam.m[14] + cam.m[2] * off[0] + cam.m[6] * off[1] + cam.m[10] * off[2]};
  s.handPose[1] = s.aimPose;
  s.hoverEff = eff;
  s.hoverDist = 0.9f;
  s.hoverBody = s.m->body_parentid[s.effBody[eff]];  // pointed-at link: cyan (chain: orange)
  renderEye(s, close, placement);
  Mat4 vpClose = mul(projection(close.fov, 0.05f, 100.f), viewFromPose(close.pose));
  jointText(s);
  drawJointPanel(s, vpClose, poseMatrix(close.pose));
  drawHud(s, vpClose, poseMatrix(close.pose));
  savePpm((std::string(argv[2]) + "/side.ppm").c_str(), W, H);
  // the user's starting view: standing at the origin, looking 30 deg down at the table
  float t30 = -30 * M_PI / 180 / 2;
  grab(s, -1);
  s.handValid[1] = s.aimValid = false;
  renderEye(s, makeView(0, 1.6f, 0, sinf(t30), 0, 0, cosf(t30)), placement);
  savePpm((std::string(argv[2]) + "/table.ppm").c_str(), W, H);
  LOGI("GL error: 0x%x", glGetError());
  return 0;
}
