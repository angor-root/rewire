#include "base_kin.h"

#include <math.h>

#define TWO_PI_F 6.28318531f
#define PI_F 3.14159265f

float wrap_pi(float a) {
  while (a > PI_F) a -= TWO_PI_F;
  while (a < -PI_F) a += TWO_PI_F;
  return a;
}

int base_ik(const BaseGeom* g, float v, float w, float* wl, float* wr) {
  float l = (v - w * 0.5f * g->track) / g->r;
  float rr = (v + w * 0.5f * g->track) / g->r;
  const float m = fmaxf(fabsf(l), fabsf(rr));
  int sat = 0;
  if (m > g->w_max) {  // escalar las dos igual: se pierde velocidad pero se mantiene la curvatura
    const float k = g->w_max / m;
    l *= k;
    rr *= k;
    sat = 1;
  }
  *wl = l;
  *wr = rr;
  return sat;
}

void base_fk(const BaseGeom* g, float wl, float wr, float* v, float* w) {
  *v = 0.5f * g->r * (wr + wl);
  *w = g->r * (wr - wl) / g->track;
}

void base_odom(const BaseGeom* g, Pose2* p, float dphi_l, float dphi_r) {
  const float ds = 0.5f * g->r * (dphi_r + dphi_l);
  const float dth = g->r * (dphi_r - dphi_l) / g->track;
  const float a = p->th + 0.5f * dth;
  p->x += ds * cosf(a);
  p->y += ds * sinf(a);
  p->th = wrap_pi(p->th + dth);
}

void circle_init(CircleTraj* c, float radius, float speed, int dir, float laps) {
  c->radius = radius;
  c->speed = speed;
  c->accel = 0.25f;
  c->dir = dir >= 0 ? 1 : -1;
  c->laps = laps;
  // con v_r = 0.15 m/s: v_r^2 ky = 1.35 -> wn ~ 1.2 rad/s; v_r kth = 0.45 + el termino de rumbo da zeta ~ 0.8
  c->kx = 2.0f;
  c->ky = 60.0f;
  c->kth = 12.0f;
  c->s = c->sdot = 0;
  c->done = 1;
  c->cx = c->cy = c->th0 = 0;
  c->ref.x = c->ref.y = c->ref.th = 0;
  c->v_ref = c->w_ref = 0;
}

void circle_start(CircleTraj* c, const Pose2* p) {
  // centro a la izquierda (dir +1) o a la derecha (dir -1) del robot, a distancia R
  c->th0 = p->th;
  c->cx = p->x - c->dir * c->radius * sinf(p->th);
  c->cy = p->y + c->dir * c->radius * cosf(p->th);
  c->s = c->sdot = 0;
  c->done = 0;
  c->ref = *p;
}

void circle_finish(CircleTraj* c) {
  if (c->done) return;
  c->laps = c->s / (TWO_PI_F * c->radius) + 1e-6f;  // la rampa de frenado arranca ya
}

void circle_step(CircleTraj* c, const Pose2* p, float dt, float* v, float* w) {
  if (c->done) { *v = *w = 0; return; }
  // perfil trapezoidal de la velocidad sobre el arco
  float target = c->speed;
  if (c->laps > 0) {
    const float remaining = c->laps * TWO_PI_F * c->radius - c->s;
    const float v_stop = sqrtf(fmaxf(0.0f, 2.0f * c->accel * remaining));
    if (v_stop < target) target = v_stop;
    if (remaining <= 0.002f || (target < 0.01f && c->sdot < 0.02f)) {
      c->done = 1;
      c->sdot = 0;
      *v = *w = 0;
      return;
    }
  }
  const float dv = c->accel * dt;
  if (c->sdot < target) c->sdot = fminf(target, c->sdot + dv);
  else c->sdot = fmaxf(target, c->sdot - dv);
  c->s += c->sdot * dt;

  // referencia: el rumbo gira s/R; el radio vector apunta del centro al robot
  const float dth = c->dir * c->s / c->radius;
  c->ref.th = wrap_pi(c->th0 + dth);
  const float phi = c->th0 - c->dir * 0.5f * PI_F + dth;
  c->ref.x = c->cx + c->radius * cosf(phi);
  c->ref.y = c->cy + c->radius * sinf(phi);
  c->v_ref = c->sdot;
  c->w_ref = c->dir * c->sdot / c->radius;

  // error en el frame del robot + Kanayama
  const float dx = c->ref.x - p->x, dy = c->ref.y - p->y;
  const float cs = cosf(p->th), sn = sinf(p->th);
  const float ex = cs * dx + sn * dy;
  const float ey = -sn * dx + cs * dy;
  const float eth = wrap_pi(c->ref.th - p->th);
  *v = c->v_ref * cosf(eth) + c->kx * ex;
  *w = c->w_ref + c->v_ref * (c->ky * ey + c->kth * sinf(eth));
}
