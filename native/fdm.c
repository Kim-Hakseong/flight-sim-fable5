/* fdm.c — see fdm.h. Operation ORDER mirrors src/physics.js + src/wind.js
 * expression-for-expression so golden trajectories track the JS reference to
 * libm-vs-V8 last-ulp differences only. Do not "optimize" the math. */
#include "fdm.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* --- Airframe constants (src/physics.js AC) -------------------------------- */
#define G_ACC 9.81
#define RHO0 1.225
#define MASS 13.5
#define JX 0.8244
#define JY 1.135
#define JZ 1.759
#define WS 0.55   /* wing area */
#define WB 2.9    /* span */
#define WC 0.19   /* chord */
#define CL0 0.28
#define CLA 3.45
#define CLDE 0.36
#define CD0 0.03
#define KIND 0.0231
#define CM0 (-0.02338)
#define CMA (-0.38)
#define CMQ (-3.6)
#define CMDE (-0.5)
#define CYB (-0.98)
#define CYDR 0.19
#define CLB (-0.12)
#define CLP (-0.26)
#define CLR 0.14
#define CLDA 0.13
#define CLDR 0.008
#define CNB 0.25
#define CNP 0.022
#define CNR (-0.35)
#define CNDA (-0.011)
#define CNDR (-0.069)
#define SPROP 0.2027
#define CPROP 1.0
#define KMOTOR 50.0
#define MAX_THRUST_N 60.0
#define MU_ROLL 0.03
#define MU_BRAKE 0.22
#define MAX_DEF 0.44
#define ACT_TAU 0.05
#define THR_TAU 0.4
#define ALPHA_CLAMP 0.30

const double FDM_TRIM_VA = 30.0, FDM_TRIM_ALPHA = 0.05566,
             FDM_TRIM_DE = -0.08906, FDM_TRIM_DT = 0.62747;

/* --- Small helpers ----------------------------------------------------------- */
static double clampd(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
static void quat_multiply(const double a[4], const double b[4], double out[4]) {
  double ax = a[0], ay = a[1], az = a[2], aw = a[3];
  double bx = b[0], by = b[1], bz = b[2], bw = b[3];
  out[0] = aw * bx + ax * bw + ay * bz - az * by;
  out[1] = aw * by - ax * bz + ay * bw + az * bx;
  out[2] = aw * bz + ax * by - ay * bx + az * bw;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
}
static void quat_normalize(double q[4]) {
  double n = hypot(hypot(q[0], q[1]), hypot(q[2], q[3]));
  if (n == 0.0) n = 1.0;
  q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
}
static void quat_rotate(const double q[4], const double v[3], double out[3]) {
  double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  double vx = v[0], vy = v[1], vz = v[2];
  double tx = 2.0 * (qy * vz - qz * vy);
  double ty = 2.0 * (qz * vx - qx * vz);
  double tz = 2.0 * (qx * vy - qy * vx);
  out[0] = vx + qw * tx + qy * tz - qz * ty;
  out[1] = vy + qw * ty + qz * tx - qx * tz;
  out[2] = vz + qw * tz + qx * ty - qy * tx;
}
static void quat_conj_rotate(const double q[4], const double v[3], double out[3]) {
  double c[4] = { -q[0], -q[1], -q[2], q[3] };
  quat_rotate(c, v, out);
}
static void quat_integrate(double q[4], const double omega[3], double dt) {
  double h = 0.5 * dt;
  double w[4] = { omega[0] * h, omega[1] * h, omega[2] * h, 0.0 };
  double dq[4];
  quat_multiply(q, w, dq);
  q[0] += dq[0]; q[1] += dq[1]; q[2] += dq[2]; q[3] += dq[3];
  quat_normalize(q);
}
/* ours → FRD (nose, right, belly); same map converts rates. */
static void to_frd(const double v[3], double out[3]) {
  out[0] = -v[2]; out[1] = v[0]; out[2] = -v[1];
}
static void from_frd(const double v[3], double out[3]) {
  out[0] = v[1]; out[1] = -v[2]; out[2] = -v[0];
}

/* --- Atmosphere + air data ---------------------------------------------------- */
double fdm_air_density(double alt_m) {
  double h = alt_m < 0 ? 0 : (alt_m > 11000 ? 11000 : alt_m);
  return RHO0 * pow(1.0 - 2.2557e-5 * h, 4.2559);
}

void fdm_air_data(const double quat[4], const double vel[3],
                  const double wind[3], double *va, double *alpha, double *beta) {
  double rel[3] = { vel[0] - wind[0], vel[1] - wind[1], vel[2] - wind[2] };
  double b[3], f[3];
  quat_conj_rotate(quat, rel, b);
  to_frd(b, f);
  double u = f[0], v = f[1], w = f[2];
  double Va = hypot(hypot(u, v), w);
  *va = Va;
  if (Va < 1.0) { *alpha = 0.0; *beta = 0.0; return; }
  *alpha = atan2(w, u);
  *beta = asin(clampd(v / Va, -1.0, 1.0));
}

/* --- Forces + moments (FRD) ---------------------------------------------------- */
void fdm_forces_moments(const fdm_state *s, const double wind[3],
                        double F[3], double M[3],
                        double *va_o, double *alpha_o, double *beta_o) {
  double Va, alpha, beta;
  fdm_air_data(s->quat, s->vel, wind, &Va, &alpha, &beta);
  double rho = fdm_air_density(s->pos[1]);
  double qbar = 0.5 * rho * Va * Va;
  double frd[3];
  to_frd(s->omega, frd);
  double p = frd[0], q = frd[1], r = frd[2];
  double bV = Va > 1.0 ? WB / (2.0 * Va) : 0.0;
  double cV = Va > 1.0 ? WC / (2.0 * Va) : 0.0;

  double aEff = clampd(alpha, -ALPHA_CLAMP, ALPHA_CLAMP);
  double CL = CL0 + CLA * aEff + CLDE * s->act.de;
  double CD = CD0 + KIND * CL * CL;
  double lift = qbar * WS * CL;
  double drag = qbar * WS * CD;
  double fy = qbar * WS * (CYB * beta + CYDR * s->act.dr);

  double thrust = 0.5 * fdm_air_density(s->pos[1]) * SPROP * CPROP *
                  ((KMOTOR * s->act.dt) * (KMOTOR * s->act.dt) - Va * Va);
  if (thrust > MAX_THRUST_N) thrust = MAX_THRUST_N;

  double ca = cos(alpha), sa = sin(alpha);
  double gw[3] = { 0.0, -MASS * G_ACC, 0.0 };
  double gb_ours[3], gB[3];
  quat_conj_rotate(s->quat, gw, gb_ours);
  to_frd(gb_ours, gB);

  F[0] = thrust - drag * ca + lift * sa + gB[0];
  F[1] = fy + gB[1];
  F[2] = -drag * sa - lift * ca + gB[2];
  M[0] = qbar * WS * WB * (CLB * beta + CLP * bV * p + CLR * bV * r + CLDA * s->act.da + CLDR * s->act.dr);
  M[1] = qbar * WS * WC * (CM0 + CMA * aEff + CMQ * cV * q + CMDE * s->act.de);
  M[2] = qbar * WS * WB * (CNB * beta + CNP * bV * p + CNR * bV * r + CNDA * s->act.da + CNDR * s->act.dr);
  if (va_o) *va_o = Va;
  if (alpha_o) *alpha_o = alpha;
  if (beta_o) *beta_o = beta;
}

/* --- Actuators (with servo faults) --------------------------------------------- */
static double act_ch(double cur, double target, double tau, int ftype,
                     double factor, double dt) {
  if (ftype == 1) return cur;               /* jam */
  double goal = (ftype == 2) ? 0.0 : target; /* floating streams to neutral */
  double t = tau * (ftype == 3 ? (factor > 0 ? factor : 6.0) : 1.0);
  double k = dt / t;
  if (k > 1.0) k = 1.0;
  return cur + (goal - cur) * k;
}

static void step_actuators(fdm_act *a, const fdm_cmds *c, const fdm_faults *f,
                           double dt) {
  fdm_faults nf = {0, 0, 0, 0, 0.0};
  if (!f) f = &nf;
  double tda = clampd(c->aileron * MAX_DEF, -MAX_DEF, MAX_DEF);
  double tde = clampd(c->elevator * MAX_DEF, -MAX_DEF, MAX_DEF);
  double tdr = clampd(c->rudder * MAX_DEF, -MAX_DEF, MAX_DEF);
  double tdt = clampd(c->throttle, 0.0, 1.0);
  a->da = act_ch(a->da, tda, ACT_TAU, f->type_da, f->factor, dt);
  a->de = act_ch(a->de, tde, ACT_TAU, f->type_de, f->factor, dt);
  a->dr = act_ch(a->dr, tdr, ACT_TAU, f->type_dr, f->factor, dt);
  a->dt = act_ch(a->dt, tdt, THR_TAU, f->type_dt, f->factor, dt);
}

/* --- Dryden wind (mulberry32 Gauss–Markov, mirrors src/wind.js) ----------------- */
static const double DRY_L[3] = {200.0, 200.0, 50.0};
static const double DRY_S[3] = {1.06, 1.06, 0.7};

static double prng_next(int32_t *state) {
  *state = (int32_t)((uint32_t)*state + 0x6d2b79f5u);
  uint32_t t = (uint32_t)*state;
  t = (t ^ (t >> 15)) * (1u | t);
  t = (t + ((t ^ (t >> 7)) * (61u | t))) ^ t;
  return (double)(t ^ (t >> 14)) / 4294967296.0;
}
static double gaussian_next(int32_t *state) {
  double u1 = prng_next(state);
  double u2 = prng_next(state);
  return sqrt(-2.0 * log(1.0 - u1)) * cos(2.0 * M_PI * u2);
}

void fdm_wind_init(fdm_wind *w, int32_t seed) {
  w->rng = seed;
  w->gust[0] = w->gust[1] = w->gust[2] = 0.0;
}

static void step_wind(fdm_wind *w, const fdm_state *s, const fdm_env *env,
                      double dt, double out[3]) {
  double zero[3] = {0, 0, 0};
  double Va, al, be;
  fdm_air_data(s->quat, s->vel, zero, &Va, &al, &be);
  double V = Va > 5.0 ? Va : 5.0;
  for (int i = 0; i < 3; i++) {
    double eta = gaussian_next(&w->rng);
    double a = (V / DRY_L[i]) * dt;
    if (a > 1.0) a = 1.0;
    w->gust[i] = w->gust[i] * (1.0 - a) + DRY_S[i] * env->turb * sqrt(2.0 * a) * eta;
  }
  double gours[3], gw[3];
  from_frd(w->gust, gours);
  quat_rotate(s->quat, gours, gw);
  out[0] = env->wind_e + gw[0];
  out[1] = gw[1];
  out[2] = -env->wind_n + gw[2];
}

/* --- Rigid-body step ------------------------------------------------------------ */
void fdm_step(fdm_state *s, const fdm_cmds *c, const fdm_faults *f,
              fdm_wind *w, const fdm_env *env, double dt, double ww_out[3]) {
  static const fdm_env calm = {0.0, 0.0, 0.0};
  double ww[3] = {0, 0, 0};
  if (w && env) step_wind(w, s, env, dt, ww);
  else if (w) step_wind(w, s, &calm, dt, ww);
  if (ww_out) { ww_out[0] = ww[0]; ww_out[1] = ww[1]; ww_out[2] = ww[2]; }

  step_actuators(&s->act, c, f, dt);
  double F[3], M[3];
  fdm_forces_moments(s, ww, F, M, 0, 0, 0);

  double f_ours[3], fw[3];
  from_frd(F, f_ours);
  quat_rotate(s->quat, f_ours, fw);
  for (int i = 0; i < 3; i++) s->vel[i] += (fw[i] / MASS) * dt;
  for (int i = 0; i < 3; i++) s->pos[i] += s->vel[i] * dt;

  double frd[3];
  to_frd(s->omega, frd);
  double p = frd[0], q = frd[1], r = frd[2];
  double pDot = (M[0] + (JY - JZ) * q * r) / JX;
  double qDot = (M[1] + (JZ - JX) * p * r) / JY;
  double rDot = (M[2] + (JX - JY) * p * q) / JZ;
  double nfrd[3] = { p + pDot * dt, q + qDot * dt, r + rDot * dt };
  from_frd(nfrd, s->omega);
  quat_integrate(s->quat, s->omega, dt);

  if (s->pos[1] <= 0.0) {
    s->pos[1] = 0.0;
    if (s->vel[1] < 0.0) s->vel[1] = 0.0;
    double gs = hypot(s->vel[0], s->vel[2]);
    if (gs > 0.0) {
      double mu = MU_ROLL + (s->act.dt < 0.1 ? MU_BRAKE : 0.0);
      double dec = mu * G_ACC * dt;
      if (dec > gs) dec = gs;
      s->vel[0] -= (s->vel[0] / gs) * dec;
      s->vel[2] -= (s->vel[2] / gs) * dec;
    }
    to_frd(s->omega, frd);
    p = frd[0]; q = frd[1]; r = frd[2];
    double right[3], nose[3];
    double ex[3] = {1, 0, 0}, ez[3] = {0, 0, -1};
    quat_rotate(s->quat, ex, right);
    quat_rotate(s->quat, ez, nose);
    double roll = asin(clampd(-right[1], -1.0, 1.0));
    double pitch = asin(clampd(nose[1], -1.0, 1.0));
    double kp = 6.0 * dt; if (kp > 1.0) kp = 1.0;
    double kr = 1.5 * dt; if (kr > 1.0) kr = 1.0;
    double pNew = p * (1.0 - kp) - roll * 4.0 * dt;
    double qNew = (pitch < -0.02 && q < 0.0) ? 0.0 : q;
    double gfrd[3] = { pNew, qNew, r * (1.0 - kr) };
    from_frd(gfrd, s->omega);
  }
}

/* --- Boot states ---------------------------------------------------------------- */
void fdm_ground_state(fdm_state *s) {
  memset(s, 0, sizeof(*s));
  s->pos[2] = 350.0;
  s->quat[3] = 1.0;
}
void fdm_initial_state(fdm_state *s) {
  memset(s, 0, sizeof(*s));
  s->pos[1] = 120.0;
  s->vel[2] = -FDM_TRIM_VA;
  double h = FDM_TRIM_ALPHA / 2.0;
  s->quat[0] = sin(h); s->quat[3] = cos(h);
  quat_normalize(s->quat);
  s->act.de = FDM_TRIM_DE;
  s->act.dt = FDM_TRIM_DT;
}
