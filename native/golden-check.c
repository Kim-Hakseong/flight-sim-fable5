/* golden-check.c — cross-validate the C core against JS golden vectors.
 * Point checks are tight (last-ulp-class); trajectory checks allow bounded
 * drift from libm-vs-V8 transcendental differences amplified over time. */
#include "fdm.h"
#include "golden.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int failures = 0;
static void check(int ok, const char *what, double worst) {
  printf("%s %s (worst %.3e)\n", ok ? "ok " : "FAIL", what, worst);
  if (!ok) failures++;
}

static void load_state(const double row[17], fdm_state *s) {
  memcpy(s->pos, row, 3 * sizeof(double));
  memcpy(s->vel, row + 3, 3 * sizeof(double));
  memcpy(s->quat, row + 6, 4 * sizeof(double));
  memcpy(s->omega, row + 10, 3 * sizeof(double));
  s->act.da = row[13]; s->act.de = row[14]; s->act.dr = row[15]; s->act.dt = row[16];
}

int main(void) {
  /* A) point-wise forces/moments */
  double worst = 0.0;
  for (int i = 0; i < N_POINTS; i++) {
    fdm_state s;
    load_state(GP_STATE[i], &s);
    double F[3], M[3], va, al, be;
    fdm_forces_moments(&s, GP_WIND[i], F, M, &va, &al, &be);
    double got[9] = {F[0], F[1], F[2], M[0], M[1], M[2], va, al, be};
    for (int k = 0; k < 9; k++) {
      double ref = GP_OUT[i][k];
      double scale = fabs(ref) > 1.0 ? fabs(ref) : 1.0;
      double err = fabs(got[k] - ref) / scale;
      if (err > worst) worst = err;
    }
  }
  check(worst < 1e-12, "forces/moments match the JS reference point-wise", worst);

  /* B) trajectories: step the C core exactly as the JS generator did. */
  static const double DT = 1.0 / 60.0;
  const double TRIM_ELEV = FDM_TRIM_DE / 0.44;

  for (int ti = 0; ti < 3; ti++) {
    fdm_state s;
    if (ti == 2) { fdm_ground_state(&s); s.pos[2] = 0.0; }
    else fdm_initial_state(&s);
    fdm_wind w;
    fdm_wind_init(&w, 2);
    fdm_env env = { TRAJ_ENV[ti][0], TRAJ_ENV[ti][1], TRAJ_ENV[ti][2] };
    int n = ti == 0 ? T0_N : ti == 1 ? T1_N : T2_N;
    const double (*ref)[17] = ti == 0 ? T0_SAMPLES : ti == 1 ? T1_SAMPLES : T2_SAMPLES;

    double worstPos = 0.0, worstQuat = 0.0;
    for (int sec = 0; sec < n; sec++) {
      for (int i = sec * 60; i < (sec + 1) * 60; i++) {
        fdm_cmds c = {0.0, TRIM_ELEV, 0.0, FDM_TRIM_DT};
        if (ti == 1) { /* doublets_turb */
          c.aileron = (i >= 300 && i < 360) ? 0.3 : 0.0;
          c.elevator = TRIM_ELEV + ((i >= 600 && i < 660) ? -0.2 : 0.0);
          c.rudder = (i >= 900 && i < 960) ? 0.2 : 0.0;
        } else if (ti == 2) { /* ground_roll */
          c.aileron = 0.0; c.elevator = 0.0; c.rudder = 0.0; c.throttle = 1.0;
        }
        fdm_step(&s, &c, 0, &w, &env, DT, 0);
      }
      for (int k = 0; k < 3; k++) {
        double e = fabs(s.pos[k] - ref[sec][k]);
        if (e > worstPos) worstPos = e;
      }
      for (int k = 0; k < 4; k++) {
        double e = fabs(s.quat[k] - ref[sec][6 + k]);
        if (e > worstQuat) worstQuat = e;
      }
    }
    const char *names[3] = {"trim_calm 30 s", "doublets_turb 20 s", "ground_roll 8 s"};
    double posTol = ti == 1 ? 0.5 : 1e-3; /* turbulence amplifies ulp noise */
    check(worstPos < posTol, names[ti], worstPos);
    check(worstQuat < (ti == 1 ? 1e-2 : 1e-6), "  └ attitude drift", worstQuat);
  }

  printf(failures ? "GOLDEN: %d FAILED\n" : "GOLDEN: ALL PASS\n", failures);
  return failures ? 1 : 0;
}
