/* cov-driver.c — exercises the public FDM API surface that golden-check.c does
 * not reach directly (fdm_euler / fdm_rates_frd — used by the FMU + VeriStand
 * wrappers — and the wind-with-null-env fallback path in fdm_step). Combined
 * with golden-check under --coverage, this brings structural coverage of the
 * deployable core (fdm.c) to its reachable maximum. Assertions keep it honest:
 * a wrong result fails the build, so this is a test, not just a line-toucher. */
#include "fdm.h"
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int fails = 0;
static void expect(int ok, const char *what) {
  if (!ok) { printf("FAIL cov-driver: %s\n", what); fails++; }
}

int main(void) {
  /* fdm_euler on a known attitude: pure 20° right roll about the nose (−Z). */
  double h = (20.0 * M_PI / 180.0) / 2.0;
  double q[4] = {0, 0, -sin(h), cos(h)};
  double roll, pitch, yaw;
  fdm_euler(q, &roll, &pitch, &yaw);
  expect(fabs(roll - 20.0 * M_PI / 180.0) < 1e-9, "euler roll = 20 deg");
  expect(fabs(pitch) < 1e-9 && fabs(yaw) < 1e-9, "euler pitch/yaw = 0");

  /* Identity attitude → all zero. */
  double qi[4] = {0, 0, 0, 1};
  fdm_euler(qi, &roll, &pitch, &yaw);
  expect(fabs(roll) < 1e-12 && fabs(pitch) < 1e-12 && fabs(yaw) < 1e-12, "euler identity");

  /* fdm_rates_frd: ours [wx,wy,wz] → FRD [−wz, wx, −wy]. */
  double om[3] = {0.1, -0.2, 0.3};
  double frd[3];
  fdm_rates_frd(om, frd);
  expect(fabs(frd[0] + 0.3) < 1e-12 && fabs(frd[1] - 0.1) < 1e-12 && fabs(frd[2] - 0.2) < 1e-12,
         "rates_frd mapping");

  /* fdm_step with a wind pointer but NULL env → calm fallback path (no crash,
   * no wind applied), and it must still integrate one step deterministically. */
  fdm_state s;
  fdm_initial_state(&s);
  fdm_wind w;
  fdm_wind_init(&w, 2);
  fdm_cmds c = {0.0, FDM_TRIM_DE / 0.44, 0.0, FDM_TRIM_DT};
  double ww[3] = {1, 1, 1};
  fdm_step(&s, &c, 0, &w, 0 /* env NULL → calm branch */, 1.0 / 60.0, ww);
  expect(fabs(ww[0]) < 1e-12 && fabs(ww[1]) < 1e-12 && fabs(ww[2]) < 1e-12,
         "null-env wind fallback is calm");
  expect(s.pos[1] > 100.0, "one step keeps it airborne");

  /* fdm_step with NO wind pointer at all (w == NULL): zero wind, still steps. */
  fdm_step(&s, &c, 0, 0, 0, 1.0 / 60.0, 0);
  expect(s.pos[1] > 100.0, "no-wind step ok");

  printf(fails ? "COV-DRIVER: %d FAILED\n" : "COV-DRIVER: PASS\n", fails);
  return fails ? 1 : 0;
}
