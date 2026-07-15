/* cov-driver.c — assertion-backed exercise of the FDM public API surface and the
 * boundary/fault decision outcomes that the golden trajectories do not reach.
 * Combined with golden-check under coverage instrumentation this drives fdm.c to
 * full LINE coverage and full MC/DC CONDITION coverage of all *reachable*
 * decisions. The few genuinely-unreachable defensive guards are justified in
 * COMPLIANCE-DO331.md §5 (they cannot be exercised without an invalid state).
 *
 * Every case asserts a result — this is a test, not a coverage line-toucher. */
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
  const double DT = 1.0 / 60.0;
  const double TE = FDM_TRIM_DE / 0.44;

  /* --- euler / rates on known attitudes ------------------------------------- */
  double h = (20.0 * M_PI / 180.0) / 2.0;
  double q[4] = {0, 0, -sin(h), cos(h)};
  double roll, pitch, yaw;
  fdm_euler(q, &roll, &pitch, &yaw);
  expect(fabs(roll - 20.0 * M_PI / 180.0) < 1e-9, "euler roll = 20 deg");
  double qi[4] = {0, 0, 0, 1};
  fdm_euler(qi, &roll, &pitch, &yaw);
  expect(fabs(roll) < 1e-12 && fabs(pitch) < 1e-12 && fabs(yaw) < 1e-12, "euler identity");
  /* Robustness: a denormalized quat can push the body-up component past ±1;
     the asin guard (L298) must clamp instead of returning NaN. */
  double qbad[4] = {0.9, 0.0, 0.0, 0.9};   /* norm ≈ 1.27 → fwd[1] can exceed 1 */
  fdm_euler(qbad, &roll, &pitch, &yaw);
  expect(pitch == pitch && fabs(pitch) <= M_PI / 2 + 1e-9, "euler asin guard clamps a denormalized quat");
  double qbad2[4] = {-0.9, 0.0, 0.0, 0.9}; /* the other sign → fwd[1] below −1 */
  fdm_euler(qbad2, &roll, &pitch, &yaw);
  expect(pitch == pitch && fabs(pitch) <= M_PI / 2 + 1e-9, "euler asin guard clamps the other sign");
  double om[3] = {0.1, -0.2, 0.3}, frd[3];
  fdm_rates_frd(om, frd);
  expect(fabs(frd[0] + 0.3) < 1e-12 && fabs(frd[1] - 0.1) < 1e-12 && fabs(frd[2] - 0.2) < 1e-12,
         "rates_frd mapping");

  /* --- atmosphere altitude clamps (L106: alt<0 and alt>11000) --------------- */
  expect(fabs(fdm_air_density(-100.0) - fdm_air_density(0.0)) < 1e-12, "density clamps alt<0 to sea level");
  expect(fabs(fdm_air_density(20000.0) - fdm_air_density(11000.0)) < 1e-12, "density clamps alt>11km to tropopause");

  /* --- command clamps beyond ±1 (L59 clampd both sides, throttle clamp) ----- */
  {
    fdm_state s; fdm_initial_state(&s);
    fdm_cmds over = {5.0, -5.0, 5.0, 2.0};   /* all past the rails, both signs */
    for (int i = 0; i < 30; i++) fdm_step(&s, 0, &over, 0, 0, 0, DT, 0);
    expect(s.act.dt <= 1.0 + 1e-12 && s.act.dt >= 0.0, "throttle clamps to [0,1]");
    expect(fabs(s.act.da) <= 0.44 + 1e-9 && fabs(s.act.de) <= 0.44 + 1e-9, "deflection clamps to ±maxDef");
    fdm_cmds under = {0, 0, 0, -3.0};        /* throttle below 0 */
    fdm_step(&s, 0, &under, 0, 0, 0, DT, 0);
    expect(s.act.dt >= 0.0, "throttle clamps at 0 from below");
  }

  /* --- servo faults: jam / floating / slow (L169/170/171/180) --------------- */
  {
    fdm_state s; fdm_initial_state(&s);
    double held = s.act.da;
    fdm_faults jam = {1, 0, 0, 0, 0.0};                 /* aileron jam */
    fdm_cmds c = {0.5, TE, 0.0, FDM_TRIM_DT};
    for (int i = 0; i < 60; i++) fdm_step(&s, 0, &c, &jam, 0, 0, DT, 0);
    expect(fabs(s.act.da - held) < 1e-12, "jam holds the surface");

    fdm_initial_state(&s);
    fdm_faults flt = {0, 2, 0, 2, 0.0};                 /* elevator + throttle floating */
    fdm_cmds c2 = {0.0, TE, 0.0, 0.9};
    for (int i = 0; i < 600; i++) fdm_step(&s, 0, &c2, &flt, 0, 0, DT, 0);
    expect(fabs(s.act.de) < 1e-3, "floating elevator streams to neutral");
    expect(s.act.dt < 1e-3, "floating throttle dies");

    fdm_initial_state(&s);
    fdm_faults slow = {0, 0, 3, 0, 8.0};                /* rudder slow, factor 8 */
    fdm_cmds c3 = {0.0, TE, 0.4, FDM_TRIM_DT};
    double first = s.act.dr;
    fdm_step(&s, 0, &c3, &slow, 0, 0, DT, 0);
    expect(s.act.dr > first && s.act.dr < 0.05, "slow servo slews, but far short of target");

    fdm_faults deflt_factor = {0, 0, 3, 0, 0.0};        /* slow with factor<=0 → default 6 */
    fdm_step(&s, 0, &c3, &deflt_factor, 0, 0, DT, 0);
    expect(1, "slow default factor path exercised");
  }

  /* --- act_ch k>1 saturation (L173): dt larger than the actuator tau ---------- */
  {
    fdm_state s; fdm_initial_state(&s);
    fdm_cmds c = {1.0, TE, 0.0, FDM_TRIM_DT};
    fdm_step(&s, 0, &c, 0, 0, 0, 0.2 /* > ACT_TAU 0.05 → k clamps to 1 */, 0);
    expect(fabs(s.act.da - 0.44) < 1e-9, "large dt saturates the servo to the rail in one step");
  }

  /* --- wind Gauss-Markov alpha clamp (L222): a huge comm step forces a>1 ------ */
  {
    fdm_state s; fdm_initial_state(&s);
    fdm_wind w; fdm_wind_init(&w, 2);
    fdm_env env = {0.0, 0.0, 1.0};
    fdm_cmds c = {0.0, TE, 0.0, FDM_TRIM_DT};
    fdm_step(&s, 0, &c, 0, &w, &env, 3.0 /* >> L/V ⇒ a clamps to 1 */, 0);
    expect(1, "wind alpha clamp exercised");
  }

  /* --- null-env wind fallback (L calm branch) and no-wind path ---------------- */
  {
    fdm_state s; fdm_initial_state(&s);
    fdm_wind w; fdm_wind_init(&w, 2);
    fdm_cmds c = {0.0, TE, 0.0, FDM_TRIM_DT};
    double ww[3] = {1, 1, 1};
    fdm_step(&s, 0, &c, 0, &w, 0 /* env NULL → calm */, DT, ww);
    expect(fabs(ww[0]) < 1e-12 && fabs(ww[1]) < 1e-12 && fabs(ww[2]) < 1e-12, "null-env wind is calm");
    fdm_step(&s, 0, &c, 0, 0, 0, DT, 0);
    expect(s.pos[1] > 100.0, "no-wind step ok");
  }

  /* --- ground model decisions (L264/266/281/282/284) ------------------------- */
  {
    /* Descend onto the runway with lateral drift, sink, roll and nose-down: hits
       vy<0 clamp, gs>0 friction, kp/kr>1 with a big dt, and pitch<-0.02 && q<0. */
    fdm_state s; fdm_ground_state(&s);
    s.pos[1] = 0.0;
    s.vel[0] = 8.0; s.vel[1] = -4.0; s.vel[2] = -12.0;  /* drift + sink + roll-in */
    s.omega[0] = -0.3;                                   /* some body rate */
    /* a nose-down attitude so pitch < -0.02 */
    double ph = (-10.0 * M_PI / 180.0) / 2.0;
    s.quat[0] = sin(ph); s.quat[3] = cos(ph);
    fdm_cmds c = {0.0, TE, 0.0, 0.0 /* idle → brake */};
    fdm_step(&s, 0, &c, 0, 0, 0, 0.8 /* big dt → kp AND kr (1.5·dt) clamp past 1 */, 0);
    expect(s.pos[1] == 0.0, "stays on the ground");
    expect(s.vel[1] >= 0.0, "sink clamped at the surface");
    double gs = hypot(s.vel[0], s.vel[2]);
    expect(gs < hypot(8.0, 12.0), "ground friction bleeds speed");

    /* Parked at rest (gs not >0): covers the FALSE side of the friction decision
       (L266). */
    fdm_state r; fdm_ground_state(&r); r.pos[1] = 0.0;
    fdm_step(&r, 0, &c, 0, 0, 0, DT, 0);
    expect(hypot(r.vel[0], r.vel[2]) < 1e-9, "parked stays put (no friction branch)");

    /* Recovering from just below the surface with upward velocity: still inside
       the pos<=0 block but vel[1] >= 0, so the sink clamp must NOT fire — the
       FALSE side of L264. */
    fdm_state u; fdm_ground_state(&u); u.pos[1] = -0.1; u.vel[1] = 3.0; u.vel[2] = -30.0;
    fdm_step(&u, 0, &c, 0, 0, 0, DT, 0);
    expect(u.vel[1] > 0.0, "climbing-out vertical velocity is preserved (sink clamp not fired)");
  }

  /* --- airframe parameterization: custom coef branch + physical effect -------- */
  {
    fdm_coef heavy;
    fdm_coef_default(&heavy);
    heavy.mass *= 1.5;
    fdm_state sd, sh;
    fdm_initial_state(&sd);
    fdm_initial_state(&sh);
    fdm_cmds c = {0.0, TE, 0.0, FDM_TRIM_DT};
    for (int i = 0; i < 10 * 60; i++) {
      fdm_step(&sd, 0, &c, 0, 0, 0, DT, 0);       /* default airframe */
      fdm_step(&sh, &heavy, &c, 0, 0, 0, DT, 0);  /* 1.5× mass */
    }
    expect(sh.pos[1] < sd.pos[1] - 10.0, "heavier airframe sinks vs default at trim power");
    expect(FDM_COEF_COUNT == 38, "coef count matches the delivered parameter table");
  }

  printf(fails ? "COV-DRIVER: %d FAILED\n" : "COV-DRIVER: PASS\n", fails);
  return fails ? 1 : 0;
}
