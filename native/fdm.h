/* fdm.h — portable flight-dynamics core (the deployable "yolk").
 *
 * Faithful C99 port of the validated JS reference (src/physics.js + src/wind.js).
 * Dependency-free, double precision, fixed-step, fully deterministic (seeded
 * Gauss–Markov Dryden turbulence). Frames follow the reference:
 *   world: right-handed, +Y up, −Z north/forward; body: +X right wing, +Y top,
 *   −Z nose. FRD helpers convert to aerospace axes at the boundary.
 * Cross-validated against JS golden vectors in CI (native/golden-check.c). */
#ifndef FDM_H
#define FDM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  double da, de, dr; /* surface deflections, rad */
  double dt;         /* throttle 0..1 (actuator, not command) */
} fdm_act;

typedef struct {
  double pos[3];   /* world x, y(up), z */
  double vel[3];   /* world m/s */
  double quat[4];  /* x, y, z, w */
  double omega[3]; /* body rates, rad/s (our axes) */
  fdm_act act;
} fdm_state;

typedef struct {
  double aileron, elevator, rudder; /* commands, −1..1 of max deflection */
  double throttle;                  /* 0..1 */
} fdm_cmds;

/* Per-channel servo faults: 0 none, 1 jam, 2 floating, 3 slow(factor). */
typedef struct {
  int type_da, type_de, type_dr, type_dt;
  double factor; /* slew factor for 'slow' (default 6 when <= 0) */
} fdm_faults;

typedef struct {
  int32_t rng;      /* mulberry32 state */
  double gust[3];   /* body-FRD gust u,v,w m/s */
} fdm_wind;

typedef struct {
  double wind_n, wind_e; /* steady wind TO north / TO east, m/s */
  double turb;           /* Dryden intensity scale (0 = calm) */
} fdm_env;


/* Airframe coefficient set — ALL doubles, so wrappers may index it as a flat
 * double[] (FMI parameter vref 200+i maps to field i; order is gated against
 * native/channels.json by gen-fmu.mjs). Defaults = the golden-validated
 * Aerosonde-class set (FDM_COEF_DEFAULT). Re-target the model to another
 * airframe by supplying your own values — no recompilation. */
typedef struct {
  double mass, Jx, Jy, Jz;
  double wingS, wingB, wingC;
  double CL0, CLa, CLde;
  double CD0, Kind;
  double Cm0, Cma, Cmq, Cmde;
  double CYb, CYdr;
  double Clb, Clp, Clr, Clda, Cldr;
  double Cnb, Cnp, Cnr, Cnda, Cndr;
  double sProp, cProp, kMotor, maxThrustN;
  double muRoll, muBrake;
  double maxDef, actTau, thrTau, alphaClamp;
} fdm_coef;

extern const fdm_coef FDM_COEF_DEFAULT;
void fdm_coef_default(fdm_coef *c);
#define FDM_COEF_COUNT ((int)(sizeof(fdm_coef) / sizeof(double)))

/* Constants mirrored from the JS reference (see src/physics.js AC/TRIM). */
extern const double FDM_TRIM_VA, FDM_TRIM_ALPHA, FDM_TRIM_DE, FDM_TRIM_DT;

void fdm_ground_state(fdm_state *s);            /* runway threshold, cold */
void fdm_initial_state(fdm_state *s);           /* airborne trim fixture */
void fdm_wind_init(fdm_wind *w, int32_t seed);

/* One fixed step: wind Gauss–Markov update + rigid-body integration.
 * wind_world_out (nullable) receives the world wind used this step. */
void fdm_step(fdm_state *s, const fdm_coef *coef, const fdm_cmds *c,
              const fdm_faults *f, fdm_wind *w, const fdm_env *env, double dt,
              double wind_world_out[3]);

/* Introspection used by golden checks and wrappers. */
void fdm_forces_moments(const fdm_state *s, const fdm_coef *coef,
                        const double wind_world[3],
                        double F_out[3], double M_out[3],
                        double *va, double *alpha, double *beta);
void fdm_air_data(const double quat[4], const double vel[3],
                  const double wind_world[3],
                  double *va, double *alpha, double *beta);
void fdm_euler(const double quat[4], double *roll, double *pitch, double *yaw);
void fdm_rates_frd(const double omega[3], double frd_out[3]);
double fdm_air_density(double alt_m);

#ifdef __cplusplus
}
#endif
#endif /* FDM_H */
