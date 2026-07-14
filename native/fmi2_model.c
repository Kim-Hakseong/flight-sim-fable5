/* fmi2_model.c — FMI 2.0 Co-Simulation wrapper around the golden-validated FDM
 * core (fdm.c). This is the PRIMARY delivery format: MARKET.md found FMU (not a
 * bespoke .so) is NI VeriStand's officially supported plant-model format and it
 * also covers RTNgine / Simulink / SCADE / dSPACE.
 *
 * Value references are assigned from channels.json (native/gen-fmu.mjs emits
 * modelDescription.xml + the vref table below stays in sync via that generator's
 * consistency check). Only the fmi2 functions VeriStand's co-sim importer needs
 * are implemented; the rest return fmi2OK/unsupported per spec.
 *
 * Self-contained: minimal fmi2 type declarations inline (no FMI SDK headers), so
 * CI builds it with plain gcc. Matches the FMI 2.0 function signatures exactly. */
#include "fdm.h"
#include <stdlib.h>
#include <string.h>

/* --- minimal FMI 2.0 ABI (subset, matches the standard) --------------------- */
typedef void *fmi2Component;
typedef unsigned int fmi2ValueReference;
typedef double fmi2Real;
typedef int fmi2Integer;
typedef int fmi2Boolean;
typedef char fmi2Char;
typedef const fmi2Char *fmi2String;
typedef int fmi2Status;
#define fmi2OK 0
#define fmi2Warning 1
#define fmi2Discard 2
#define fmi2Error 3
#define fmi2True 1
#define fmi2False 0

#define EXPORT __attribute__((visibility("default")))

/* --- value references (MUST match native/gen-fmu.mjs / channels.json) -------- */
/* Inputs 0..11, outputs 100..119. gen-fmu.mjs verifies this mapping. */
enum {
  VR_CMD_A = 0, VR_CMD_E, VR_CMD_R, VR_CMD_T,
  VR_ENV_N, VR_ENV_E, VR_ENV_TURB,
  VR_FLT_A, VR_FLT_E, VR_FLT_R, VR_FLT_T,
  VR_RESET,
  VR_IN_COUNT
};
#define VR_OUT_BASE 100
enum {
  VR_POS_N = VR_OUT_BASE, VR_POS_E, VR_POS_D,
  VR_VEL_N, VR_VEL_E, VR_VEL_D,
  VR_ATT_ROLL, VR_ATT_PITCH, VR_ATT_YAW,
  VR_RATE_P, VR_RATE_Q, VR_RATE_R,
  VR_AIR_VA, VR_AIR_ALPHA, VR_AIR_BETA,
  VR_ACT_A, VR_ACT_E, VR_ACT_R, VR_ACT_T,
  VR_WOW
};

typedef struct {
  fdm_state state;
  fdm_wind wind;
  double in[VR_IN_COUNT];
  double prev_reset;
  char instanceName[64];
} Model;

/* --- lifecycle --------------------------------------------------------------- */
EXPORT fmi2String fmi2GetTypesPlatform(void) { return "default"; }
EXPORT fmi2String fmi2GetVersion(void) { return "2.0"; }

static void model_boot(Model *m, int airborne) {
  if (airborne) fdm_initial_state(&m->state);
  else fdm_ground_state(&m->state);
  fdm_wind_init(&m->wind, 2);
}

EXPORT fmi2Component fmi2Instantiate(fmi2String instanceName, int fmuType,
                                     fmi2String guid, fmi2String resourceLocation,
                                     const void *functions, fmi2Boolean visible,
                                     fmi2Boolean loggingOn) {
  (void)fmuType; (void)guid; (void)resourceLocation;
  (void)functions; (void)visible; (void)loggingOn;
  Model *m = (Model *)calloc(1, sizeof(Model));
  if (!m) return NULL;
  strncpy(m->instanceName, instanceName ? instanceName : "fdm-uav", 63);
  model_boot(m, 0);
  m->in[VR_CMD_T] = 0.0;
  return (fmi2Component)m;
}

EXPORT void fmi2FreeInstance(fmi2Component c) { free(c); }

EXPORT fmi2Status fmi2SetupExperiment(fmi2Component c, fmi2Boolean tolDefined,
                                      fmi2Real tol, fmi2Real startTime,
                                      fmi2Boolean stopDefined, fmi2Real stopTime) {
  (void)c; (void)tolDefined; (void)tol; (void)startTime;
  (void)stopDefined; (void)stopTime;
  return fmi2OK;
}
EXPORT fmi2Status fmi2EnterInitializationMode(fmi2Component c) { (void)c; return fmi2OK; }
EXPORT fmi2Status fmi2ExitInitializationMode(fmi2Component c) { (void)c; return fmi2OK; }
EXPORT fmi2Status fmi2Terminate(fmi2Component c) { (void)c; return fmi2OK; }
EXPORT fmi2Status fmi2Reset(fmi2Component c) {
  Model *m = (Model *)c;
  model_boot(m, 0);
  m->prev_reset = 0.0;
  return fmi2OK;
}

/* --- I/O --------------------------------------------------------------------- */
EXPORT fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[],
                              size_t n, const fmi2Real value[]) {
  Model *m = (Model *)c;
  for (size_t i = 0; i < n; i++)
    if (vr[i] < VR_IN_COUNT) m->in[vr[i]] = value[i];
  return fmi2OK;
}

EXPORT fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[],
                              size_t n, fmi2Real value[]) {
  Model *m = (Model *)c;
  const fdm_state *s = &m->state;
  double roll, pitch, yaw, frd[3], va, al, be, ww[3] = {0, 0, 0};
  fdm_euler(s->quat, &roll, &pitch, &yaw);
  fdm_rates_frd(s->omega, frd);
  /* Report air data against zero wind for a stable, wind-independent Va readout;
   * the internal step already used the true wind. */
  fdm_air_data(s->quat, s->vel, ww, &va, &al, &be);
  for (size_t i = 0; i < n; i++) {
    double out = 0.0;
    switch (vr[i]) {
      case VR_POS_N: out = -s->pos[2]; break;
      case VR_POS_E: out = s->pos[0]; break;
      case VR_POS_D: out = -s->pos[1]; break;
      case VR_VEL_N: out = -s->vel[2]; break;
      case VR_VEL_E: out = s->vel[0]; break;
      case VR_VEL_D: out = -s->vel[1]; break;
      case VR_ATT_ROLL: out = roll; break;
      case VR_ATT_PITCH: out = pitch; break;
      case VR_ATT_YAW: out = yaw; break;
      case VR_RATE_P: out = frd[0]; break;
      case VR_RATE_Q: out = frd[1]; break;
      case VR_RATE_R: out = frd[2]; break;
      case VR_AIR_VA: out = va; break;
      case VR_AIR_ALPHA: out = al; break;
      case VR_AIR_BETA: out = be; break;
      case VR_ACT_A: out = s->act.da; break;
      case VR_ACT_E: out = s->act.de; break;
      case VR_ACT_R: out = s->act.dr; break;
      case VR_ACT_T: out = s->act.dt; break;
      case VR_WOW: out = s->pos[1] <= 0.5 ? 1.0 : 0.0; break;
      default: break;
    }
    value[i] = out;
  }
  return fmi2OK;
}

/* --- co-simulation step ------------------------------------------------------- */
EXPORT fmi2Status fmi2DoStep(fmi2Component c, fmi2Real currentTime,
                             fmi2Real stepSize, fmi2Boolean noSetPrior) {
  (void)currentTime; (void)noSetPrior;
  Model *m = (Model *)c;
  if (m->in[VR_RESET] > 0.5 && m->prev_reset <= 0.5)
    model_boot(m, m->in[VR_RESET] > 1.5);
  m->prev_reset = m->in[VR_RESET];

  fdm_cmds cmd = {m->in[VR_CMD_A], m->in[VR_CMD_E], m->in[VR_CMD_R], m->in[VR_CMD_T]};
  fdm_faults f = {(int)m->in[VR_FLT_A], (int)m->in[VR_FLT_E],
                  (int)m->in[VR_FLT_R], (int)m->in[VR_FLT_T], 0.0};
  fdm_env env = {m->in[VR_ENV_N], m->in[VR_ENV_E], m->in[VR_ENV_TURB]};

  /* Sub-step to the model's fixed 1/60 s base rate if the master hands us a
   * larger communication step, so RT determinism is independent of the caller. */
  const double BASE = 1.0 / 60.0;
  double remaining = stepSize;
  while (remaining > 1e-12) {
    double dt = remaining < BASE ? remaining : BASE;
    fdm_step(&m->state, &cmd, &f, &m->wind, &env, dt, 0);
    remaining -= dt;
  }
  return fmi2OK;
}
