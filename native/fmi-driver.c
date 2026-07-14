/* fmi-driver.c — loads the built FMU shared library via dlopen and drives it
 * through the real FMI 2.0 co-simulation call sequence, then prints the final
 * state as JSON for fmi-check.mjs to compare against a JS golden trajectory.
 * This exercises the ACTUAL exported fmi2 ABI, not the C core directly. */
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void *(*instantiate_t)(const char *, int, const char *, const char *,
                               const void *, int, int);
typedef int (*setup_t)(void *, int, double, double, int, double);
typedef int (*initmode_t)(void *);
typedef int (*setreal_t)(void *, const unsigned int *, size_t, const double *);
typedef int (*getreal_t)(void *, const unsigned int *, size_t, double *);
typedef int (*dostep_t)(void *, double, double, int);
typedef void (*free_t)(void *);

int main(int argc, char **argv) {
  if (argc < 4) { fprintf(stderr, "usage: fmi-driver <so> <case> <seconds>\n"); return 2; }
  const char *sopath = argv[1];
  const char *scase = argv[2];
  int seconds = atoi(argv[3]);

  void *h = dlopen(sopath, RTLD_NOW);
  if (!h) { fprintf(stderr, "dlopen: %s\n", dlerror()); return 2; }
  instantiate_t f_inst = (instantiate_t)dlsym(h, "fmi2Instantiate");
  setup_t f_setup = (setup_t)dlsym(h, "fmi2SetupExperiment");
  initmode_t f_enter = (initmode_t)dlsym(h, "fmi2EnterInitializationMode");
  initmode_t f_exit = (initmode_t)dlsym(h, "fmi2ExitInitializationMode");
  setreal_t f_set = (setreal_t)dlsym(h, "fmi2SetReal");
  getreal_t f_get = (getreal_t)dlsym(h, "fmi2GetReal");
  dostep_t f_step = (dostep_t)dlsym(h, "fmi2DoStep");
  free_t f_free = (free_t)dlsym(h, "fmi2FreeInstance");
  if (!f_inst || !f_set || !f_get || !f_step) {
    fprintf(stderr, "missing fmi2 symbols\n"); return 2;
  }

  void *c = f_inst("fdm-uav", 1 /*CoSimulation*/, "guid", "", NULL, 0, 0);
  if (!c) { fprintf(stderr, "instantiate failed\n"); return 2; }
  f_setup(c, 0, 0.0, 0.0, 0, 0.0);
  f_enter(c);
  f_exit(c);

  /* Input vrefs: 0..11 per channels.json / fmi2_model.c enum. */
  enum { CMD_A = 0, CMD_E, CMD_R, CMD_T, ENV_N, ENV_E, ENV_TURB,
         FLT_A, FLT_E, FLT_R, FLT_T, RESET };
  const double DT = 1.0 / 60.0;
  const double TRIM_ELEV = -0.08906 / 0.44, TRIM_DT = 0.62747;

  /* case selects boot + command law, mirroring native/gen-golden trajectories. */
  int airborne = strcmp(scase, "ground_roll") != 0;
  unsigned int vr_reset[1] = {RESET};
  double reset_v[1] = { airborne ? 2.0 : 1.0 };
  f_set(c, vr_reset, 1, reset_v); /* rising edge on first step boots the mode */

  int n = seconds * 60;
  for (int i = 0; i < n; i++) {
    double in[12] = {0};
    in[RESET] = airborne ? 2.0 : 1.0;
    if (strcmp(scase, "ground_roll") == 0) {
      in[CMD_E] = 0.0; in[CMD_T] = 1.0;
    } else { /* trim_calm */
      in[CMD_E] = TRIM_ELEV; in[CMD_T] = TRIM_DT;
    }
    unsigned int vr[12];
    for (unsigned int k = 0; k < 12; k++) vr[k] = k;
    f_set(c, vr, 12, in);
    f_step(c, i * DT, DT, 1);
  }

  unsigned int ovr[20];
  for (unsigned int k = 0; k < 20; k++) ovr[k] = 100 + k;
  double out[20];
  f_get(c, ovr, 20, out);
  /* Print PosN,PosE,PosD, Va, Roll,Pitch,Yaw as JSON. */
  printf("{\"posN\":%.17g,\"posE\":%.17g,\"posD\":%.17g,\"va\":%.17g,"
         "\"roll\":%.17g,\"pitch\":%.17g,\"yaw\":%.17g}\n",
         out[0], out[1], out[2], out[12], out[6], out[7], out[8]);
  if (f_free) f_free(c);
  dlclose(h);
  return 0;
}
