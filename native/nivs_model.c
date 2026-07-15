/* nivs_model.c — NI VeriStand Model Framework wrapper around the golden-
 * validated FDM core (fdm.c). Channel map is generated from channels.json
 * (native/gen-spec.mjs emits INTERFACE.md from the same source of truth).
 *
 * Deployment (customer toolchain, VeriStand Linux RT target):
 *   gcc -std=c99 -O2 -fPIC -shared -I<NIVS_SDK> nivs_model.c fdm.c \
 *       <NIVS_SDK>/ni_modelframework.c -o fdm-uav.so -lm
 * CI compiles against native/ni_stub/ (types only) to keep the wrapper honest. */
#include "ni_modelframework.h"
#include "fdm.h"
#include <string.h>

/* --- Channel structs (order MUST match channels.json) ---------------------- */
typedef struct {
  double Cmd_Aileron, Cmd_Elevator, Cmd_Rudder, Cmd_Throttle;
  double Env_WindN, Env_WindE, Env_Turb;
  double Flt_Aileron, Flt_Elevator, Flt_Rudder, Flt_Throttle;
  double Sim_Reset;
} Inports;

typedef struct {
  double Pos_N, Pos_E, Pos_D;
  double Vel_N, Vel_E, Vel_D;
  double Att_Roll, Att_Pitch, Att_Yaw;
  double Rate_P, Rate_Q, Rate_R;
  double Air_Va, Air_Alpha, Air_Beta;
  double Act_Aileron, Act_Elevator, Act_Rudder, Act_Throttle;
  double WoW;
} Outports;

/* --- Framework metadata ------------------------------------------------------ */
static const double BASE_DT = 1.0 / 60.0;
const char *USER_ModelName = "fdm-uav";
const char *USER_Builder = "flight-sim-fable5 (golden-validated against the JS reference)";
double USER_BaseRate = 1.0 / 60.0;
NI_Task rtTaskAttribs = {0, 1.0 / 60.0, 0.0};

int32_t InportSize = (int32_t)(sizeof(Inports) / sizeof(double));
int32_t OutportSize = (int32_t)(sizeof(Outports) / sizeof(double));
int32_t ParameterSize = 0;
int32_t SignalSize = 0;
NI_Parameter rtParamAttribs[1];
NI_Signal rtSignalAttribs[1];
int32_t ParamDimList[1];
int32_t SigDimList[1];

/* --- Model state --------------------------------------------------------------- */
static fdm_state g_state;
static fdm_wind g_wind;
static double g_prev_reset = 0.0;

static void boot(int airborne) {
  if (airborne) fdm_initial_state(&g_state);
  else fdm_ground_state(&g_state);
  fdm_wind_init(&g_wind, 2);
}

int32_t USER_Initialize(void) {
  boot(0); /* like the real vehicle: cold on the runway */
  g_prev_reset = 0.0;
  return NI_OK;
}

int32_t USER_ModelStart(void) { return NI_OK; }

int32_t USER_TakeOneStep(double *inData, double *outData, double timestamp) {
  (void)timestamp;
  const Inports *in = (const Inports *)inData;
  Outports *out = (Outports *)outData;

  if (in->Sim_Reset > 0.5 && g_prev_reset <= 0.5) boot(in->Sim_Reset > 1.5);
  g_prev_reset = in->Sim_Reset;

  fdm_cmds c = {in->Cmd_Aileron, in->Cmd_Elevator, in->Cmd_Rudder, in->Cmd_Throttle};
  fdm_faults f = {(int)in->Flt_Aileron, (int)in->Flt_Elevator,
                  (int)in->Flt_Rudder, (int)in->Flt_Throttle, 0.0};
  fdm_env env = {in->Env_WindN, in->Env_WindE, in->Env_Turb};
  double ww[3];
  fdm_step(&g_state, 0 /* default airframe; parameterized path = FMU */, &c, &f, &g_wind, &env, BASE_DT, ww);

  /* ours → NED: N = −z, E = +x, D = −y */
  out->Pos_N = -g_state.pos[2];
  out->Pos_E = g_state.pos[0];
  out->Pos_D = -g_state.pos[1];
  out->Vel_N = -g_state.vel[2];
  out->Vel_E = g_state.vel[0];
  out->Vel_D = -g_state.vel[1];
  fdm_euler(g_state.quat, &out->Att_Roll, &out->Att_Pitch, &out->Att_Yaw);
  {
    double frd[3];
    fdm_rates_frd(g_state.omega, frd);
    out->Rate_P = frd[0]; out->Rate_Q = frd[1]; out->Rate_R = frd[2];
  }
  fdm_air_data(g_state.quat, g_state.vel, ww, &out->Air_Va, &out->Air_Alpha, &out->Air_Beta);
  out->Act_Aileron = g_state.act.da;
  out->Act_Elevator = g_state.act.de;
  out->Act_Rudder = g_state.act.dr;
  out->Act_Throttle = g_state.act.dt;
  out->WoW = g_state.pos[1] <= 0.5 ? 1.0 : 0.0;
  return NI_OK;
}

int32_t USER_Finalize(void) { return NI_OK; }
