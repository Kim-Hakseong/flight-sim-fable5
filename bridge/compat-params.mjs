// QGC compatibility parameter stubs. Our HEARTBEAT claims MAV_AUTOPILOT_ARDUPILOTMEGA
// (so QGC shows proper ArduPlane mode names), which makes QGC's Vehicle Setup pages
// query the standard ArduPilot parameter set. These stubs answer those queries so
// the "parameters missing from firmware" dialog goes away. They live ONLY in the
// bridge (settable + echoed, never forwarded to the sim) — the model's real
// parameter table stays clean (src/params.js).
export const COMPAT_PARAMS = new Map(Object.entries({
  // RC input mapping + calibration (QGC Radio page)
  RCMAP_ROLL: 1, RCMAP_PITCH: 2, RCMAP_THROTTLE: 3, RCMAP_YAW: 4,
  ...Object.fromEntries([1, 2, 3, 4, 5, 6, 7, 8].flatMap((i) => [
    [`RC${i}_MIN`, 1000], [`RC${i}_MAX`, 2000], [`RC${i}_TRIM`, 1500],
  ])),
  // Flight-mode slots (ArduPlane numbering; we honor SET_MODE anyway)
  // (mode 13=TAKEOFF shows as 'unknown' on older QGC mode lists → slot 6 stays Manual)
  FLTMODE1: 0, FLTMODE2: 10, FLTMODE3: 15, FLTMODE4: 11, FLTMODE5: 12, FLTMODE6: 0,
  // Compass/INS identity + calibration. QGC flags a sensor "needs setup" when its
  // offsets are all zero — report plausible NONZERO calibration values so the
  // sim presents as a calibrated vehicle (values are cosmetic; the sim's own
  // sensor model lives in src/sensors.js).
  COMPASS_DEV_ID: 97539, COMPASS_DEV_ID2: 0, COMPASS_DEV_ID3: 0,
  COMPASS_OFS_X: 12.5, COMPASS_OFS_Y: -8.3, COMPASS_OFS_Z: 21.7,
  COMPASS_OFS2_X: 0, COMPASS_OFS2_Y: 0, COMPASS_OFS2_Z: 0,
  COMPASS_OFS3_X: 0, COMPASS_OFS3_Y: 0, COMPASS_OFS3_Z: 0,
  COMPASS_DEC: 0.1466,
  // QGC queries these lazily after the first batch — cover the compass family
  COMPASS_USE: 1, COMPASS_USE2: 0, COMPASS_USE3: 0,
  COMPASS_EXTERNAL: 0, COMPASS_ORIENT: 0, COMPASS_AUTODEC: 1, COMPASS_LEARN: 0,
  INS_GYROFFS_X: 0, INS_GYROFFS_Y: 0, INS_GYROFFS_Z: 0,
  INS_ACCOFFS_X: 0.021, INS_ACCOFFS_Y: -0.013, INS_ACCOFFS_Z: 0.045,
  INS_ACCSCAL_X: 1.001, INS_ACCSCAL_Y: 0.999, INS_ACCSCAL_Z: 1.002,
  // Newer-firmware calibration checks: sensor IDs + calibration temperatures
  // (-300 = uncalibrated sentinel; a plausible temp = calibrated)
  INS_ACC_ID: 1442082, INS_ACC2_ID: 0, INS_ACC3_ID: 0,
  INS_ACC1_CALTEMP: 34.5, INS_ACC2_CALTEMP: -300, INS_ACC3_CALTEMP: -300,
  INS_GYR_ID: 2098184, INS_GYR2_ID: 0, INS_GYR3_ID: 0,
  INS_GYR1_CALTEMP: 34.5, INS_GYR2_CALTEMP: -300, INS_GYR3_CALTEMP: -300,
  // Battery + arming
  BATT_MONITOR: 4, ARMING_CHECK: 1,
  // Geofence (QGC Fence editor). Cosmetic stubs so the fence page loads — the sim
  // enforces the fence GEOMETRY the user draws + uploads (via the fence mission
  // protocol, mission_type=1), NOT these params. In particular FENCE_ALT_MAX is
  // NOT auto-applied: forwarding it silently imposed a 120 m ceiling that
  // RTL-diverted any higher flight. FENCE_TYPE 7 = alt+circle+polygon.
  FENCE_ENABLE: 0, FENCE_TYPE: 7, FENCE_ACTION: 1, FENCE_TOTAL: 0,
  FENCE_ALT_MAX: 120, FENCE_ALT_MIN: -10, FENCE_RADIUS: 300, FENCE_MARGIN: 2,
  FENCE_RET_RALLY: 0, FENCE_AUTOENABLE: 0,
}));
