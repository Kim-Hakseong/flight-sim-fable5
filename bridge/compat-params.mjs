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
  // Compass/INS identity + calibration (0 = none/uncalibrated is fine for a sim)
  COMPASS_DEV_ID: 1, COMPASS_DEV_ID2: 0, COMPASS_DEV_ID3: 0,
  COMPASS_OFS_X: 0, COMPASS_OFS_Y: 0, COMPASS_OFS_Z: 0,
  COMPASS_OFS2_X: 0, COMPASS_OFS2_Y: 0, COMPASS_OFS2_Z: 0,
  COMPASS_OFS3_X: 0, COMPASS_OFS3_Y: 0, COMPASS_OFS3_Z: 0,
  COMPASS_DEC: 0,
  // QGC queries these lazily after the first batch — cover the compass family
  COMPASS_USE: 1, COMPASS_USE2: 0, COMPASS_USE3: 0,
  COMPASS_EXTERNAL: 0, COMPASS_ORIENT: 0, COMPASS_AUTODEC: 1, COMPASS_LEARN: 0,
  INS_GYROFFS_X: 0, INS_GYROFFS_Y: 0, INS_GYROFFS_Z: 0,
  INS_ACCOFFS_X: 0, INS_ACCOFFS_Y: 0, INS_ACCOFFS_Z: 0,
  // Battery + arming
  BATT_MONITOR: 4, ARMING_CHECK: 1,
}));
