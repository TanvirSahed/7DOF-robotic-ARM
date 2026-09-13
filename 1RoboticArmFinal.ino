/* =============================================================================
   7-DOF Robotic Arm — PD position control

   Control law per joint, evaluated every DT_MS:
       e = r - y
       u = Kp*e + Kd*(de)
       command = clamp(r + u)
   where r is the trajectory setpoint for this tick and y is the joint angle
   measured through the servo feedback wire on the analog input.

   PD stays disabled until the ADC-to-degree constants below are filled in.
   With it disabled, every joint command passes straight through and the arm
   runs purely open-loop.

   ---------------------------------------------------------------------------
   BRING-UP ORDER
     1. CALIB_MODE 1      -> run, record printed SLOPE/OFFSET, paste below
     2. CALIBRATION_DONE 1, CALIB_MODE 0
     3. STEP_TEST_MODE 1  -> log step responses, tune KP then KD
     4. STEP_TEST_MODE 0  -> normal operation with PD active
   ---------------------------------------------------------------------------
   WIRING: signal pins 2..8 drive s1..s7. Feedback wires land on A0..A6 for
   s1..s7. Verify this mapping with a meter before enabling PD — a swapped
   pair means each joint reads the other's angle and the correction pushes
   the wrong way.
============================================================================= */

#include <Servo.h>

/* ----------------------------- BUILD OPTIONS ----------------------------- */
#define CALIBRATION_DONE  0   // 1 after pasting measured SLOPE/OFFSET values
#define CALIB_MODE        0   // 1 = run calibration sweep instead of main loop
#define STEP_TEST_MODE    0   // 1 = run step-response logger instead of main loop
#define KD_TIME_BASED     0   // 0 = de per sample, 1 = de/dt per second
#define RESET_SEQ_STATE   0   // 1 = allow repeat picks after a full cycle

/* ------------------------------ PD GAINS --------------------------------- */
/* Kd scale depends on KD_TIME_BASED:
     0 -> Kd multiplies (e - ePrev)          [deg per deg/sample]
     1 -> Kd multiplies (e - ePrev)/DT_S     [deg per deg/s]
   At DT = 20 ms the two forms differ by a factor of 50, so a Kd tuned for
   one is meaningless in the other. 0.7 belongs to the per-sample form.      */
const float KP = 2.2f;
const float KD = 0.7f;

/* --------------------------- CONTROL TIMING ------------------------------ */
const uint16_t DT_MS       = 20;     // control period, aligned to the 50 Hz servo frame
const float    DT_S        = DT_MS / 1000.0f;
const float    MS_PER_DEG  = 10.0f;  // ramp rate: 10 ms of travel per degree
const uint16_t SETTLE_MS   = 100;    // hold the endpoint under PD to drive out residual error
const uint8_t  FB_SAMPLES  = 8;      // ADC averages per read; the pot is noisy while driven
const float    DEADBAND    = 1.5f;   // deg; below MG996R resolution, treat as zero error
const float    D_ALPHA     = 0.80f;  // derivative low-pass; 0 = none, 0.9 = heavy
const float    U_CLAMP     = 25.0f;  // deg; ceiling on the correction term itself

/* ------------------------------- JOINTS ---------------------------------- */
const uint8_t NJ = 7;
enum { J1 = 0, J2, J3, J4, J5, J6, J7 };

Servo sv[NJ];
const uint8_t SIG_PIN[NJ] = { 2, 3, 4, 5, 6, 7, 8 };
const uint8_t FB_PIN[NJ]  = { A0, A1, A2, A3, A4, A5, A6 };

/* Soft limits, set from the angle range each joint actually uses plus margin.
   These bound the PD output so a large transient error cannot drive a joint
   into its end stop.                                                        */
const float LIM_LO[NJ] = {   0.0f,  20.0f,  20.0f,   0.0f,  60.0f,   0.0f, 110.0f };
const float LIM_HI[NJ] = { 180.0f, 160.0f, 160.0f, 180.0f, 180.0f, 180.0f, 180.0f };

/* Home pose, one entry per joint. */
const float HOME[NJ] = { 90.0f, 30.0f, 150.0f, 90.0f, 180.0f, 0.0f, 180.0f };

/* ------------------------ FEEDBACK CALIBRATION --------------------------- */
/* Converts the raw 0..1023 ADC reading on the feedback wire into degrees:
       angle_deg = SLOPE[j] * raw + OFFSET[j]
   Values below are placeholders. Run CALIB_MODE and replace them.           */

// float SLOPE[NJ]  = { 0.461f, 0.493f, 0.414f, 0.325f, 0.386f, 0.473f, 0.466f }; // my calib constants 
// float OFFSET[NJ] = { -88.1f, -80.3f, -86.6f, -98.3f, -94.6f, -82.5f, -84.1f }; // my calib constants
float SLOPE[NJ]  = { 0.450f, 0.450f, 0.450f, 0.450f, 0.450f, 0.450f, 0.450f }; // replace with your recorded value and turn the CALIBRATION_DONE flag 1
float OFFSET[NJ] = { -45.0f, -45.0f, -45.0f, -45.0f, -45.0f, -45.0f, -45.0f }; // replace with your recorded value and turn the CALIBRATION_DONE flag 1

/* ------------------------------ STATE ------------------------------------ */
float cmd[NJ];              // last angle written to each joint
float ePrev[NJ];            // previous error, per joint
float dFilt[NJ];            // filtered derivative, per joint
bool  pdActive = false;     // gated on calibration constants being real

String names;               // incoming control character from the host
unsigned int tym;           // window timer for the serial listen phases
int x = 0;                  // bottle sequence step
int y = 0;                  // cup sequence step

/* ========================= LOW-LEVEL PRIMITIVES ========================== */

static float readDeg(uint8_t j) {
  uint16_t acc = 0;
  for (uint8_t k = 0; k < FB_SAMPLES; k++) acc += analogRead(FB_PIN[j]);
  float raw = (float)acc / (float)FB_SAMPLES;
  return SLOPE[j] * raw + OFFSET[j];
}

static void resetPD(uint8_t j) { ePrev[j] = 0.0f; dFilt[j] = 0.0f; }
static void resetPDAll()       { for (uint8_t j = 0; j < NJ; j++) resetPD(j); }

/* One PD evaluation. r is this tick's setpoint; returns the angle to command.
   Returns r untouched when PD is disabled.                                  */
static float pdCorrect(uint8_t j, float r) {
  if (!pdActive) return r;

  float y_meas = readDeg(j);
  float e = r - y_meas;
  if (fabs(e) < DEADBAND) e = 0.0f;

#if KD_TIME_BASED
  float dE = (e - ePrev[j]) / DT_S;
#else
  float dE = (e - ePrev[j]);
#endif
  /* Raw differencing at 50 Hz turns pot noise into a large fake derivative,
     so the term is low-passed before Kd is applied.                         */
  dFilt[j] = D_ALPHA * dFilt[j] + (1.0f - D_ALPHA) * dE;
  ePrev[j] = e;

  float u = KP * e + KD * dFilt[j];
  u = constrain(u, -U_CLAMP, U_CLAMP);
  return constrain(r + u, LIM_LO[j], LIM_HI[j]);
}

static void writeJoint(uint8_t j, float angle) {
  int deg = (int)lround(angle);
  deg = constrain(deg, 0, 180);
  sv[j].write(deg);
  cmd[j] = angle;
}

/* Fixed-cadence tick. Absorbs the ADC read time so the control period holds
   at DT_MS no matter how many joints were sampled this cycle.               */
static void tickWait(uint32_t &tNext) {
  while ((int32_t)(millis() - tNext) < 0) { /* spin */ }
  tNext += DT_MS;
}

/* ========================== MOTION PRIMITIVES ============================ */

/* Ramps one joint to target. Duration scales with travel distance, giving a
   constant commanded velocity of one degree per MS_PER_DEG.                 */
void moveJoint(uint8_t j, float target) {
  float start = cmd[j];
  float span  = fabs(target - start);
  uint32_t dur = (uint32_t)(span * MS_PER_DEG);
  if (dur < DT_MS) dur = DT_MS;

  uint32_t t0 = millis();
  uint32_t tNext = t0 + DT_MS;

  for (;;) {
    uint32_t el = millis() - t0;
    float frac = (el >= dur) ? 1.0f : (float)el / (float)dur;
    float r = start + (target - start) * frac;

    writeJoint(j, pdCorrect(j, r));
    if (frac >= 1.0f) break;
    tickWait(tNext);
  }

  uint32_t tEnd = millis() + SETTLE_MS;
  while ((int32_t)(millis() - tEnd) < 0) {
    writeJoint(j, pdCorrect(j, target));
    tickWait(tNext);
  }
  /* Book the logical position as the setpoint, not the corrected command,
     so the next ramp starts from a clean number.                            */
  cmd[j] = target;
}

/* Shoulder pair: J3 mirrors J2 about 180 deg. Both joints get their own PD
   evaluation against their own feedback wire.                               */
void movePair(float target2) {
  float start2 = cmd[J2];
  float span   = fabs(target2 - start2);
  uint32_t dur = (uint32_t)(span * MS_PER_DEG);
  if (dur < DT_MS) dur = DT_MS;

  uint32_t t0 = millis();
  uint32_t tNext = t0 + DT_MS;

  for (;;) {
    uint32_t el = millis() - t0;
    float frac = (el >= dur) ? 1.0f : (float)el / (float)dur;
    float r2 = start2 + (target2 - start2) * frac;
    float r3 = 180.0f - r2;

    writeJoint(J2, pdCorrect(J2, r2));
    writeJoint(J3, pdCorrect(J3, r3));
    if (frac >= 1.0f) break;
    tickWait(tNext);
  }

  uint32_t tEnd = millis() + SETTLE_MS;
  while ((int32_t)(millis() - tEnd) < 0) {
    writeJoint(J2, pdCorrect(J2, target2));
    writeJoint(J3, pdCorrect(J3, 180.0f - target2));
    tickWait(tNext);
  }
  cmd[J2] = target2;
  cmd[J3] = 180.0f - target2;
}

/* Snaps every joint to the home pose. PD state is cleared because the
   commanded position jumps here — a carried-over derivative would fire a
   large spurious correction on the next move.                               */
void rst() {
  for (uint8_t j = 0; j < NJ; j++) {
    sv[j].write((int)HOME[j]);
    cmd[j] = HOME[j];
  }
  resetPDAll();
}

/* ============================== SEQUENCES ================================ */

void pick_bottle() {
  if (x == 0)  { rst(); delay(600); x = 1;  }
  if (x == 1)  { moveJoint(J1, 120); x = 2;  }   //  90 -> 120
  if (x == 2)  { moveJoint(J5,  70); x = 3;  }   // 180 ->  70
  if (x == 3)  { moveJoint(J7, 120); x = 4;  }   // 180 -> 120, gripper closes
  if (x == 4)  { movePair(150);      x = 5;  }   // J2  30 -> 150
  if (x == 5)  { moveJoint(J7, 170); x = 6;  }   // 120 -> 170
  if (x == 6)  { movePair(90);       x = 7;  }   // J2 150 ->  90
  if (x == 7)  { moveJoint(J1, 180); x = 8;  }   // 120 -> 180, swing to drop zone
  if (x == 8)  { movePair(150);      x = 9;  }   // J2  90 -> 150
  if (x == 9)  { moveJoint(J7, 120); x = 10; }   // 170 -> 120, release
  if (x == 10) { rst();              x = 11; }
#if RESET_SEQ_STATE
  if (x == 11) { x = 0; }
#endif
}

void pick_cup() {
  if (y == 0)  { rst(); delay(600); y = 1;  }
  if (y == 1)  { moveJoint(J1,  45); y = 2;  }   //  90 ->  45
  if (y == 2)  { moveJoint(J5,  70); y = 3;  }   // 180 ->  70
  if (y == 3)  { moveJoint(J7, 120); y = 4;  }   // 180 -> 120, gripper closes
  if (y == 4)  { movePair(150);      y = 5;  }   // J2  30 -> 150
  if (y == 5)  { moveJoint(J7, 170); y = 6;  }   // 120 -> 170
  if (y == 6)  { movePair(90);       y = 7;  }   // J2 150 ->  90
  if (y == 7)  { moveJoint(J1,   0); y = 8;  }   //  45 ->   0, swing to drop zone
  if (y == 8)  { movePair(150);      y = 9;  }   // J2  90 -> 150
  if (y == 9)  { moveJoint(J7, 120); y = 10; }   // 170 -> 120, release
  if (y == 10) { rst();              y = 11; }
#if RESET_SEQ_STATE
  if (y == 11) { y = 0; }
#endif
}

/* ============================== SETUP ==================================== */

bool PD_RUNTIME_OK();   // forward declaration

void setup() {
  Serial.begin(115200);

  for (uint8_t j = 0; j < NJ; j++) {
    sv[j].attach(SIG_PIN[j]);
    pinMode(SIG_PIN[j], OUTPUT);
    pinMode(FB_PIN[j], INPUT);
    cmd[j] = HOME[j];
  }
  resetPDAll();

#if CALIBRATION_DONE
  pdActive = PD_RUNTIME_OK();
#else
  pdActive = false;
#endif

  tym = millis();
  delay(200);
  rst();

#if CALIB_MODE || STEP_TEST_MODE
  delay(1000);
#endif
}

/* Refuses to enable PD while the conversion constants are still placeholders.
   Feeding garbage degrees into the error term would slam joints into their
   limits on the first tick.                                                 */
bool PD_RUNTIME_OK() {
  for (uint8_t j = 0; j < NJ; j++) {
    if (fabs(SLOPE[j] - 0.450f) < 1e-6f && fabs(OFFSET[j] + 45.0f) < 1e-6f) {
      Serial.println(F("[WARN] placeholder calibration detected - PD disabled"));
      return false;
    }
  }
  Serial.println(F("[OK] PD active"));
  return true;
}

/* ========================== CALIBRATION ROUTINE ========================== */
/* Parks each joint at three known angles inside its soft limits, averages the
   ADC at each, fits a straight line through the low and high points, and
   prints the constants. The mid point is used only as a linearity check.    */
#if CALIB_MODE
void runCalibration() {
  Serial.println(F("=== FEEDBACK CALIBRATION ==="));
  Serial.println(F("joint, angLo, rawLo, angMid, rawMid, angHi, rawHi, slope, offset, midErrDeg"));

  for (uint8_t j = 0; j < NJ; j++) {
    float aLo  = LIM_LO[j] + 5.0f;
    float aHi  = LIM_HI[j] - 5.0f;
    float aMid = 0.5f * (aLo + aHi);
    float ang[3] = { aLo, aMid, aHi };
    float raw[3];

    for (uint8_t k = 0; k < 3; k++) {
      sv[j].write((int)lround(ang[k]));
      delay(1200);                              // let the joint arrive and stop moving
      uint32_t acc = 0;
      for (uint8_t n = 0; n < 32; n++) { acc += analogRead(FB_PIN[j]); delay(5); }
      raw[k] = (float)acc / 32.0f;
    }

    float slope  = (ang[2] - ang[0]) / (raw[2] - raw[0]);
    float offset = ang[0] - slope * raw[0];
    float midErr = (slope * raw[1] + offset) - ang[1];

    Serial.print(j + 1); Serial.print(F(", "));
    for (uint8_t k = 0; k < 3; k++) {
      Serial.print(ang[k], 1); Serial.print(F(", "));
      Serial.print(raw[k], 1); Serial.print(F(", "));
    }
    Serial.print(slope, 6);  Serial.print(F(", "));
    Serial.print(offset, 3); Serial.print(F(", "));
    Serial.println(midErr, 2);

    sv[j].write((int)HOME[j]);
    delay(600);
  }
  Serial.println(F("=== paste slope/offset columns into SLOPE[] / OFFSET[] ==="));
  Serial.println(F("midErrDeg above ~3 deg means a non-linear pot - use a LUT instead"));
}
#endif

/* ========================= STEP-RESPONSE LOGGER ========================== */
/* Holds one joint at a fixed setpoint after a step and logs r, y, e and the
   command every DT_MS as CSV. Run one joint at a time with the rest parked
   at home, raise KP until the response is quick without overshoot, then
   raise KD to damp what is left.                                            */
#if STEP_TEST_MODE
const uint8_t  TEST_JOINT = J1;
const float    STEP_FROM  = 90.0f;
const float    STEP_TO    = 180.0f;
const uint16_t LOG_MS     = 2000;

void runStepTest() {
  Serial.println(F("t_ms,r,y,e,u"));
  writeJoint(TEST_JOINT, STEP_FROM);
  delay(1000);
  resetPD(TEST_JOINT);

  uint32_t t0 = millis();
  uint32_t tNext = t0 + DT_MS;
  while (millis() - t0 < LOG_MS) {
    float r  = STEP_TO;
    float yv = readDeg(TEST_JOINT);
    float u  = pdCorrect(TEST_JOINT, r);
    writeJoint(TEST_JOINT, u);

    Serial.print(millis() - t0); Serial.print(',');
    Serial.print(r, 2);          Serial.print(',');
    Serial.print(yv, 2);         Serial.print(',');
    Serial.print(r - yv, 2);     Serial.print(',');
    Serial.println(u, 2);

    tickWait(tNext);
  }
  Serial.println(F("=== step test complete ==="));
}
#endif

/* =============================== LOOP ==================================== */
/* Listens for a class character from the host for 5 s, runs the matching
   sequence, then listens again for a second object.

   x and y terminate at 11 and are not cleared, so each object is picked once
   per power cycle. Set RESET_SEQ_STATE 1 for repeat picks.                  */

void loop() {

#if CALIB_MODE
  runCalibration();
  while (1) { }
#elif STEP_TEST_MODE
  runStepTest();
  while (1) { }
#else

  while ((millis() - tym) <= 5000) {
    if (Serial.available()) {
      names = Serial.readStringUntil('\r');
    }
  }
  tym = millis();

  if (names == "a") {
    pick_bottle();
    tym = millis();
    while ((millis() - tym) <= 5000) {
      if (Serial.available()) names = Serial.readStringUntil('\r');
    }
    tym = millis();
    if (names == "c") { delay(2000); pick_cup(); }
  }

  else if (names == "b") {
    pick_bottle();
    tym = millis();
    while ((millis() - tym) <= 5000) {
      if (Serial.available()) names = Serial.readStringUntil('\r');
    }
    tym = millis();
    if (names == "c")      { delay(2000); pick_cup();    }
    else if (names == "b") { delay(2000); pick_bottle(); }
  }

  else if (names == "c") {
    pick_cup();
    tym = millis();
    while ((millis() - tym) <= 5000) {
      if (Serial.available()) names = Serial.readStringUntil('\r');
    }
    tym = millis();
    if (names == "b")      { delay(2000); pick_bottle(); }
    else if (names == "c") { delay(2000); pick_cup();    }
  }

#endif
}
