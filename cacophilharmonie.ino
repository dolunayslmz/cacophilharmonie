#include <Arduino.h>

// --- Forward declare to satisfy Arduino's auto-prototype generation ---
struct Voice;

/* ========================= USER CONTROLS (EDIT ME) =========================
   Uno R4 PWM pins: 3, 5, 6, 9, 10, 11
   Pin plan:
   - Vibration motors: 9, 10, 11, 3  (all PWM via MOSFETs)
   - Stepper: DIR=4, STEP=12
   - Reactive DC "percussion": 6  (PWM) + optional bursts
   - Constant DC windowed: 7
   - Gearmotor: 8  (windows; while ON it pulses; 100% in finale)
   - Solenoid (no diode): 5 (PWM) with soft start/stop & 7V cap
*/

// --- Debug / Tempo ---
bool     SERIAL_DEBUG          = false;
uint16_t BPM                   = 96;
uint8_t  PPQN                  = 24;
uint32_t pieceStartMs          = 0;

// --- Piezo trigger ---
const int piezoPin             = A0;
int      triggerThreshold      = 20;
int      hysteresis            = 10;
uint16_t baselineSamples       = 100;
uint32_t lockoutMs             = 20000;

// --- Stepper program (scaled to 95%) + 2s pauses ---
// Base shape {6,6,12}; UP is two segments, DOWN should equal sum(UP) after scaling.
uint16_t stepperSegCount       = 3;
int      stepperTurnsBase[]    = { 6, 6, 12 };
int      stepperTurns[3];                 // computed in setup
float    stepperScale          = 0.95f;   // <--- 95% as requested
uint32_t stepperDurMs[]        = { 3000, 3000, 3000 };
bool     stepperDirCW[]        = { true, true, false };
uint32_t stepperPauseMs        = 2000;

// --- Fanout after stepper completes ---
uint32_t fanoutMs              = 1000;

// --- Piece length (~1 minute): reprise at ~48s, then ~9s finale + 1 bar wrap ---
uint32_t repriseAfterMs        = 48000UL; // trigger final sequence around 48 s
uint8_t  busyBars              = 1;       // one bar (~2.5 s @ 96 BPM) for the wrap

// --- Vibration motor envelopes (0..255) for pins 9/10/11/3 ---
struct ADSR { uint16_t attackMs, decayMs; uint8_t sustainLvl; uint16_t releaseMs; };
ADSR vibEnv[4] = {
  {  40, 180, 140, 120 },  // 9
  {  10, 120, 120,  90 },  // 10
  { 120, 300, 180, 250 },  // 11
  {  60, 140, 100, 120 },  // 3
};

// Euclidean settings per vib (16-step)
struct Euclid { uint8_t pulses, steps, rotate; };
Euclid vibEuclid[4] = {
  { 4, 16, 0 },   // 9
  { 8, 16, 0 },   // 10
  { 5, 16, 2 },   // 11
  { 7, 16, 1 },   // 3
};

// --- M6 reactive DC (baseline envelope) + (optional) bursts every 10s ---
ADSR motor6Env = { 200, 800, 200, 1200 };
uint32_t m6BurstIntervalMs = 10000UL;
uint16_t m6BurstDurationMs = 600;
uint8_t  m6BurstLevel      = 255;
bool     m6BurstActive     = false;
uint32_t m6NextBurstAt     = 0;
uint32_t m6BurstEndAt      = 0;

// --- M7 constant DC (windowed ON/OFF) ---
uint16_t m7OnMinSec  = 5,  m7OnMaxSec  = 15;
uint16_t m7OffMinSec = 5,  m7OffMaxSec = 15;

// --- Gearmotor on 8: windows + in-window pulses (short gap) ---
const int GEAR_PIN = 8;
// Base window durations
const unsigned long GEAR_ON_MS_BASE   = 2000;   // default ON window
const unsigned long GEAR_OFF_MS_BASE  = 3000;   // default OFF window
// Guarantee at least two long ON windows with >= 4 pulses
const unsigned long GEAR_LONG_ON_MS   = 5000;   // long ON (>= 4 pulses at ~0.55s period)
uint8_t  gearLongOnRemaining          = 2;

bool     gearOnWindow     = false;     // are we inside an ON window?
uint32_t gearWindowStart  = 0;
uint32_t gearCurrentOnMs  = GEAR_ON_MS_BASE;
uint32_t gearCurrentOffMs = GEAR_OFF_MS_BASE;

// In-window pulse dynamics: keep ON width, shorten the gap
const unsigned long GEAR_PULSE_WIDTH_MS  = 400;  // ON time (unchanged)
const unsigned long GEAR_PULSE_GAP_MS    = 150;  // OFF time between pulses (shorter)
const unsigned long GEAR_PULSE_PERIOD_MS = GEAR_PULSE_WIDTH_MS + GEAR_PULSE_GAP_MS; // 550 ms
bool     gearPulseActive    = false;
uint32_t gearPulsePrevMs    = 0;
uint32_t gearPulseStartMs   = 0;

// Finale override (still forces 100%)
bool     gearForceFull      = false;

// --- Solenoid (no diode!) on PWM pin 5 with soft ramps and 7V cap ---
const int SOL_PIN = 5;          // PWM
// Coil fed by 12V supply → to emulate ≤7V, cap duty ≈ 7/12 ≈ 0.58
uint8_t  solDutyCap        = 148;      // 58% of 255 ≈ 148 (max allowed duty)
uint8_t  solPullDuty       = 148;      // pull-in duty (capped)
uint8_t  solHoldDuty       = 110;      // hold duty (keep sound but lower stress)
uint16_t solRampUpMs       = 20;       // soft-start time
uint16_t solRampDownMs     = 40;       // soft-stop time (longer = safer)
uint16_t solMinOffMs       = 80;       // min time before next hit (avoid chatter)
uint16_t solMinOnMs        = 60;       // min time on (avoid buzz)
uint16_t solHitOnMs        = 90;       // percussive hit on-time before ramp down

/* ===== Pins ===== */
int vibPins[4]   = { 9, 10, 11, 3 };   // vib motors (ALL PWM)
const int DIR_PIN= 4;                  // stepper DIR
const int STEP_PIN=12;                 // stepper STEP
const int M6_PIN = 6;                  // reactive DC (PWM)
const int M7_PIN = 7;                  // constant DC (digital/PWM)

// --- Stepper timings ---
const uint16_t STEPS_PER_REV = 400;
const uint16_t STEP_PULSE_US = 10;
const uint16_t DIR_SETUP_US  = 20;

/* ========================= INTERNALS ========================= */

static inline bool isPWMPin(int p){ return (p==3 || p==5 || p==6 || p==9 || p==10 || p==11); }

// ----- Voice type (must precede helpers) -----
struct Voice {
  bool     active   = false;
  bool     gate     = false;
  uint32_t gateOnMs = 0;
  uint32_t gateOffMs= 0;
  uint8_t  level    = 0;
  ADSR     env;
  int      pin;
  bool     pwmCapable = true;
  uint32_t strikeEndMs = 0;
};

// Vib calibration (to even out loudness across motors 9/10/11/3)
float   vibGain[4]   = {1.35, 1.00, 1.50, 1.60};
uint8_t vibFloor[4]  = {  60,   40,   70,   70};
uint16_t vibStrikeMs = 50;

// Helpers
static inline void writeLevel(Voice &v, uint8_t lvl){
  v.level = lvl;
  if (v.pwmCapable) analogWrite(v.pin, lvl);
  else digitalWrite(v.pin, (lvl > 0) ? HIGH : LOW);
}
static inline void gateOn (Voice &v, uint32_t now){ v.gate = true;  v.active = true;  v.gateOnMs  = now; v.strikeEndMs = now + vibStrikeMs; }
static inline void gateOff(Voice &v, uint32_t now){ v.gate = false; v.gateOffMs = now; }
static inline uint8_t computeADSR(const ADSR &e, bool gate, uint32_t onMs, uint32_t offMs, uint32_t nowMs){
  if (gate) {
    uint32_t t = nowMs - onMs;
    if (t <= e.attackMs) return (uint8_t)min<uint32_t>(255, (t * 255UL) / max<uint16_t>(1, e.attackMs));
    t -= e.attackMs;
    if (t <= e.decayMs) {
      uint8_t from = 255, to = e.sustainLvl;
      return (uint8_t)(from + (int32_t)(to - from) * (int32_t)t / (int32_t)max<uint16_t>(1, e.decayMs));
    }
    return e.sustainLvl;
  } else {
    uint32_t t = nowMs - offMs;
    if (t >= e.releaseMs) return 0;
    uint8_t start = e.sustainLvl;
    int32_t val = (int32_t)start - ((int32_t)start * (int32_t)t / (int32_t)max<uint16_t>(1, e.releaseMs));
    return (uint8_t)max<int32_t>(0, val);
  }
}

// Drive vib with gain/floor/strike
static inline void driveVibImpl(Voice &vv, uint8_t idx, uint32_t now){
  uint8_t base = computeADSR(vv.env, vv.gate, vv.gateOnMs, vv.gateOffMs, now);
  uint16_t eff = (uint16_t)((float)base * vibGain[idx]);
  if (eff > 0 && eff < vibFloor[idx]) eff = vibFloor[idx];
  if (vv.gate && now <= vv.strikeEndMs) eff = 255;
  if (eff > 255) eff = 255;
  writeLevel(vv, (uint8_t)eff);
  if (!vv.gate && eff == 0) vv.active = false;
}

// Random window helper
uint32_t rndMillis(uint16_t sMin, uint16_t sMax){ if (sMax < sMin) sMax = sMin; return (uint32_t)random(sMin, sMax+1) * 1000UL; }

// Euclid generator (16)
void fillEuclid(bool *out16, uint8_t pulses, uint8_t steps, uint8_t rotate){
  const uint8_t N = 16;
  bool tmp[N] = {0};
  steps  = constrain(steps, 1, N);
  pulses = constrain(pulses, 0, steps);
  for (uint8_t i=0; i<steps; i++) tmp[i] = ((i*pulses)/steps) != (((i+1)*pulses)/steps);
  for (uint8_t i=0; i<N; i++) out16[(i + (rotate & (N-1))) & (N-1)] = (i < steps) ? tmp[i] : false;
}

// Stepper helpers
inline void pulseStep(uint16_t hi, uint16_t lo){ digitalWrite(STEP_PIN, HIGH); delayMicroseconds(hi); digitalWrite(STEP_PIN, LOW); delayMicroseconds(lo); }
void rotateTurns(int turns, uint32_t durMs, bool cw){
  uint32_t steps = (uint32_t)abs(turns) * STEPS_PER_REV; if (!steps || !durMs) return;
  digitalWrite(DIR_PIN, cw ? HIGH : LOW); delayMicroseconds(DIR_SETUP_US);
  uint32_t period = (durMs * 1000UL) / steps; if (period <= STEP_PULSE_US) period = STEP_PULSE_US + 1;
  uint16_t low_us = (uint16_t)(period - STEP_PULSE_US);
  for (uint32_t i=0; i<steps; i++) pulseStep(STEP_PULSE_US, low_us);
}
void runStepperScaled(){
  for (uint16_t i=0; i<stepperSegCount; i++) {
    rotateTurns(stepperTurns[i], stepperDurMs[i], stepperDirCW[i]);
    if (i < stepperSegCount - 1) delay(stepperPauseMs);
  }
}

// Fanout
void doFanout(){
  uint32_t t0 = millis();
  analogWrite(M6_PIN, 255);
  digitalWrite(M7_PIN, HIGH);
  while (millis() - t0 < fanoutMs) delay(1);
  analogWrite(M6_PIN, 0);
  digitalWrite(M7_PIN, LOW);
}

/* ---------- Solenoid soft controller (Pin 5 PWM) ---------- */
enum SolState { SOL_IDLE, SOL_RAMP_UP, SOL_HOLD, SOL_RAMP_DOWN };
SolState solState = SOL_IDLE;
uint8_t  solCurrentDuty = 0;
uint32_t solPhaseStart  = 0;
uint32_t solLastChange  = 0;

static inline uint8_t clampDuty(uint8_t d){ return (d > solDutyCap) ? solDutyCap : d; }
static inline void solWrite(uint8_t duty){ analogWrite(SOL_PIN, clampDuty(duty)); }

void solBeginHit(uint32_t now){
  if (now - solLastChange < solMinOffMs && solState != SOL_IDLE) return;
  solState = SOL_RAMP_UP; solPhaseStart = now; solLastChange = now;
}

void solUpdate(uint32_t now){
  switch (solState){
    case SOL_IDLE:
      solCurrentDuty = 0; solWrite(0);
      break;
    case SOL_RAMP_UP: {
      uint32_t t = now - solPhaseStart;
      if (t >= solRampUpMs){
        solCurrentDuty = solPullDuty;
        solWrite(solCurrentDuty);
        solState = SOL_HOLD; solPhaseStart = now;
      } else {
        uint8_t d = (uint8_t)((uint32_t)solPullDuty * t / max<uint16_t>(1, solRampUpMs));
        solCurrentDuty = d; solWrite(d);
      }
    } break;
    case SOL_HOLD: {
      uint32_t t = now - solPhaseStart;
      if (t >= solHitOnMs && t >= solMinOnMs){
        solCurrentDuty = solHoldDuty; solWrite(solCurrentDuty);
        solState = SOL_RAMP_DOWN; solPhaseStart = now;
      } else {
        solWrite(solPullDuty);
      }
    } break;
    case SOL_RAMP_DOWN: {
      uint32_t t = now - solPhaseStart;
      if (t >= solRampDownMs){
        solCurrentDuty = 0; solWrite(0);
        solState = SOL_IDLE; solLastChange = now;
      } else {
        uint8_t d = (uint8_t)((uint32_t)solHoldDuty * (solRampDownMs - t) / max<uint16_t>(1, solRampDownMs));
        solCurrentDuty = d; solWrite(d);
      }
    } break;
  }
}

// Quick API for a percussive “click”
void solHit(){ solBeginHit(millis()); }

/* -------------------- Runtime state -------------------- */

const uint8_t STEPS_PER_BAR = 16;
bool vibPattern[4][STEPS_PER_BAR];

const int numReadings = 5;
int readings[numReadings] = {0};
int readIndex = 0; long total = 0; int smoothed = 0; int baseline = 0;

enum SystemState { IDLE, STEPPER_RUNNING, FANOUT_STATE, SEQUENCING, REPRISE_STEPPER, BUSY_CRESCENDO };
SystemState state = IDLE;
uint32_t stateStartMs = 0, lockoutUntil = 0;

uint32_t lastTickMs = 0, ticks = 0;
uint32_t tickIntervalMs(){ return max<uint32_t>(1, (60000UL / BPM) / PPQN); }
uint32_t barsElapsed = 0;

Voice vib[4], m6;
bool     m7IsOn = false;
uint32_t m7WindowEndMs = 0;

/* ========================= SETUP ========================= */
void setup(){
  if (SERIAL_DEBUG) { Serial.begin(115200); delay(50); }
  randomSeed(analogRead(A1));

  // Stepper scaling to 95% with balanced down:
  // up1 = round(6*0.95), up2 = round(6*0.95), down = up1 + up2 (balanced)
  int up1 = (int)round(stepperTurnsBase[0] * stepperScale);
  int up2 = (int)round(stepperTurnsBase[1] * stepperScale);
  if (up1 < 1) up1 = 1;
  if (up2 < 1) up2 = 1;
  int down = up1 + up2; // ensure perfect balance
  stepperTurns[0] = up1;
  stepperTurns[1] = up2;
  stepperTurns[2] = down;

  // Vib: 9/10/11/3
  for (int i=0;i<4;i++){
    pinMode(vibPins[i], OUTPUT);
    vib[i].pin = vibPins[i];
    vib[i].env = vibEnv[i];
    vib[i].pwmCapable = isPWMPin(vibPins[i]);
    writeLevel(vib[i], 0);
  }

  // Stepper
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);

  // DC motors
  pinMode(M6_PIN, OUTPUT); analogWrite(M6_PIN, 0);
  pinMode(M7_PIN, OUTPUT); digitalWrite(M7_PIN, LOW);

  // Gearmotor (Pin 8)
  pinMode(GEAR_PIN, OUTPUT); digitalWrite(GEAR_PIN, LOW);
  gearForceFull      = false;
  gearOnWindow       = false;
  gearWindowStart    = millis();
  gearCurrentOnMs    = GEAR_ON_MS_BASE;
  gearCurrentOffMs   = GEAR_OFF_MS_BASE;
  gearPulseActive    = false;
  gearPulsePrevMs    = millis();
  gearPulseStartMs   = 0;

  // Solenoid (Pin 5 PWM)
  pinMode(SOL_PIN, OUTPUT); analogWrite(SOL_PIN, 0);
  solState = SOL_IDLE; solCurrentDuty = 0;

  // Voice m6
  m6.pin = M6_PIN; m6.env = motor6Env; m6.pwmCapable = isPWMPin(M6_PIN); writeLevel(m6, 0);

  // Piezo baseline
  long sum=0; for (int i=0;i<(int)baselineSamples;i++){ sum += analogRead(piezoPin); delay(5); }
  baseline = sum / (int)baselineSamples;

  // Euclid for vib
  for (int v=0; v<4; v++) fillEuclid(vibPattern[v], vibEuclid[v].pulses, vibEuclid[v].steps, vibEuclid[v].rotate);

  state = IDLE; stateStartMs = millis(); lastTickMs = millis();
}

/* ========================= LOOP ========================= */
void loop(){
  uint32_t now = millis();

  // PIEZO TRIGGER
  total -= readings[readIndex];
  int raw = analogRead(piezoPin);
  readings[readIndex] = raw; total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  smoothed = total / numReadings; int delta = smoothed - baseline;

  if (state == IDLE && now >= lockoutUntil){
    if (abs(delta) > (triggerThreshold + hysteresis)){
      lockoutUntil = now + lockoutMs;
      pieceStartMs = now;

      // Stepper (scaled to 95%, with balanced down)
      state = STEPPER_RUNNING; stateStartMs = now; runStepperScaled();

      // Fanout 1s
      state = FANOUT_STATE; stateStartMs = millis(); doFanout();

      // Sequencing
      state = SEQUENCING; stateStartMs = millis();
      barsElapsed = 0;

      for (int i=0;i<4; i++) gateOn(vib[i], stateStartMs);
      gateOn(m6, stateStartMs);

      // M7 windows (begin ON)
      m7IsOn = true; digitalWrite(M7_PIN, HIGH);
      m7WindowEndMs = now + rndMillis(m7OnMinSec, m7OnMaxSec);

      // Gear windows: begin OFF; first ON will be default unless long windows remain
      gearForceFull      = false;
      gearOnWindow       = false;
      gearWindowStart    = millis();
      gearPulseActive    = false;
      gearPulsePrevMs    = millis();
      gearPulseStartMs   = 0;

      // M6 burst scheduler
      m6BurstActive = false; m6NextBurstAt = millis(); m6BurstEndAt = 0;
    }
  }

  // MUSICAL CLOCK
  if (now - lastTickMs >= tickIntervalMs()){
    lastTickMs += tickIntervalMs();
    ticks++;

    if (state == SEQUENCING || state == BUSY_CRESCENDO){
      uint32_t tpq = PPQN, tpBar = tpq * 4;
      uint32_t barTick = ticks % tpBar;
      uint8_t step = (barTick * STEPS_PER_BAR) / tpBar;

      if (state == SEQUENCING){
        // Vib from Euclid
        for (int i=0;i<4;i++){
          if (vibPattern[i][step]) gateOn(vib[i], now);
          else if (vib[i].gate)    gateOff(vib[i], now);
        }
      } else {
        // BUSY: dense
        for (int i=0;i<4;i++) gateOn(vib[i], now);
      }

      if (barTick == tpBar - 1) barsElapsed++;
    }
  }

  // ABSOLUTE ~48s → reprise stepper (while others blast) → short wrap → stop
  if (state == SEQUENCING){
    if (now - pieceStartMs >= repriseAfterMs){
      state = REPRISE_STEPPER; stateStartMs = now;

      // Finale: force gearmotor HIGH (100%)
      gearForceFull = true; digitalWrite(GEAR_PIN, HIGH);

      // Final stepper run (still at scaled/balanced values)
      runStepperScaled();

      // Move into short BUSY wrap (1 bar), then stop
      state = BUSY_CRESCENDO; stateStartMs = millis();
      barsElapsed = 0;
    }
  }
  if (state == BUSY_CRESCENDO){
    static bool inited = false;
    static uint32_t busyStartBars = 0;
    uint32_t tpBar = PPQN * 4;
    uint32_t currentBars = (ticks / tpBar);
    if (!inited){ busyStartBars = currentBars; inited = true; }
    if ((currentBars - busyStartBars) >= busyBars){
      // Hard stop all
      for (int i=0;i<4;i++){ gateOff(vib[i], now); writeLevel(vib[i], 0); }
      gateOff(m6, now); writeLevel(m6, 0);
      digitalWrite(M7_PIN, LOW); m7IsOn = false;

      // Gear off
      gearForceFull = false;
      gearOnWindow = false;
      gearPulseActive = false;
      digitalWrite(GEAR_PIN, LOW);

      // Solenoid to idle
      solState = SOL_IDLE; solWrite(0);

      state = IDLE; stateStartMs = now; inited = false;
    }
  }

  // M7 ON/OFF windows
  if (state == SEQUENCING || state == BUSY_CRESCENDO){
    if (now >= m7WindowEndMs){
      if (m7IsOn){ m7IsOn = false; digitalWrite(M7_PIN, LOW);  m7WindowEndMs = now + rndMillis(m7OffMinSec, m7OffMaxSec); }
      else       { m7IsOn = true;  digitalWrite(M7_PIN, HIGH); m7WindowEndMs = now + rndMillis(m7OnMinSec,  m7OnMaxSec ); }
    }
  } else { digitalWrite(M7_PIN, LOW); m7IsOn = false; }

  // ===== Pin 8 windows + in-window pulsing (short gap) =====
  if (state == SEQUENCING || state == BUSY_CRESCENDO){
    if (!gearForceFull){
      // Window transitions
      if (gearOnWindow){
        // check ON duration
        if (now - gearWindowStart >= gearCurrentOnMs){
          gearOnWindow    = false;
          gearWindowStart = now;
          gearPulseActive = false;
          digitalWrite(GEAR_PIN, LOW);
          gearCurrentOffMs = GEAR_OFF_MS_BASE; // reset to base
        }
      } else {
        // check OFF duration
        if (now - gearWindowStart >= gearCurrentOffMs){
          gearOnWindow    = true;
          gearWindowStart = now;
          // choose ON length: use long window if any remaining
          if (gearLongOnRemaining > 0){
            gearCurrentOnMs = GEAR_LONG_ON_MS;
            gearLongOnRemaining--;
          } else {
            gearCurrentOnMs = GEAR_ON_MS_BASE;
          }
          // align pulse period to this ON start
          gearPulsePrevMs  = now;
          gearPulseStartMs = 0;
          gearPulseActive  = false;
        }
      }

      // In-window pulse train: 400ms ON + 150ms OFF = 550ms period
      if (gearOnWindow){
        if (!gearPulseActive && (now - gearPulsePrevMs >= GEAR_PULSE_PERIOD_MS)){
          gearPulsePrevMs  = now;
          gearPulseActive  = true;
          gearPulseStartMs = now;
          digitalWrite(GEAR_PIN, HIGH);
        }
        if (gearPulseActive && (now - gearPulseStartMs >= GEAR_PULSE_WIDTH_MS)){
          gearPulseActive = false;
          digitalWrite(GEAR_PIN, LOW);
        }
      }
    } else {
      // Finale override: solid HIGH
      digitalWrite(GEAR_PIN, HIGH);
    }
  } else {
    // outside musical phases
    gearOnWindow = false;
    gearPulseActive = false;
    digitalWrite(GEAR_PIN, LOW);
  }

  // M6 bursts (optional)
  if (state == SEQUENCING){
    if (!m6BurstActive && now >= m6NextBurstAt){
      m6BurstActive = true; m6BurstEndAt  = now + m6BurstDurationMs; analogWrite(M6_PIN, m6BurstLevel);
    }
    if (m6BurstActive && now >= m6BurstEndAt){
      m6BurstActive = false; analogWrite(M6_PIN, 0); m6NextBurstAt = now + m6BurstIntervalMs;
    }
  } else { m6BurstActive = false; }

  // Envelopes
  for (int i=0;i<4;i++){
    if ((state==SEQUENCING || state==BUSY_CRESCENDO) || vib[i].active){
      driveVibImpl(vib[i], i, now);
    }
  }
  if ((state==SEQUENCING || state==BUSY_CRESCENDO) || m6.active){
    if (!m6BurstActive){
      uint8_t lvl = computeADSR(m6.env, m6.gate, m6.gateOnMs, m6.gateOffMs, now);
      writeLevel(m6, lvl);
      if (!m6.gate && lvl==0) m6.active = false;
    }
  }

  // Solenoid soft controller tick
  solUpdate(now);

  delay(1);
}
