/*
  ==========================================================
  Hosyond 4WD Smart Robot Car Kit
  MASTER v2.5 — Teacher Edition (Compact + Safe)
  ==========================================================

  Author: Δημήτρης Κανατάς
  For: Σύλλογος Τεχνολογίας Θράκης
  License: MIT (προτείνεται για εκπαιδευτική χρήση)
  
  ----------------------------------------------------------
  Εκπαιδευτικό sketch “όλα-σε-ένα” για 4WD car με:
  1) MANUAL (IR + Bluetooth + Serial)
  2) LINE TRACKING (3 sensors)
  3) OBSTACLE AVOIDANCE (Ultrasonic + Servo) με FAIL-SAFE

  - State Machine για τα modes (STOP / MANUAL / LINE / AVOID)
  - Χωρίς delay() στην κίνηση (μόνο delayMicroseconds για sonar trigger)
  - Κοινό σύστημα εντολών: ίδιοι χαρακτήρες για Serial και Bluetooth
  ----------------------------------------------------------
  
  SERIAL / BLUETOOTH COMMANDS (9600 baud)
    S = STOP
    M = MANUAL
    T = LINE
    O = AVOID
    U/D/L/R/X (μόνο στο MANUAL)
    H = Help

  IR (NEC cmd bytes) — βάσει δοκιμών/χειριστηρίου
    Κίνηση:  UP=0x18, DOWN=0x4A, LEFT=0x10, RIGHT=0x5A, OK=0x38
    Modes:   1=0xA2(MANUAL), 2=0x62(LINE), 3=0xE2(AVOID), 0=0x98(STOP)
    Speed:   *=0x68(speed-), #=0xB0(speed+)
    TRIM:    4=0x22(trim-), 5=0x02(reset), 6=0xC2(trim+)

  ΣΗΜΑΝΤΙΚΟ (Hardware)
  - Στο upload βγάλε προσωρινά το Bluetooth module.
  - BT RX θέλει διαιρέτη τάσης (Arduino TX 5V -> BT RX ~3.3V).
  - Servo: ιδανικά ξεχωριστή 5V τροφοδοσία με κοινή GND.
*/

#include <Arduino.h>
#include <Servo.h>
#include <IRremote.h>
#include <SoftwareSerial.h>

// =======================================================
// 1) ΡΥΘΜΙΣΕΙΣ / FLAGS
// =======================================================

#define USE_IR         1
#define USE_BLUETOOTH  1
#define USE_SERVO      1
#define USE_ULTRASONIC 1

// Αν οι αισθητήρες γραμμής δίνουν 0 πάνω στη γραμμή και 1 εκτός (ή το αντίστροφο),
// εδώ το “γυρίζεις” χωρίς να αλλάξεις καλώδια.
#define LINE_ACTIVE_LOW 0

// Εκπαιδευτικό logging στο Serial (βάλε 0 αν θες τελείως “σιωπηλό”).
#define VERBOSE 1

#if VERBOSE
  #define LOG(s)    Serial.print(s)
  #define LOGLN(s)  Serial.println(s)
#else
  #define LOG(s)
  #define LOGLN(s)
#endif

// =======================================================
// 2) PINS (όπως επιβεβαιώθηκαν στα tests)
// =======================================================

// L298N
const uint8_t PIN_LB   = 2;
const uint8_t PIN_LF   = 4;
const uint8_t PIN_RB   = 7;
const uint8_t PIN_RF   = 8;
const uint8_t PIN_LPWM = 5;
const uint8_t PIN_RPWM = 6;

// Line sensors
const uint8_t PIN_LINE_L = 9;
const uint8_t PIN_LINE_M = 10;
const uint8_t PIN_LINE_R = 11;

// Ultrasonic (TRIG/ECHO) + Servo
const uint8_t PIN_TRIG  = A0;
const uint8_t PIN_ECHO  = A1;
const uint8_t PIN_SERVO = 3;

// IR receiver
const uint8_t PIN_IR_RECV = 12;

// Bluetooth (HC-05/HC-06) σε A2/A3 (digital 16/17)
const uint8_t PIN_BT_RX = A2; // Arduino RX <- BT TX
const uint8_t PIN_BT_TX = A3; // Arduino TX -> BT RX (με διαιρέτη τάσης)

// =======================================================
// 3) IR COMMANDS (NEC cmd bytes)
// =======================================================

static const uint8_t IR_UP    = 0x18;
static const uint8_t IR_DOWN  = 0x4A;
static const uint8_t IR_LEFT  = 0x10;
static const uint8_t IR_RIGHT = 0x5A;
static const uint8_t IR_OK    = 0x38;

static const uint8_t IR_0     = 0x98;
static const uint8_t IR_1     = 0xA2;
static const uint8_t IR_2     = 0x62;
static const uint8_t IR_3     = 0xE2;

static const uint8_t IR_4     = 0x22; // trim-
static const uint8_t IR_5     = 0x02; // trim reset
static const uint8_t IR_6     = 0xC2; // trim+

static const uint8_t IR_STAR  = 0x68; // speed-
static const uint8_t IR_HASH  = 0xB0; // speed+

static const uint32_t IR_REPEAT = 0xFFFFFFFF;

// =======================================================
// 4) MODES / STATES
// =======================================================

enum class Mode : uint8_t { STOP = 0, MANUAL, LINE, AVOID };
enum class AvoidState : uint8_t {
  IDLE = 0,        // Default: moving forward, checking obstacles
  STOP, BACK, LOOK_L, LOOK_R, TURN, CENTER,  // Original reactive states
  FULL_SCAN_START, // REV 2.1: initiate 180° scan
  FULL_SCAN_IN_PROGRESS, // scanning arc
  FULL_SCAN_EVALUATE, // analyze valleys
  FULL_SCAN_TURN_TO_VALLEY, // turn toward best valley
  STRESS // trapped / oscillating - needs escape
};

Mode currentMode = Mode::STOP;
AvoidState avoidState = AvoidState::IDLE;

// =======================================================
// 5) TUNING (τα πλήκτρα της συμπεριφοράς)
// =======================================================

// Ταχύτητες
const uint8_t SPEED_DEFAULT = 160;
const uint8_t SPEED_TURN    = 150;
const uint8_t SPEED_SLOW    = 120;

// Line correction strength (πόσο με ένταση διορθώνει αριστερά/δεξιά)
const uint8_t LINE_K = 60;

// Obstacle threshold (cm): κάτω από αυτό θεωρούμε “κοντά εμπόδιο”
const uint16_t OBSTACLE_NEAR_CM = 22;

// Manual speed (ρυθμιζόμενο από IR */#)
uint8_t speedManual = SPEED_DEFAULT;
const uint8_t SPEED_STEP = 15;
const uint8_t SPEED_MIN  = 80;
const uint8_t SPEED_MAX  = 255;

// TRIM ευθείας (live από IR 4/6) — μικρή διόρθωση λόγω μηχανικών ανοχών
int8_t trimStraight = 0;
const int8_t TRIM_STEP = 2;
const int8_t TRIM_MIN  = -40;
const int8_t TRIM_MAX  =  40;

// Servo angles
#if USE_SERVO
Servo headServo;
#endif
const uint8_t SERVO_CENTER = 90;
const uint8_t SERVO_LEFT   = 150;
const uint8_t SERVO_RIGHT  = 30;

// =======================================================
// REV 2.1: Navigation Configuration (Tunable Parameters)
// =======================================================
struct NavConfig {
  // Full scan 180° parameters (configurable)
  uint8_t scanStartAngle     = 0;      // servo angle to start scan (left extreme)
  uint8_t scanEndAngle       = 180;    // servo angle to end scan (right extreme)
  uint8_t scanStepDeg        = 2;      // step size in degrees (smaller = finer scan)
  uint8_t servoSettleMs      = 50;     // delay for servo to reach position

  // Valley detection thresholds
  uint16_t valleyDepthCm     = 40;     // minimum distance to consider (cm)
  uint8_t minValleySamples   = 3;      // minimum consecutive measurements
  uint16_t minValleyWidthCm  = 20;     // minimum valley width (robot width + margin)

  // Valley scoring weights
  uint8_t weightWidth        = 2;      // weight for valley width
  uint8_t weightDepth        = 1;      // weight for valley depth
  uint8_t weightTurn         = 3;      // weight for turn penalty (higher = prefer forward)
  uint16_t minValleyScore    = 75;     // minimum score to consider valley valid (raised above 60)

  // Anti-stuck mechanism triggers
  uint8_t maxAvoidTurns      = 3;      // consecutive turns before full scan

  // Turn timing (ms per degree)
  uint8_t turnMsPerDeg       = 12;

  // Caps and limits
  uint16_t maxDistCm         = 300;    // cap distance readings at this value
  uint16_t sonarMaxCm        = 400;    // consider invalid if >= this
};

NavConfig navConfig;

// =======================================================
// 6) TIMERS (χωρίς delay)
// =======================================================

const unsigned long LINE_INTERVAL_MS       = 20;
const unsigned long SONAR_INTERVAL_MS      = 60;
const unsigned long AVOID_STEP_INTERVAL_MS = 40;
const unsigned long MANUAL_TIMEOUT_MS      = 1200;

// Αποφυγή “runaway” αν sonar δεν βλέπει (timeouts)
const uint16_t SONAR_INVALID_CM = 400;
const uint8_t  SONAR_INVALID_LIMIT = 8;
const unsigned long AVOID_RETRY_MS = 600;

// =======================================================
// 7) GLOBALS (runtime)
// =======================================================

#if USE_BLUETOOTH
SoftwareSerial BT(PIN_BT_RX, PIN_BT_TX);
#endif

// timers
unsigned long tLine  = 0;
unsigned long tSonar = 0;
unsigned long tAvoid = 0;
unsigned long tManualLastCmd = 0;

// sonar cache
uint16_t distCm = 999;
uint8_t sonarInvalidCount = 0;
unsigned long avoidHoldUntil = 0;
uint16_t distLeft = 999, distRight = 999;

// IR repeat-safe
uint8_t lastIrCmd = 0;

// =======================================================
// REV 2.1: Full Scan & Valley Data
// =======================================================

#define MAX_SCAN_SAMPLES 91

struct Valley {
  uint8_t startIdx;
  uint8_t endIdx;
  uint16_t centerAngle;
  uint16_t widthCm;
  uint16_t depthCm;
  uint16_t score;
};

uint16_t scanDistances[MAX_SCAN_SAMPLES];
uint8_t scanSampleCount = 0;

Valley validValleys[5];
uint8_t validValleyCount = 0;
Valley selectedValley;

// Anti-stuck mechanism
uint8_t consecutiveTurns = 0;
bool lastTurnWasLeft = false;
uint8_t oscillationCount = 0;

static inline uint16_t readUltrasonicCm();

// =======================================================
// 8) ΒΟΗΘΗΤΙΚΑ (logs / help)
// =======================================================

// =======================================================
// REV 2.1: SCANNING & VALLEY ANALYSIS FUNCTIONS
// =======================================================

static inline uint8_t calcScanSamples() {
  return (uint8_t)((navConfig.scanEndAngle - navConfig.scanStartAngle) / navConfig.scanStepDeg) + 1;
}

static inline void performFullScan() {
  scanSampleCount = calcScanSamples();
  if (scanSampleCount > MAX_SCAN_SAMPLES) scanSampleCount = MAX_SCAN_SAMPLES;

#if VERBOSE
  Serial.print(F("[SCAN] Starting 180deg scan, "));
  Serial.print(scanSampleCount);
  Serial.println(F(" samples"));
#endif

  for (uint8_t i = 0; i < scanSampleCount; i++) {
    uint16_t angle = navConfig.scanStartAngle + (uint16_t)i * navConfig.scanStepDeg;
    if (angle > 180) angle = 180;

#if USE_SERVO
    headServo.write((uint8_t)angle);
#endif

    delay(navConfig.servoSettleMs);

    uint16_t d = readUltrasonicCm();
    if (d >= navConfig.sonarMaxCm) d = navConfig.maxDistCm;
    else if (d > navConfig.maxDistCm) d = navConfig.maxDistCm;

    scanDistances[i] = d;
  }

#if USE_SERVO
  headServo.write(SERVO_CENTER);
#endif

#if VERBOSE
  Serial.println(F("[SCAN] Completed"));
#endif
}

static inline void detectValleys() {
  validValleyCount = 0;
  if (scanSampleCount < 3) return;

  uint8_t i = 0;
  while (i < scanSampleCount && validValleyCount < 5) {
    if (scanDistances[i] < navConfig.valleyDepthCm) {
      i++;
      continue;
    }

    uint8_t valleyStart = i;
    uint16_t sumDist = 0;
    uint8_t count = 0;

    while (i < scanSampleCount && scanDistances[i] >= navConfig.valleyDepthCm) {
      sumDist += scanDistances[i];
      count++;
      i++;
    }

    uint8_t valleyEnd = i - 1;
    if (count < navConfig.minValleySamples) continue;

    uint16_t centerIdx = (valleyStart + valleyEnd) / 2;
    uint16_t centerAngle = navConfig.scanStartAngle + centerIdx * navConfig.scanStepDeg;
    uint16_t depthCm = sumDist / count;

    uint16_t angleSpan = (valleyEnd - valleyStart) * navConfig.scanStepDeg;
    uint16_t widthCm = depthCm;
    if (angleSpan > 0) {
      widthCm = (depthCm * angleSpan) / 60;
    }

    if (widthCm < navConfig.minValleyWidthCm) continue;

    Valley &v = validValleys[validValleyCount];
    v.startIdx = valleyStart;
    v.endIdx = valleyEnd;
    v.centerAngle = centerAngle;
    v.widthCm = widthCm;
    v.depthCm = depthCm;
    v.score = 0;

    validValleyCount++;
  }

#if VERBOSE
  Serial.print(F("[VALLEYS] Found "));
  Serial.print(validValleyCount);
  Serial.println(F(" valid valleys"));
#endif
}

static inline void scoreValleys() {
  for (uint8_t i = 0; i < validValleyCount; i++) {
    Valley &v = validValleys[i];

    uint16_t widthScore = v.widthCm * navConfig.weightWidth;
    uint16_t depthScore = v.depthCm * navConfig.weightDepth;

    uint16_t forwardDir = 90;
    int16_t angleDiff = (int16_t)v.centerAngle - (int16_t)forwardDir;
    if (angleDiff < 0) angleDiff = -angleDiff;
    uint16_t turnPenalty = (angleDiff * navConfig.weightTurn) / 10;

    v.score = (widthScore + depthScore) > turnPenalty ?
              (widthScore + depthScore) - turnPenalty : 1;
  }

  uint16_t bestScore = 0;
  uint8_t bestIdx = 0;
  for (uint8_t i = 0; i < validValleyCount; i++) {
    if (validValleys[i].score > bestScore) {
      bestScore = validValleys[i].score;
      bestIdx = i;
    }
  }

  if (bestScore >= navConfig.minValleyScore && validValleyCount > 0) {
    selectedValley = validValleys[bestIdx];
#if VERBOSE
    Serial.print(F("[VALLEY] Best: angle="));
    Serial.print(selectedValley.centerAngle);
    Serial.print(F(" score="));
    Serial.println(selectedValley.score);
#endif
  } else {
#if VERBOSE
    LOGLN(F("[VALLEY] No valid valley found"));
#endif
    validValleyCount = 0;
  }
}

static inline void printHelp() {
#if VERBOSE
  Serial.println(F("\n=== HOSYOND MASTER v2.6 (NO FOLLOW) HELP ==="));
  Serial.println(F("Modes: S=STOP, M=MANUAL, T=LINE, O=AVOID"));
  Serial.println(F("Manual moves (MANUAL): U/D/L/R/S (Hosyond App compatible)"));
  Serial.println(F("Help: H"));
  Serial.println(F("IR: 1=MANUAL, 2=LINE, 3=AVOID, 0=STOP, OK=STOP"));
  Serial.println(F("IR: *=speed-, #=speed+ | 4=trim-, 6=trim+, 5=trim reset"));
  Serial.println(F("===========================================\n"));
#endif
}

static inline void logMode(Mode m) {
#if VERBOSE
  LOG(F("[MODE] "));
  if (m == Mode::STOP)   LOGLN(F("STOP"));
  if (m == Mode::MANUAL) LOGLN(F("MANUAL"));
  if (m == Mode::LINE)   LOGLN(F("LINE"));
  if (m == Mode::AVOID)  LOGLN(F("AVOID"));
#endif
}

static inline void logAvoid(AvoidState s) {
#if VERBOSE
  LOG(F("[AVOID] "));
  if (s == AvoidState::IDLE)   LOGLN(F("IDLE"));
  if (s == AvoidState::STOP)   LOGLN(F("STOP"));
  if (s == AvoidState::BACK)   LOGLN(F("BACK"));
  if (s == AvoidState::LOOK_L) LOGLN(F("LOOK LEFT"));
  if (s == AvoidState::LOOK_R) LOGLN(F("LOOK RIGHT"));
  if (s == AvoidState::TURN)   LOGLN(F("TURN"));
  if (s == AvoidState::CENTER) LOGLN(F("CENTER"));
  if (s == AvoidState::FULL_SCAN_START) LOGLN(F("FULL_SCAN_START"));
  if (s == AvoidState::FULL_SCAN_IN_PROGRESS) LOGLN(F("FULL_SCAN_IN_PROGRESS"));
  if (s == AvoidState::FULL_SCAN_EVALUATE) LOGLN(F("FULL_SCAN_EVALUATE"));
  if (s == AvoidState::FULL_SCAN_TURN_TO_VALLEY) LOGLN(F("FULL_SCAN_TURN_TO_VALLEY"));
  if (s == AvoidState::STRESS) LOGLN(F("STRESS"));
#endif
}

static inline void logTrimSpeed() {
#if VERBOSE
  Serial.print(F("[SPEED] manual=")); Serial.println(speedManual);
  Serial.print(F("[TRIM]  straight=")); Serial.println(trimStraight);
#endif
}

// =======================================================
// 9) MOTOR CONTROL (low-level)
// =======================================================

static inline void setMotorRaw(int8_t leftDir, int8_t rightDir, uint8_t leftPWM, uint8_t rightPWM) {
  if (leftDir > 0)      { digitalWrite(PIN_LF, HIGH); digitalWrite(PIN_LB, LOW); }
  else if (leftDir < 0) { digitalWrite(PIN_LF, LOW);  digitalWrite(PIN_LB, HIGH); }
  else                  { digitalWrite(PIN_LF, LOW);  digitalWrite(PIN_LB, LOW); }

  if (rightDir > 0)      { digitalWrite(PIN_RF, HIGH); digitalWrite(PIN_RB, LOW); }
  else if (rightDir < 0) { digitalWrite(PIN_RF, LOW);  digitalWrite(PIN_RB, HIGH); }
  else                   { digitalWrite(PIN_RF, LOW);  digitalWrite(PIN_RB, LOW); }

  analogWrite(PIN_LPWM, leftPWM);
  analogWrite(PIN_RPWM, rightPWM);
}

static inline void stopMotors()                   { setMotorRaw(0, 0, 0, 0); }
static inline void forward(uint8_t sp)            { setMotorRaw(+1, +1, sp, sp); }
static inline void backward(uint8_t sp)           { setMotorRaw(-1, -1, sp, sp); }
static inline void turnLeft(uint8_t sp)           { setMotorRaw(-1, +1, sp, sp); }
static inline void turnRight(uint8_t sp)          { setMotorRaw(+1, -1, sp, sp); }
static inline void forwardLR(uint8_t L, uint8_t R){ setMotorRaw(+1, +1, L, R); }

static inline void forwardTrim(uint8_t base) {
  int L = constrain((int)base + (int)trimStraight, 0, 255);
  int R = constrain((int)base - (int)trimStraight, 0, 255);
  forwardLR((uint8_t)L, (uint8_t)R);
}

// =======================================================
// 10) MODE MANAGEMENT
// =======================================================

static inline void enterMode(Mode m) {
  currentMode = m;
  stopMotors();

#if USE_SERVO
  headServo.write(SERVO_CENTER);
#endif

  unsigned long now = millis();
  if (m == Mode::MANUAL) tManualLastCmd = now;
  if (m == Mode::LINE)   tLine = now;
  if (m == Mode::AVOID) {
    Serial.println("---->Mode 3 Enabled");
    avoidState = AvoidState::FULL_SCAN_START;
    tAvoid = now;
    sonarInvalidCount = 0;
    avoidHoldUntil = 0;
    distLeft = distRight = 999;
    consecutiveTurns = 0;
    oscillationCount = 0;
    lastTurnWasLeft = false;
    selectedValley.startIdx = 0;
    selectedValley.endIdx = 0;
    selectedValley.centerAngle = 90;
    selectedValley.widthCm = 0;
    selectedValley.depthCm = 0;
    selectedValley.score = 0;
    scanSampleCount = 0;
    logAvoid(avoidState);
  }

  logMode(m);
}

// =======================================================
// 11) INPUT: Serial / Bluetooth
// =======================================================

static inline void applyCommandChar(char c) {
  if (c == '\r' || c == '\n' || c == ' ') return;

#if VERBOSE
  Serial.print(F("[CMD] "));
  Serial.println(c);
#endif

  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

  switch (c) {
    case 'S': enterMode(Mode::STOP); break;
    case 'M': enterMode(Mode::MANUAL); break;
    case 'T': enterMode(Mode::LINE); break;
    case 'O': enterMode(Mode::AVOID); break;
    case 'I': enterMode(Mode::MANUAL); break;
    case 'G':
      enterMode(Mode::STOP);
#if VERBOSE
      LOGLN(F("[INFO] Gravity mode not supported -> STOP"));
#endif
      break;

    case 'U':
      if (currentMode != Mode::MANUAL) enterMode(Mode::MANUAL);
      forward(speedManual); tManualLastCmd = millis();
      break;
    case 'D':
      if (currentMode != Mode::MANUAL) enterMode(Mode::MANUAL);
      backward(speedManual); tManualLastCmd = millis();
      break;
    case 'L':
      if (currentMode != Mode::MANUAL) enterMode(Mode::MANUAL);
      turnLeft(SPEED_TURN); tManualLastCmd = millis();
      break;
    case 'R':
      if (currentMode != Mode::MANUAL) enterMode(Mode::MANUAL);
      turnRight(SPEED_TURN); tManualLastCmd = millis();
      break;
    case 'X':
      if (currentMode == Mode::MANUAL) { stopMotors(); tManualLastCmd = millis(); }
      break;

    case 'H':
      printHelp();
      break;

    default:
      break;
  }
}

static inline void updateSerial() {
  while (Serial.available() > 0) applyCommandChar((char)Serial.read());
}

#if USE_BLUETOOTH
static inline void updateBluetooth() {
  while (BT.available() > 0) applyCommandChar((char)BT.read());
}
#endif

// =======================================================
// 12) INPUT: IR (repeat-safe)
// =======================================================

static inline bool isRepeatableMotion(uint8_t cmd) {
  return (cmd == IR_UP || cmd == IR_DOWN || cmd == IR_LEFT || cmd == IR_RIGHT);
}

static inline void speedAdjust(int8_t delta) {
  int s = (int)speedManual + (int)delta;
  speedManual = (uint8_t)constrain(s, (int)SPEED_MIN, (int)SPEED_MAX);
  logTrimSpeed();
}

static inline void trimAdjust(int8_t delta) {
  int t = (int)trimStraight + (int)delta;
  trimStraight = (int8_t)constrain(t, (int)TRIM_MIN, (int)TRIM_MAX);
  logTrimSpeed();
}

static inline void goManualIfNeeded() {
  if (currentMode != Mode::MANUAL) enterMode(Mode::MANUAL);
}

static inline void applyIrCmd(uint8_t cmd) {
  lastIrCmd = cmd;

  if (cmd == IR_1) { enterMode(Mode::MANUAL); return; }
  if (cmd == IR_2) { enterMode(Mode::LINE);   return; }
  if (cmd == IR_3) { enterMode(Mode::AVOID);  return; }
  if (cmd == IR_0) { enterMode(Mode::STOP);   return; }
  if (cmd == IR_OK){ enterMode(Mode::STOP);   return; }

  if (cmd == IR_STAR) { speedAdjust(-SPEED_STEP); return; }
  if (cmd == IR_HASH) { speedAdjust(+SPEED_STEP); return; }

  if (cmd == IR_4) { trimAdjust(-TRIM_STEP); return; }
  if (cmd == IR_6) { trimAdjust(+TRIM_STEP); return; }
  if (cmd == IR_5) { trimStraight = 0; logTrimSpeed(); return; }

  if (cmd == IR_UP)    { goManualIfNeeded(); forward(speedManual);  tManualLastCmd = millis(); return; }
  if (cmd == IR_DOWN)  { goManualIfNeeded(); backward(speedManual); tManualLastCmd = millis(); return; }
  if (cmd == IR_LEFT)  { goManualIfNeeded(); turnLeft(SPEED_TURN);  tManualLastCmd = millis(); return; }
  if (cmd == IR_RIGHT) { goManualIfNeeded(); turnRight(SPEED_TURN); tManualLastCmd = millis(); return; }
}

static inline void updateIR() {
#if USE_IR
  static IRrecv irrecv(PIN_IR_RECV);
  static decode_results results;
  static bool started = false;

  if (!started) { irrecv.enableIRIn(); started = true; }

  if (irrecv.decode(&results)) {
    uint32_t code = (uint32_t)results.value;
    irrecv.resume();

    const bool repeat = (code == IR_REPEAT);
    const uint8_t cmd = (uint8_t)((code >> 8) & 0xFF);

    if (repeat) {
      if (lastIrCmd && isRepeatableMotion(lastIrCmd)) {
        goManualIfNeeded();
        tManualLastCmd = millis();
        if      (lastIrCmd == IR_UP)    forward(speedManual);
        else if (lastIrCmd == IR_DOWN)  backward(speedManual);
        else if (lastIrCmd == IR_LEFT)  turnLeft(SPEED_TURN);
        else if (lastIrCmd == IR_RIGHT) turnRight(SPEED_TURN);
      }
    } else {
      applyIrCmd(cmd);
    }
  }
#endif
}

// =======================================================
// 13) LINE TRACKING
// =======================================================

static inline bool readLinePin(uint8_t pin) {
  bool v = digitalRead(pin);
  if (LINE_ACTIVE_LOW) v = !v;
  return v;
}

static inline void updateLine(unsigned long now) {
  if (now - tLine < LINE_INTERVAL_MS) return;
  tLine = now;

  const bool L = readLinePin(PIN_LINE_L);
  const bool M = readLinePin(PIN_LINE_M);
  const bool R = readLinePin(PIN_LINE_R);

  if (M && !L && !R) { forwardTrim(SPEED_DEFAULT); return; }

  if (R && !L) {
    uint8_t leftPWM  = (uint8_t)constrain((int)SPEED_DEFAULT + (int)LINE_K, 0, 255);
    uint8_t rightPWM = (uint8_t)constrain((int)SPEED_DEFAULT - (int)LINE_K, 0, 255);
    forwardLR(leftPWM, rightPWM);
    return;
  }

  if (L && !R) {
    uint8_t leftPWM  = (uint8_t)constrain((int)SPEED_DEFAULT - (int)LINE_K, 0, 255);
    uint8_t rightPWM = (uint8_t)constrain((int)SPEED_DEFAULT + (int)LINE_K, 0, 255);
    forwardLR(leftPWM, rightPWM);
    return;
  }

  if (!L && !M && !R) { stopMotors(); return; }

  forwardTrim(SPEED_SLOW);
}

// =======================================================
// 14) ULTRASONIC + SAFE AVOID
// =======================================================

static inline uint16_t readUltrasonicCm() {
#if USE_ULTRASONIC
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long d = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (d == 0) return 999;
  return (uint16_t)(d / 58UL);
#else
  return 999;
#endif
}

static inline void updateSonar(unsigned long now) {
  if (now - tSonar < SONAR_INTERVAL_MS) return;
  tSonar = now;

  distCm = readUltrasonicCm();

  if (distCm >= SONAR_INVALID_CM) {
    if (sonarInvalidCount < 255) sonarInvalidCount++;
  } else {
    sonarInvalidCount = 0;
  }
}

static inline void avoidSet(AvoidState st, unsigned long now) {
  avoidState = st;
  tAvoid = now;
  logAvoid(st);
}

static inline void updateAvoid(unsigned long now) {
  updateSonar(now);

  if (sonarInvalidCount >= SONAR_INVALID_LIMIT) {
    stopMotors();
#if VERBOSE
    LOGLN(F("[AVOID] FAIL-SAFE: SONAR INVALID -> FULL_SCAN"));
#endif
    avoidSet(AvoidState::FULL_SCAN_START, now);
    if (avoidHoldUntil == 0) avoidHoldUntil = now + AVOID_RETRY_MS;
    if (now < avoidHoldUntil) return;
    avoidHoldUntil = 0;
    sonarInvalidCount = 0;
    return;
  }

  if (now - tAvoid < AVOID_STEP_INTERVAL_MS) return;

  switch (avoidState) {
    case AvoidState::IDLE:
      if (distCm > OBSTACLE_NEAR_CM) {
        forward(SPEED_DEFAULT);
        consecutiveTurns = 0;
      } else {
        avoidSet(AvoidState::STOP, now);
      }
      break;

    case AvoidState::STOP:
      stopMotors();
      if (now - tAvoid >= 120) avoidSet(AvoidState::BACK, now);
      break;

    case AvoidState::BACK:
      backward(SPEED_SLOW);
      if (now - tAvoid >= 220) {
        stopMotors();
#if USE_SERVO
        headServo.write(SERVO_LEFT);
#endif
        avoidSet(AvoidState::LOOK_L, now);
      }
      break;

    case AvoidState::LOOK_L:
      if (now - tAvoid >= 180) {
        distLeft = readUltrasonicCm();
#if USE_SERVO
        headServo.write(SERVO_RIGHT);
#endif
        avoidSet(AvoidState::LOOK_R, now);
      }
      break;

    case AvoidState::LOOK_R:
      if (now - tAvoid >= 180) {
        distRight = readUltrasonicCm();

        bool turnL = (distLeft >= distRight);
        if (turnL != lastTurnWasLeft) {
          oscillationCount++;
        } else {
          oscillationCount = 0;
        }
        lastTurnWasLeft = turnL;

        consecutiveTurns++;

        if (consecutiveTurns >= navConfig.maxAvoidTurns || oscillationCount >= 2) {
          avoidSet(AvoidState::FULL_SCAN_START, now);
        } else {
          avoidSet(AvoidState::TURN, now);
        }
      }
      break;

    case AvoidState::TURN:
      if (distLeft >= distRight) turnLeft(SPEED_TURN);
      else turnRight(SPEED_TURN);

      if (now - tAvoid >= 260) {
        stopMotors();
#if USE_SERVO
        headServo.write(SERVO_CENTER);
#endif
        avoidSet(AvoidState::CENTER, now);
      }
      break;

    case AvoidState::CENTER:
      if (now - tAvoid >= 160) {
        distLeft = distRight = 999;
        avoidSet(AvoidState::IDLE, now);
      }
      break;

    case AvoidState::FULL_SCAN_START:
      stopMotors();
      if (now - tAvoid >= 150) {
        avoidSet(AvoidState::FULL_SCAN_IN_PROGRESS, now);
      }
      break;

    case AvoidState::FULL_SCAN_IN_PROGRESS:
      if (scanSampleCount == 0) {
        performFullScan();
        avoidSet(AvoidState::FULL_SCAN_EVALUATE, now);
      }
      break;

    case AvoidState::FULL_SCAN_EVALUATE:
      detectValleys();
      if (validValleyCount > 0) {
        scoreValleys();
        avoidSet(AvoidState::FULL_SCAN_TURN_TO_VALLEY, now);
      } else {
        avoidSet(AvoidState::STRESS, now);
      }
      break;

    case AvoidState::FULL_SCAN_TURN_TO_VALLEY:
      {
        int16_t angleDiff = (int16_t)selectedValley.centerAngle - 90;
        if (angleDiff < -1) {
          turnLeft(SPEED_TURN);
        } else if (angleDiff > 1) {
          turnRight(SPEED_TURN);
        } else {
          stopMotors();
        }

        uint16_t turnDuration = (uint16_t)(abs(angleDiff) * navConfig.turnMsPerDeg);
        if (turnDuration == 0) turnDuration = navConfig.turnMsPerDeg;
        if (now - tAvoid >= turnDuration) {
          stopMotors();
          scanSampleCount = 0;
          consecutiveTurns = 0;
          oscillationCount = 0;
          avoidSet(AvoidState::IDLE, now);
        }
      }
      break;

    case AvoidState::STRESS:
      backward(SPEED_SLOW);
      if (now - tAvoid >= 600) {
        stopMotors();
        scanSampleCount = 0;
        avoidSet(AvoidState::FULL_SCAN_START, now);
      }
      break;
  }
}

// =======================================================
// 15) SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(9600);
  delay(200);

  pinMode(PIN_LB, OUTPUT);   pinMode(PIN_LF, OUTPUT);
  pinMode(PIN_RB, OUTPUT);   pinMode(PIN_RF, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT); pinMode(PIN_RPWM, OUTPUT);

  pinMode(PIN_LINE_L, INPUT_PULLUP);
  pinMode(PIN_LINE_M, INPUT_PULLUP);
  pinMode(PIN_LINE_R, INPUT_PULLUP);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

#if USE_IR
  pinMode(PIN_IR_RECV, INPUT);
#endif

#if USE_SERVO
  headServo.attach(PIN_SERVO);
  headServo.write(SERVO_CENTER);
#endif

#if USE_BLUETOOTH
  BT.begin(9600);
#endif

  unsigned long now = millis();
  tLine = tSonar = tAvoid = tManualLastCmd = now;

  enterMode(Mode::STOP);

#if VERBOSE
  LOGLN(F("HOSYOND MASTER v2.6 READY (NO FOLLOW)"));
  LOGLN(F("Enhanced Navigation with Full Scan + Valley Detect"));
  printHelp();
  logTrimSpeed();
#endif
}

void loop() {
  unsigned long now = millis();

  updateSerial();
#if USE_BLUETOOTH
  updateBluetooth();
#endif
  updateIR();

  if (currentMode == Mode::MANUAL && (now - tManualLastCmd > MANUAL_TIMEOUT_MS)) {
    stopMotors();
  }

  if (currentMode == Mode::LINE)  updateLine(now);
  if (currentMode == Mode::AVOID) updateAvoid(now);
}
