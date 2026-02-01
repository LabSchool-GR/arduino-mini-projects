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
enum class AvoidState : uint8_t { IDLE = 0, STOP, BACK, LOOK_L, LOOK_R, TURN, CENTER };

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
// 6) TIMERS (χωρίς delay)
// =======================================================

const unsigned long LINE_INTERVAL_MS       = 20;
const unsigned long SONAR_INTERVAL_MS      = 60;
const unsigned long AVOID_STEP_INTERVAL_MS = 40;
const unsigned long MANUAL_TIMEOUT_MS      = 1200;

// Αποφυγή “runaway” αν sonar δεν βλέπει (timeouts)
const uint16_t SONAR_INVALID_CM = 400;     // >=400 θεωρείται άκυρο (π.χ. 999)
const uint8_t  SONAR_INVALID_LIMIT = 8;    // πόσες άκυρες συνεχόμενες
const unsigned long AVOID_RETRY_MS = 600;  // pause και ξαναδοκιμή

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
// 8) ΒΟΗΘΗΤΙΚΑ (logs / help)
// =======================================================

static inline void printHelp() {
#if VERBOSE
  Serial.println(F("\n=== HOSYOND MASTER v2.5 HELP ==="));
  Serial.println(F("Serial/BT: S=STOP, M=MANUAL, T=LINE, O=AVOID"));
  Serial.println(F("Manual (MANUAL): U/D/L/R/X"));
  Serial.println(F("Help: H"));
  Serial.println(F("IR: 1=MANUAL, 2=LINE, 3=AVOID, 0=STOP, OK=STOP"));
  Serial.println(F("IR: *=speed-, #=speed+ | 4=trim-, 6=trim+, 5=trim reset"));
  Serial.println(F("================================\n"));
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
#endif
}

static inline void logManual(const __FlashStringHelper* msg) {
#if VERBOSE
  LOG(F("[MANUAL] "));
  LOGLN(msg);
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
  // left
  if (leftDir > 0)      { digitalWrite(PIN_LF, HIGH); digitalWrite(PIN_LB, LOW); }
  else if (leftDir < 0) { digitalWrite(PIN_LF, LOW);  digitalWrite(PIN_LB, HIGH); }
  else                  { digitalWrite(PIN_LF, LOW);  digitalWrite(PIN_LB, LOW); }

  // right
  if (rightDir > 0)      { digitalWrite(PIN_RF, HIGH); digitalWrite(PIN_RB, LOW); }
  else if (rightDir < 0) { digitalWrite(PIN_RF, LOW);  digitalWrite(PIN_RB, HIGH); }
  else                   { digitalWrite(PIN_RF, LOW);  digitalWrite(PIN_RB, LOW); }

  analogWrite(PIN_LPWM, leftPWM);
  analogWrite(PIN_RPWM, rightPWM);
}

static inline void stopMotors()                  { setMotorRaw(0, 0, 0, 0); }
static inline void forward(uint8_t sp)           { setMotorRaw(+1, +1, sp, sp); }
static inline void backward(uint8_t sp)          { setMotorRaw(-1, -1, sp, sp); }
static inline void turnLeft(uint8_t sp)          { setMotorRaw(-1, +1, sp, sp); }
static inline void turnRight(uint8_t sp)         { setMotorRaw(+1, -1, sp, sp); }
static inline void forwardLR(uint8_t L, uint8_t R){ setMotorRaw(+1, +1, L, R); }

// TRIM ευθείας: L = base+trim, R = base-trim
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

  // reset mode-specific timing/flags
  unsigned long now = millis();
  if (m == Mode::MANUAL) tManualLastCmd = now;
  if (m == Mode::LINE)   tLine = now;
  if (m == Mode::AVOID) {
    avoidState = AvoidState::IDLE;
    tAvoid = now;
    sonarInvalidCount = 0;
    avoidHoldUntil = 0;
    distLeft = distRight = 999;
    logAvoid(avoidState);
  }

  logMode(m);
}

// =======================================================
// 11) INPUT: Serial / Bluetooth
// =======================================================

static inline void applyCommandChar(char c) {
  if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');

  switch (c) {
    // modes
    case 'S': enterMode(Mode::STOP);   break;
    case 'M': enterMode(Mode::MANUAL); break;
    case 'T': enterMode(Mode::LINE);   break;
    case 'O': enterMode(Mode::AVOID);  break;

    // manual moves
    case 'U':
      if (currentMode == Mode::MANUAL) { forward(speedManual); tManualLastCmd = millis(); logManual(F("FORWARD")); }
      break;
    case 'D':
      if (currentMode == Mode::MANUAL) { backward(speedManual); tManualLastCmd = millis(); logManual(F("BACKWARD")); }
      break;
    case 'L':
      if (currentMode == Mode::MANUAL) { turnLeft(SPEED_TURN); tManualLastCmd = millis(); logManual(F("LEFT")); }
      break;
    case 'R':
      if (currentMode == Mode::MANUAL) { turnRight(SPEED_TURN); tManualLastCmd = millis(); logManual(F("RIGHT")); }
      break;
    case 'X':
      if (currentMode == Mode::MANUAL) { stopMotors(); tManualLastCmd = millis(); logManual(F("STOP")); }
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

  // modes (μία φορά)
  if (cmd == IR_1) { enterMode(Mode::MANUAL); return; }
  if (cmd == IR_2) { enterMode(Mode::LINE);   return; }
  if (cmd == IR_3) { enterMode(Mode::AVOID);  return; }
  if (cmd == IR_0) { enterMode(Mode::STOP);   return; }
  if (cmd == IR_OK){ enterMode(Mode::STOP);   return; }

  // speed
  if (cmd == IR_STAR) { speedAdjust(-SPEED_STEP); return; }
  if (cmd == IR_HASH) { speedAdjust(+SPEED_STEP); return; }

  // trim
  if (cmd == IR_4) { trimAdjust(-TRIM_STEP); return; }
  if (cmd == IR_6) { trimAdjust(+TRIM_STEP); return; }
  if (cmd == IR_5) { trimStraight = 0; logTrimSpeed(); return; }

  // motion (repeatable)
  if (cmd == IR_UP)    { goManualIfNeeded(); forward(speedManual);  tManualLastCmd = millis(); logManual(F("FORWARD (IR)")); return; }
  if (cmd == IR_DOWN)  { goManualIfNeeded(); backward(speedManual); tManualLastCmd = millis(); logManual(F("BACKWARD (IR)")); return; }
  if (cmd == IR_LEFT)  { goManualIfNeeded(); turnLeft(SPEED_TURN);  tManualLastCmd = millis(); logManual(F("LEFT (IR)")); return; }
  if (cmd == IR_RIGHT) { goManualIfNeeded(); turnRight(SPEED_TURN); tManualLastCmd = millis(); logManual(F("RIGHT (IR)")); return; }
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
      // repeats μόνο σε κίνηση (όχι σε mode/speed/trim)
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

  // 1) Μόνο ο μεσαίος -> ευθεία (με TRIM)
  if (M && !L && !R) { forwardTrim(SPEED_DEFAULT); return; }

  // 2) Δεξί βλέπει γραμμή -> διόρθωση προς δεξιά
  if (R && !L) {
    uint8_t leftPWM  = (uint8_t)constrain((int)SPEED_DEFAULT + (int)LINE_K, 0, 255);
    uint8_t rightPWM = (uint8_t)constrain((int)SPEED_DEFAULT - (int)LINE_K, 0, 255);
    forwardLR(leftPWM, rightPWM);
    return;
  }

  // 3) Αριστερό βλέπει γραμμή -> διόρθωση προς αριστερά
  if (L && !R) {
    uint8_t leftPWM  = (uint8_t)constrain((int)SPEED_DEFAULT - (int)LINE_K, 0, 255);
    uint8_t rightPWM = (uint8_t)constrain((int)SPEED_DEFAULT + (int)LINE_K, 0, 255);
    forwardLR(leftPWM, rightPWM);
    return;
  }

  // 4) Κανένας δεν βλέπει -> stop motors (μένουμε σε LINE mode)
  if (!L && !M && !R) { stopMotors(); return; }

  // 5) Άλλος συνδυασμός -> αργή ευθεία (με TRIM)
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

  // invalid counter (timeouts/θόρυβος)
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

  // FAIL-SAFE: αν δεν “βλέπει” αξιόπιστα, σταματάμε.
  if (sonarInvalidCount >= SONAR_INVALID_LIMIT) {
    stopMotors();
#if VERBOSE
    LOGLN(F("[AVOID] FAIL-SAFE: SONAR INVALID -> STOP"));
#endif
    if (avoidHoldUntil == 0) avoidHoldUntil = now + AVOID_RETRY_MS;
    if (now < avoidHoldUntil) return;

    // μετά το hold: ξαναδοκιμή
    avoidHoldUntil = 0;
    sonarInvalidCount = 0;
    avoidSet(AvoidState::IDLE, now);
    return;
  }

  // Step timing
  if (now - tAvoid < AVOID_STEP_INTERVAL_MS) return;

  switch (avoidState) {
    case AvoidState::IDLE:
      if (distCm > OBSTACLE_NEAR_CM) forward(SPEED_DEFAULT);
      else avoidSet(AvoidState::STOP, now);
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
        avoidSet(AvoidState::TURN, now);
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
  }
}

// =======================================================
// 15) SETUP / LOOP
// =======================================================

void setup() {
  Serial.begin(9600);
  delay(200);

  // Motors
  pinMode(PIN_LB, OUTPUT);   pinMode(PIN_LF, OUTPUT);
  pinMode(PIN_RB, OUTPUT);   pinMode(PIN_RF, OUTPUT);
  pinMode(PIN_LPWM, OUTPUT); pinMode(PIN_RPWM, OUTPUT);

  // Line sensors (pullups για “ήσυχα” σήματα)
  pinMode(PIN_LINE_L, INPUT_PULLUP);
  pinMode(PIN_LINE_M, INPUT_PULLUP);
  pinMode(PIN_LINE_R, INPUT_PULLUP);

  // Ultrasonic
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // IR
#if USE_IR
  pinMode(PIN_IR_RECV, INPUT);
#endif

  // Servo
#if USE_SERVO
  headServo.attach(PIN_SERVO);
  headServo.write(SERVO_CENTER);
#endif

  // Bluetooth
#if USE_BLUETOOTH
  BT.begin(9600);
#endif

  // init timers
  unsigned long now = millis();
  tLine = tSonar = tAvoid = tManualLastCmd = now;

  enterMode(Mode::STOP);

#if VERBOSE
  LOGLN(F("HOSYOND MASTER v2.5 READY"));
  printHelp();
  logTrimSpeed();
#endif
}

void loop() {
  unsigned long now = millis();

  // Inputs
  updateSerial();
#if USE_BLUETOOTH
  updateBluetooth();
#endif
  updateIR();

  // Manual safety: αν δεν έρθει εντολή, σταματάμε τα μοτέρ (αποφυγή runaway)
  if (currentMode == Mode::MANUAL && (now - tManualLastCmd > MANUAL_TIMEOUT_MS)) {
    stopMotors();
  }

  // Active mode
  if (currentMode == Mode::LINE)  updateLine(now);
  if (currentMode == Mode::AVOID) updateAvoid(now);
}
