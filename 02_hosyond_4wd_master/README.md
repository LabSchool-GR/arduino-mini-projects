# 02 - Hosyond 4WD Smart Robot Car Kit — MASTER v2.6

Εκπαιδευτικό “όλα-σε-ένα” sketch για 4WD Arduino car με ασφαλή state-machine λογική και 3 modes:

- **MANUAL** (IR + Bluetooth + Serial)
- **LINE TRACKING** (3 sensors)
- **OBSTACLE AVOIDANCE** (Ultrasonic + Servo) με **fail-safe** και **Full Scan + Valley Detection**

---

## 1) Τι νέο υπάρχει στη v2.6

Η v2.6 κρατά τη σταθερή συμπεριφορά της Teacher Edition και προσθέτει ενισχυμένη πλοήγηση στην αποφυγή εμποδίων:

- Νέες καταστάσεις στο `AvoidState`:
  - `FULL_SCAN_START`
  - `FULL_SCAN_IN_PROGRESS`
  - `FULL_SCAN_EVALUATE`
  - `FULL_SCAN_TURN_TO_VALLEY`
  - `STRESS`
- Σάρωση 180° με servo (`performFullScan()`)
- Ανίχνευση “κοιλάδων”/διαδρόμων (`detectValleys()`)
- Βαθμολόγηση κοιλάδων (`scoreValleys()`) με βάρη:
  - πλάτος
  - βάθος
  - ποινή μεγάλης στροφής
- Anti-stuck λογική:
  - `consecutiveTurns`
  - `oscillationCount`
- Όταν δεν υπάρχει ασφαλές άνοιγμα, το ρομπότ μπαίνει σε `STRESS` και επιχειρεί escape + επανασάρωση.

---

## 2) Commands (Serial / Bluetooth, 9600)

### Modes
- `S` = STOP
- `M` = MANUAL
- `T` = LINE
- `O` = AVOID

### Movement (MANUAL)
- `U` = forward
- `D` = backward
- `L` = left
- `R` = right
- `X` = stop (μέσα στο MANUAL)

### Extra
- `H` = Help
- `I` = MANUAL (Hosyond app IR Control button)
- `G` = STOP (gravity mode not supported)

---

## 3) IR mapping (NEC command bytes)

### Κίνηση
- `UP=0x18`
- `DOWN=0x4A`
- `LEFT=0x10`
- `RIGHT=0x5A`
- `OK=0x38` (STOP)

### Modes
- `1=0xA2` → MANUAL
- `2=0x62` → LINE
- `3=0xE2` → AVOID
- `0=0x98` → STOP

### Ρυθμίσεις
- `*=0x68` → speed-
- `#=0xB0` → speed+
- `4=0x22` → trim-
- `5=0x02` → trim reset
- `6=0xC2` → trim+

---

## 4) Pinout

### L298N
- LB = D2
- LF = D4
- RB = D7
- RF = D8
- LPWM = D5
- RPWM = D6

### Line sensors
- L = D9
- M = D10
- R = D11

> Χρησιμοποιείται `INPUT_PULLUP`.
> Αν η λογική είναι ανάποδη, άλλαξε `#define LINE_ACTIVE_LOW 0` σε `1`.

### Ultrasonic + Servo
- TRIG = A0
- ECHO = A1
- SERVO = D3

### IR + Bluetooth
- IR receiver = D12
- BT RX (Arduino) = A2  (Arduino RX <- BT TX)
- BT TX (Arduino) = A3  (Arduino TX -> BT RX, με διαιρέτη τάσης)

---

## 5) Ασφάλεια / Fail-safe

- Αν το sonar δώσει επαναλαμβανόμενες άκυρες μετρήσεις, ενεργοποιείται fail-safe.
- Το mode AVOID μπαίνει σε full scan αντί να συνεχίσει “στα τυφλά”.
- Στο MANUAL, αν δεν έρθει εντολή για ~1.2s, τα μοτέρ σταματούν αυτόματα.

---

## 6) Τροφοδοσία & Upload tips

- Στο upload βγάζουμε προσωρινά το Bluetooth module.
- BT RX χρειάζεται διαιρέτη τάσης (5V -> ~3.3V).
- Servo: προτείνεται ξεχωριστή σταθερή 5V τροφοδοσία με **κοινή GND** με Arduino/L298N.

---

## 7) Έκδοση

- Sketch: **MASTER v2.6 (Teacher Edition, NO FOLLOW)**
- Έμφαση σε εκπαιδευτική καθαρότητα, ασφάλεια και επεκτασιμότητα.

## License

MIT
