# Hosyond 4WD Smart Robot Car Kit — MASTER v2.5 (Teacher Edition)

Ένα εκπαιδευτικό project για Arduino 4WD robot car με 3 βασικές λειτουργίες:
- **MANUAL** (IR Remote + Bluetooth + Serial)
- **LINE TRACKING** (3 αισθητήρες γραμμής)
- **OBSTACLE AVOIDANCE** (Ultrasonic + Servo) με **fail-safe** ώστε να μη “φεύγει στα τυφλά”.

---

## 1) Απαιτήσεις
- Arduino UNO / συμβατό
- L298N Motor Driver
- 3x Line tracking sensors (L/M/R)
- Ultrasonic sensor (HC-SR04 ή παρόμοιο)
- Servo (SG90)
- IR receiver + IR remote
- (Προαιρετικά) HC-05 / HC-06 Bluetooth module

---

## 2) Pinout (όπως υλοποιήθηκε στην τρέχουσα έκδοση του κώδικα)

### L298N
- LB = D2
- LF = D4
- RB = D7
- RF = D8
- LPWM = D5
- RPWM = D6

### Line Sensors
- L = D9
- M = D10
- R = D11  
> Οι είσοδοι είναι `INPUT_PULLUP`. Αν οι αισθητήρες σου “διαβάζουν ανάποδα”, άλλαξε στο sketch:
`#define LINE_ACTIVE_LOW 1`

### Ultrasonic
- TRIG = A0
- ECHO = A1  
> Αν TRIG/ECHO είναι ανάποδα → το πρόγραμμα θα βλέπει `999cm` και θα ενεργοποιεί FAIL-SAFE.

### Servo
- SERVO = D3

### IR Receiver
- IR = D12

### Bluetooth (HC-05/HC-06)
- BT RX (Arduino) = A2  (Arduino RX <- BT TX)
- BT TX (Arduino) = A3  (Arduino TX -> BT RX)  
> Το **BT RX** θέλει **διαιρέτη τάσης** (5V -> ~3.3V).

---

## 3) Εντολές

### Serial / Bluetooth (9600 baud)
Modes:
- `S` = STOP
- `M` = MANUAL
- `T` = LINE
- `O` = AVOID

Manual (μόνο σε MANUAL):
- `U` = forward
- `D` = backward
- `L` = left
- `R` = right
- `X` = stop motors

Help:
- `H` = εμφανίζει οδηγίες (Serial)

### IR Remote (NEC cmd mapping)
Movement:
- UP / DOWN / LEFT / RIGHT
- OK = STOP (panic)

Modes:
- `1` = MANUAL
- `2` = LINE
- `3` = AVOID
- `0` = STOP

Speed (MANUAL speed):
- `*` = speed down
- `#` = speed up

TRIM (ευθυγράμμιση ευθείας στη γραμμή):
- `4` = TRIM -
- `6` = TRIM +
- `5` = TRIM reset (0)

> Repeat-safe: αν κρατάς πατημένο βελάκι, τα repeats επηρεάζουν μόνο την κίνηση (όχι mode/speed/trim).

---

## 4) TRIM ευθείας
Αν στο LINE mode το ρομπότ “τραβάει” λίγο δεξιά/αριστερά (μηχανικές ανοχές):

1. Πάτα `2` (LINE)
2. Άφησέ το να κινηθεί 1–2 μέτρα στη γραμμή
3. Αν τραβά **δεξιά** → πάτα `6` (TRIM +)
4. Αν τραβά **αριστερά** → πάτα `4` (TRIM -)
5. Αν μπερδευτείς → πάτα `5` (reset TRIM)

---

## 5) AVOID fail-safe (για ασφάλεια)
Το AVOID mode περιλαμβάνει ασφάλεια:
- Αν ο ultrasonic δίνει πολλές άκυρες μετρήσεις (π.χ. timeout/999cm),
  το ρομπότ **σταματά** και κάνει retry μετά από λίγο.

Αυτό προστατεύει από “runaway” όταν υπάρχει πρόβλημα σε αισθητήρα/καλωδίωση/τροφοδοσία.

---

## 6) Σημαντικές οδηγίες τροφοδοσίας (σταθερή λειτουργία)
- Ιδανικά ο **servo** να τροφοδοτείται από **ξεχωριστά σταθερά 5V** (step-down ή power bank)
- **Κοινή γείωση (GND)** μεταξύ Arduino / L298N / servo / αισθητήρων.
- Αν παρατηρείς resets ή “τρελά” sonar readings, το 90% είναι θέμα τροφοδοσίας/πτωσης τάσης.

---

## 7) Upload tips
- Κατά το upload στο Arduino IDE, **αφαίρεσε προσωρινά το Bluetooth module** (σε αρκετές συνδεσμολογίες επηρεάζει το Serial/Upload).
- Μετά το upload, ξανασύνδεσε το BT.

---

## 8) Γρήγορο troubleshooting
- **LINE δεν ακολουθεί** → άλλαξε `LINE_ACTIVE_LOW` ή έλεγξε τη θέση των αισθητήρων πάνω στη γραμμή.
- **Τραβάει προς μία πλευρά** → χρησιμοποίησε TRIM (4/6).
- 
---

## License
Για εκπαιδευτική χρήση (MIT).
