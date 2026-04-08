# 02 — Hosyond 4WD Smart Robot Car Kit  
## MASTER v2.7 (Teacher Edition)

📁 Φάκελος: `02_hosyond_4wd_master/`

---

## Α. Προεπισκόπηση

<p align="center">
  <img src="images/preview.jpg" alt="Hosyond 4WD Robot"><br>
  <em>Ολοκλήρωση της κατασκευής στον Σύλλογο Τεχνολογίας Θράκης</em><br>
  <em>Ομάδα Κατασκευής: Κώστας Λ., Γιάννης Γ., Άρης Τ., Δημήτρης Κ.</em>
</p>

---

## Β. Περιγραφή

Ένα ολοκληρωμένο εκπαιδευτικό project για Arduino 4WD robot car, σχεδιασμένο για σχολικά εργαστήρια και μαθήματα τεχνολογίας.

Το project βασίζεται σε **state machine λογική**, υποστηρίζει πολλαπλά modes λειτουργίας και ενσωματώνει μηχανισμούς ασφαλείας (fail-safe), ώστε να λειτουργεί αξιόπιστα σε πραγματικές συνθήκες τάξης.

---

## Γ. Λειτουργίες (Modes)

- **MANUAL**  
  Έλεγχος μέσω IR Remote, Bluetooth ή Serial

- **LINE TRACKING**  
  Παρακολούθηση γραμμής με 3 αισθητήρες

- **OBSTACLE AVOIDANCE**  
  Αποφυγή εμποδίων με ultrasonic + servo

---

## Τι νέο υπάρχει στη v2.7

Η έκδοση v2.7 επεκτείνει το σύστημα αποφυγής εμποδίων:

- Full scan 180° με servo
- Ανίχνευση “διαδρόμων” (valley detection)
- Αξιολόγηση διαδρομών (πλάτος / βάθος / γωνία)
- Anti-stuck μηχανισμός:
  - `consecutiveTurns`
  - `oscillationCount`
- Κατάσταση **STRESS** όταν δεν υπάρχει ασφαλής διαδρομή

Το ρομπότ δεν κινείται πλέον “τυφλά”, αλλά επιλέγει την καλύτερη κατεύθυνση.

Προγραμματιστής / συντάκτης του κώδικα έκδοσης: Άρης Τ.

---

## Δ. Παιδαγωγική φιλοσοφία

- State Machine αρχιτεκτονική
- Non-blocking προγραμματισμός (`millis()`)
- Fail-safe μηχανισμοί
- Modular σχεδίαση (modes)

Στόχος: ασφαλής και προβλέψιμη συμπεριφορά σε εκπαιδευτικό περιβάλλον

---

## Ε. Υλικά

- Arduino UNO (ή συμβατό)
- L298N Motor Driver
- 3 αισθητήρες γραμμής
- Ultrasonic sensor (HC-SR04)
- Servo (SG90)
- IR receiver + remote
- Bluetooth module (HC-05 / HC-06)

---

## ΣΤ. Συνδεσμολογία

### Motor Driver (L298N)
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

> Χρησιμοποιείται `INPUT_PULLUP`

---

### Ultrasonic + Servo
- TRIG = A0  
- ECHO = A1  
- SERVO = D3  

---

### IR Receiver
- IR = D12  

---

### Bluetooth
- RX (Arduino) = A2  
- TX (Arduino) = A3  

⚠ Απαιτείται διαιρέτης τάσης στο RX

---

## Ε. Commands (Serial / Bluetooth)

### Modes
- `S` = STOP  
- `M` = MANUAL  
- `T` = LINE  
- `O` = AVOID  

### Κίνηση (MANUAL)
- `U` = forward  
- `D` = backward  
- `L` = left  
- `R` = right  

---

## 📡 IR Remote

### Modes
- `1` → MANUAL  
- `2` → LINE  
- `3` → AVOID  
- `0` → STOP  

### Κίνηση
- UP / DOWN / LEFT / RIGHT  
- OK → STOP  

### Ρυθμίσεις
- `*` → speed down  
- `#` → speed up  
- `4 / 6` → trim  
- `5` → reset trim  

---

## Ζ. Μηχανισμοί Ασφαλείας

- Fail-safe σε άκυρες μετρήσεις sonar  
- Αυτόματο STOP σε MANUAL (~1.2s χωρίς εντολή)  
- Anti-stuck και επανασάρωση περιβάλλοντος  

---

## Ε. Τροφοδοσία & Upload

- Προτείνεται ξεχωριστή 5V για servo  
- Όλα τα GND κοινά  
- Κατά το upload:
  - αφαιρούμε προσωρινά το Bluetooth  

---

## Ζ. Εκτέλεση

1. Άνοιξε:
Hosyond_4wd_Master_TeacherEdition.ino
2. Επίλεξε Arduino UNO
3. Upload
4. Σύνδεσε Bluetooth / IR

---

## Η. Εκπαιδευτικοί στόχοι

- Ρομποτική πλοήγηση
- State machines
- Sensor fusion
- Embedded ασφάλεια (fail-safe)
- Αυτόνομη λήψη αποφάσεων

---

## Θ. Έκδοση

- **MASTER v2.7 (Teacher Edition)**
- Σταθερή και δοκιμασμένη σε εργαστηριακό περιβάλλον

---

## Ι. License

MIT License — Εκπαιδευτική χρήση