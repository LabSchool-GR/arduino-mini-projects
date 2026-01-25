/*
  ==========================================================
  OLED DHT11 Mini Dashboard (SSD1306 128x64)
  ==========================================================

  Author: Δημήτρης Κανάτας
  For: Σύλλογος Τεχνολογίας Θράκης
  License: MIT (προτείνεται για εκπαιδευτική χρήση)
*/

#include <Wire.h>                 // Επικοινωνία I2C (η OLED μιλάει με I2C)
#include <Adafruit_GFX.h>         // Βιβλιοθήκη γραφικών (κείμενα, γραμμές, κύκλοι)
#include <Adafruit_SSD1306.h>     // Βιβλιοθήκη για OLED SSD1306
#include <DHT.h>                  // Βιβλιοθήκη για αισθητήρα DHT11/DHT22

// =======================================================
// 1) ΡΥΘΜΙΣΕΙΣ ΟΘΟΝΗΣ OLED 128x64
// =======================================================

// Πλάτος και ύψος οθόνης σε pixels
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Reset pin: συνήθως -1 όταν δεν χρησιμοποιούμε ξεχωριστό reset pin
#define OLED_RESET -1

// Δημιουργούμε το αντικείμενο της οθόνης.
// Από εδώ και πέρα, ό,τι σχεδιάζουμε το κάνουμε με display....
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =======================================================
// 2) ΡΥΘΜΙΣΕΙΣ ΑΙΣΘΗΤΗΡΑ DHT11
// =======================================================

// Το pin δεδομένων (DATA) του DHT
#define DHTPIN 2

// Δηλώνουμε ότι έχουμε DHT11
#define DHTTYPE DHT11

// Δημιουργούμε το αντικείμενο του αισθητήρα
DHT dht(DHTPIN, DHTTYPE);

// =======================================================
// 3) ΔΙΑΧΩΡΙΣΜΟΣ ΟΘΟΝΗΣ ΣΕ "ΖΩΝΕΣ"
// =======================================================

// Η οθόνη μας είναι 128x64.
// Θέλουμε να την χωρίσουμε σε:

// (Α) Πάνω μπάρα 16 pixels για scrolling text
const int TOP_H = 16;

// Η διαχωριστική γραμμή θα είναι στο y=16
const int SEP_Y = TOP_H;

// (Β) Κάτω περιοχή από y=17 μέχρι y=63
const int BODY_Y = TOP_H + 1;

// Το ύψος της κάτω περιοχής
const int BODY_H = SCREEN_HEIGHT - BODY_Y;

// Η κάτω περιοχή χωρίζεται σε 2 μισά των 64 pixels:
// Αριστερά: θερμοκρασία
// Δεξιά: υγρασία
const int HALF_W = 64;

// =======================================================
// 4) TIMERS (ΧΩΡΙΣ delay())
// =======================================================

// Κάθε πόσα ms (χιλιοστά του δευτερολέπτου) διαβάζουμε τον αισθητήρα
// 2000ms = 2 δευτερόλεπτα
const unsigned long SENSOR_INTERVAL_MS = 2000;

// Κάθε πόσα ms ανανεώνουμε την οθόνη (≈ 30fps)
const unsigned long DISPLAY_INTERVAL_MS = 33;

// Αυτές οι μεταβλητές κρατούν "πότε έγινε τελευταία φορά" κάτι
unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;

// =======================================================
// 5) ΔΕΔΟΜΕΝΑ ΑΙΣΘΗΤΗΡΑ
// =======================================================

// t = θερμοκρασία
// h = υγρασία
// NAN σημαίνει "δεν υπάρχει τιμή" ή "έγινε λάθος"
float t = NAN;
float h = NAN;

// =======================================================
// 6) SCROLLING TEXT (ομαλή κίνηση)
// =======================================================

// Το κείμενο που κυλάει στην πάνω μπάρα
String scrollingText = "STETH.GR | TECHTHRACE.COM";

// Η θέση Χ του κειμένου (float για ομαλή κίνηση)
float scrollX = SCREEN_WIDTH;

// Ταχύτητα κύλισης σε pixels ανά δευτερόλεπτο
// Αν αυξήσεις την τιμή, θα τρέχει πιο γρήγορα.
const float SCROLL_SPEED_PX_PER_SEC = 45.0f;

// Θα κρατήσουμε το πλάτος του κειμένου σε pixels,
// ώστε όταν φύγει αριστερά να το επαναφέρουμε σωστά.
int scrollTextWidth = 0;

// Χρόνος τελευταίας ενημέρωσης animation (για dt)
unsigned long lastAnimTime = 0;

// =======================================================
// 7) ΒΟΗΘΗΤΙΚΕΣ ΣΥΝΑΡΤΗΣΕΙΣ (για ωραία εμφάνιση)
// =======================================================

/*
  drawCenteredTextInBox(...)
  -------------------------
  Ζωγραφίζει ένα κείμενο ΚΕΝΤΡΑΡΙΣΜΕΝΟ μέσα σε ένα "κουτί".
*/
void drawCenteredTextInBox(int x, int y, int w, int hgt, const String &text, uint8_t size) {
  int16_t x1, y1;
  uint16_t tw, th;

  // Ορίζουμε μέγεθος κειμένου
  display.setTextSize(size);

  // Μετράμε πόσο χώρο πιάνει το κείμενο (πλάτος tw, ύψος th)
  display.getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);

  // Υπολογίζουμε το σημείο εκκίνησης για να είναι στο κέντρο
  int cx = x + (w - (int)tw) / 2;
  int cy = y + (hgt - (int)th) / 2;

  // Βάζουμε cursor και τυπώνουμε
  display.setCursor(cx, cy);
  display.print(text);
}

/*
  drawCenteredTempValue(...)
  -------------------------
  Να εμφανίσουμε θερμοκρασία με δεκαδικό (π.χ. 17.3)
  αλλά να χωράει ωραία στο πλαίσιο.
*/
void drawCenteredTempValue(int boxX, int boxY, int boxW, int boxH, float temp) {
  String tempNum = String(temp, 1);  // 1 δεκαδικό
  String unitC = "C";               // κρατάμε απλό "C" (το ° δεν εμφανίζεται πάντα)

  int16_t x1, y1;
  uint16_t wNum, hNum, wC, hC;

  // Μετράμε το μεγάλο νούμερο
  display.setTextSize(2);
  display.getTextBounds(tempNum, 0, 0, &x1, &y1, &wNum, &hNum);

  // Μετράμε το μικρό C
  display.setTextSize(1);
  display.getTextBounds(unitC, 0, 0, &x1, &y1, &wC, &hC);

  // Κενό ανάμεσα
  int gap = 2;

  // Συνολικό πλάτος = μεγάλο νούμερο + κενό + μικρό C
  int totalW = (int)wNum + gap + (int)wC;

  // Υπολογίζουμε από πού ξεκινά για να είναι στο κέντρο
  int startX = boxX + (boxW - totalW) / 2;

  // Κέντρο του κουτιού κατακόρυφα
  int centerY = boxY + (boxH / 2);

  // Θέση Y του μεγάλου αριθμού ώστε να κεντραριστεί
  int numY = centerY - (hNum / 2);

  // Τυπώνουμε το μεγάλο νούμερο
  display.setTextSize(2);
  display.setCursor(startX, numY);
  display.print(tempNum);

  // Τυπώνουμε το μικρό C δίπλα, λίγο πιο χαμηλά
  display.setTextSize(1);
  display.setCursor(startX + wNum + gap, numY + 6);
  display.print(unitC);
}

// =======================================================
// 8) ΕΙΚΟΝΙΔΙΑ (με drawLine / drawCircle)
// =======================================================

/*
  drawThermometerIcon(x, y)
  -------------------------
  Σχεδιάζει ένα μικρό θερμόμετρο.
*/
void drawThermometerIcon(int x, int y) {
  // "Μπάλα" κάτω (bulb)
  display.drawCircle(x + 3, y + 10, 3, WHITE);

  // "Σωλήνας" προς τα πάνω
  display.drawLine(x + 3, y + 2, x + 3, y + 8, WHITE);

  // Περίγραμμα σωλήνα (2 γραμμές)
  display.drawLine(x + 2, y + 2, x + 2, y + 8, WHITE);
  display.drawLine(x + 4, y + 2, x + 4, y + 8, WHITE);

  // Μικρό "καπάκι" πάνω
  display.drawLine(x + 2, y + 2, x + 4, y + 2, WHITE);
}

/*
  drawDropIcon(x, y)
  ------------------
  Σχεδιάζει μια μικρή σταγόνα (για υγρασία).
*/
void drawDropIcon(int x, int y) {
  // Κάτω στρογγυλό μέρος
  display.drawCircle(x + 4, y + 9, 3, WHITE);

  // Πάνω μύτη σταγόνας
  display.drawLine(x + 4, y + 1, x + 2, y + 6, WHITE);
  display.drawLine(x + 4, y + 1, x + 6, y + 6, WHITE);
}

// =======================================================
// 9) UPDATE ΛΟΓΙΚΗΣ (χωρίς γραφικά)
// =======================================================

/*
  updateSensor(now)
  -----------------
  Διαβάζει τον αισθητήρα μόνο όταν περάσουν 2 δευτερόλεπτα.
  Αυτό είναι σημαντικό γιατί:
  - Ο DHT11 δεν πρέπει να διαβάζεται συνέχεια
  - Θέλουμε να παραμένει ομαλό το scrolling
*/
void updateSensor(unsigned long now) {
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    t = dht.readTemperature();  // θερμοκρασία
    h = dht.readHumidity();     // υγρασία
    lastSensorRead = now;
  }
}

/*
  updateScrolling(now)
  --------------------
  Δεν λέμε "πήγαινε -2 pixels κάθε φορά",
  αλλά λέμε "πήγαινε με 45 pixels/δευτερόλεπτο".

  Αυτό γίνεται με dt:
  dt = χρόνος που πέρασε σε δευτερόλεπτα.
*/
void updateScrolling(unsigned long now) {
  // Αν είναι η πρώτη φορά, αρχικοποιούμε
  if (lastAnimTime == 0) lastAnimTime = now;

  // dt σε δευτερόλεπτα
  float dt = (now - lastAnimTime) / 1000.0f;
  lastAnimTime = now;

  // Κίνηση προς τα αριστερά
  scrollX -= SCROLL_SPEED_PX_PER_SEC * dt;

  // Όταν φύγει τελείως αριστερά, ξαναμπαίνει από δεξιά
  if (scrollX < -(float)scrollTextWidth) {
    scrollX = (float)SCREEN_WIDTH;
  }
}

// =======================================================
// 10) RENDER (ζωγραφίζει την οθόνη)
// =======================================================

/*
  renderScreen()
  --------------
  Αυτή η συνάρτηση ζωγραφίζει ΟΛΗ την οθόνη:
  - πάνω scrolling
  - γραμμές
  - θερμοκρασία / υγρασία ή NO DATA
*/
void renderScreen() {
  // Καθαρίζουμε την οθόνη (σβήνουμε ό,τι υπήρχε)
  display.clearDisplay();

  // -------- ΠΑΝΩ ΜΠΑΡΑ: scrolling text --------
  display.setTextSize(1);
  display.setCursor((int)scrollX, 4);   // cast σε int για σχεδίαση
  display.print(scrollingText);

  // -------- Γραμμές διαχωρισμού --------
  display.drawLine(0, SEP_Y, 127, SEP_Y, WHITE);      // οριζόντια γραμμή
  display.drawLine(HALF_W, SEP_Y, HALF_W, 63, WHITE);  // κάθετη γραμμή

  // Συντεταγμένες για αριστερό/δεξί μισό
  int leftX = 0;
  int rightX = HALF_W;

  // -------- Αν δεν έχουμε δεδομένα από αισθητήρα --------
  if (isnan(t) || isnan(h)) {
    // Μεγάλο μήνυμα
    drawCenteredTextInBox(0, BODY_Y, SCREEN_WIDTH, BODY_H, "NO DATA", 2);

    // Μικρό μήνυμα από κάτω (λίγο πιο πάνω για να μην κόβεται)
    display.setTextSize(1);
    display.setCursor(28, BODY_Y + 38);
    display.print(F("Check sensor"));

    // Εμφάνιση στην οθόνη και τέλος
    display.display();
    return;
  }

  // -------- Labels + Icons --------

  // Θερμόμετρο (αριστερά)
  drawThermometerIcon(leftX + 6, BODY_Y + 2);
  display.setTextSize(1);
  display.setCursor(leftX + 20, BODY_Y + 4);
  display.print(F("TEMP"));

  // Σταγόνα (δεξιά)
  drawDropIcon(rightX + 6, BODY_Y + 2);
  display.setTextSize(1);
  display.setCursor(rightX + 20, BODY_Y + 4);
  display.print(F("HUM"));  // συντομογραφία για να χωράει

  // -------- Περιοχή τιμών --------
  const int valueY = BODY_Y + 14;
  const int valueH = BODY_H - 14;

  // Θερμοκρασία (17.3 + C)
  drawCenteredTempValue(leftX, valueY, HALF_W, valueH, t);

  // Υγρασία (ακέραιο %)
  String humStr = String((int)h) + " %";
  drawCenteredTextInBox(rightX, valueY, HALF_W, valueH, humStr, 2);

  // Τέλος: εμφανίζουμε όλα όσα σχεδιάσαμε
  display.display();
}

// =======================================================
// 11) SETUP (τρέχει ΜΙΑ φορά)
// =======================================================
void setup() {
  Serial.begin(9600);
  dht.begin();

  // Αρχικοποίηση OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED Error"));
    for (;;); // σταματάει εδώ αν δεν βρει την OLED
  }

  // Προσανατολισμός οθόνης
  display.setRotation(0);

  // Πολύ σημαντικό:
  // Αν είναι true, όταν το κείμενο ξεκινά εκτός οθόνης,
  // μπορεί να "πηδήξει" γραμμή. Το κλείνουμε.
  display.setTextWrap(false);

  // Χρώμα κειμένου (στη μονόχρωμη OLED είναι WHITE)
  display.setTextColor(WHITE);

  // Υπολογίζουμε 1 φορά το πλάτος του scrolling κειμένου
  int16_t x1, y1;
  uint16_t w, hgt;
  display.setTextSize(1);
  display.getTextBounds(scrollingText, 0, 0, &x1, &y1, &w, &hgt);
  scrollTextWidth = (int)w;

  // Ρυθμίζουμε τους timers ώστε να ξεκινήσουν αμέσως
  unsigned long now = millis();
  lastSensorRead = now - SENSOR_INTERVAL_MS;     // διαβάζει άμεσα
  lastDisplayUpdate = now;
  lastAnimTime = now;
}

// =======================================================
// 12) LOOP (τρέχει συνέχεια)
// =======================================================
void loop() {
  // Παίρνουμε την τρέχουσα "ώρα" (ms από την εκκίνηση)
  unsigned long now = millis();

  // 1) Διαβάζουμε αισθητήρα όταν πρέπει
  updateSensor(now);

  // 2) Ενημερώνουμε το scrolling (ομαλή κίνηση)
  updateScrolling(now);

  // 3) Ανανεώνουμε την οθόνη περίπου 30 φορές/δευτερόλεπτο
  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdate = now;
    renderScreen();
  }

  // Δεν έχουμε delay(), άρα το Arduino παραμένει "γρήγορο"
  // και μπορούμε να προσθέσουμε:
  // - κουμπιά
  // - μενού
  // - σειριακές εντολές
  // - alarms
  // χωρίς να κολλάει η οθόνη.
}
