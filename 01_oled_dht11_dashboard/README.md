# 01 - OLED DHT11 Mini Dashboard

## Φωτογραφία σύνδεσης
![arduino_02](arduino_02.jpg)


Μικρό Arduino project για αρχάριους που εμφανίζει σε OLED 128x64:

- Scrolling τίτλο στην πάνω μπάρα
- Θερμοκρασία (με 1 δεκαδικό)
- Υγρασία (%)
- Εικονίδια (θερμόμετρο + σταγόνα)
- Χωρίς delay() (timers με millis)

## Υλικά
- Arduino UNO/Nano
- OLED SSD1306 128x64 (I2C, 0x3C)
- DHT11
- Καλώδια

## Συνδεσμολογία
### OLED
- VCC → 5V
- GND → GND
- SDA → A4
- SCL → A5

### DHT11
- VCC → 5V
- GND → GND
- DATA → D2
