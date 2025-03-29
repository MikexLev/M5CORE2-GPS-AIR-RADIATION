#include <M5Core2.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <cactus_io_BME280_I2C.h>
#include <MiCS6814-I2C.h>
#include "FONT1.h"
#include "FONT2.h"
#include "CITIES.h"
#include "moonPhase.h"
moonPhase moon;

float lastPressure = -1;  // -1 statt 0, damit der erste Vergleich funktioniert
String lastWeatherIcon = "";
String lastArrowIcon = "";
bool selectingCountry = true;
bool selectingCity = false;

BME280_I2C bme(0x76);  //i2C PA_SDA 32,PA_SCL 33
TinyGPSPlus gps;

// Struktur für Zeit
struct tm timeinfo = {};

void updateTimeFromGPS() {
  if (gps.date.isValid() && gps.time.isValid()) {
    
    // GPS-Daten in `struct tm` speichern
    timeinfo.tm_year = gps.date.year() - 1900;  // tm_year zählt ab 1900
    timeinfo.tm_mon = gps.date.month() - 1;     // tm_mon zählt ab 0 (Jan = 0)
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();

    // `struct tm` in UNIX-Zeit umwandeln
   time_t gps_time = mktime(&timeinfo);

    Serial.printf("GPS-Zeit: %04d-%02d-%02d %02d:%02d:%02d\n",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());

    Serial.printf("UNIX-Timestamp: %ld\n", gps_time);

    // Mondphase mit richtiger Zeit aufrufen
    moonData_t moonData = moon.getPhase(gps_time);
  }
}
unsigned long lastGPSTimeUpdate = 0;  // Letzte Aktualisierung der Uhrzeit

const int gpsUpdateInterval = 100;      // GPS-Anzeige nur alle 100 ms aktualisieren
const float CO_THRESHOLD = 30.0;        // CO: gefährlich ab 30 ppm
const float NH3_THRESHOLD = 25.0;       // NH3: gefährlich ab 25 ppm
const float NO2_THRESHOLD = 10.0;       // NO2: gefährlich ab 10 ppm
const float EMF_THRESHOLD = 50.0;       // EMF: gefährlich ab 20
const float RADIATION_THRESHOLD = 6.0;  // Strahlung: gefährlich ab 5.0 µSv/h
const float alpha = 0.2;                // Glättungsfaktor für GPS (zwischen 0 und 1)
HardwareSerial MySerial(1);             // Verwende den zweiten Hardware-Serial-Port

Adafruit_ADS1115 ads;
double homeLat = 0.0;
double homeLon = 0.0;
static const int RXPin = 13, TXPin = 14;
static const uint32_t GPSBaud = 38400;

const int geigerPin = 26;               // GPIO für Geigerzähler
volatile unsigned long pulseCount = 0;  // Impulszähler
volatile bool drawBitmapFlag = false;   // Flag, um Bitmap zu zeichnen

void drawPNGGeigerSignal() {
  if (SD.exists("/radiation.png")) {

    M5.Lcd.drawPngFile(SD, "/radiation.png", 198, 101);
    delay(300);
  } else {
    Serial.println("Fehler: radiation.png nicht gefunden!");
  }
}

float doseRate = 0.0;     // Dosis in µSv/h
float averageDose = 0.0;  // Durchschnittliche Dosis

const float calibrationFactor = 1.0;  //108.0; 6.0;  // Kalibrierung: CPM pro µSv/h
// Historie für Durchschnittswerte
#define RATE_GRAPH_WIDTH 83
#define AVG_GRAPH_WIDTH 17
#define MOON_PHASES_PATH "/phases/"  // Ordner für die PNG-Dateien auf der SD-Karte

float avgGraphBuffer[AVG_GRAPH_WIDTH] = { 0 };
int avgGraphIndex = 0;
int rateGraphBuffer[RATE_GRAPH_WIDTH] = { 0 };
int rateGraphIndex = 0;
const int numHistory = 80;              // Historie für 60 Sekunden
float doseHistory[numHistory] = { 0 };  // Speicherung der letzten Werte
int historyIndex = 0;                   // Index für den Historienpuffer

unsigned long lastCount = 0;  // Letzte Impulszählung

float el = 0;
float az = 0;
float el_r = 0;
float az_r = 0;
float e = 0;

const float rad_fac = 0.017453292;
const float pi = 3.1415926536;
String nearestCity = "";     // Speichert den Stadtnamen
String nearestCountry = "";  // Speichert das Land
float distance(float lat1, float lon1, float lat2, float lon2) {
  float dlat = radians(lat2 - lat1);
  float dlon = radians(lon2 - lon1);
  float a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) * sin(dlon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371.0 * c;  // Entfernung in km
}
#define NUM_CITIES (sizeof(cities) / sizeof(cities[0]))  // Falls cities[] ein Array ist
void findNearestCity(float lat, float lon, String &city, String &country) {
  float minDistance = 9999999;

  for (int i = 0; i < NUM_CITIES; i++) {
    float d = distance(lat, lon, cities[i].lat, cities[i].lon);
    if (d < minDistance) {
      minDistance = d;
      city = cities[i].name;
      country = cities[i].country;
    }
  }
}

int seconds = 0;
int x = 0;
int y = 0;
int q = 0;
int r = 0;
int i = 0;
int j = 0;

bool GPSnotReady = false;
bool sensorConnected;

SoftwareSerial ss(RXPin, TXPin);

static const int MAX_SATELLITES = 99;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1);  // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);       // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);          // $GPGSV sentence, third element
TinyGPSCustom satNumber[4];                         // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

TinyGPSCustom prn[] = {
  TinyGPSCustom(gps, "GPGSV", 4),
  TinyGPSCustom(gps, "GPGSV", 8),
  TinyGPSCustom(gps, "GPGSV", 12),
  TinyGPSCustom(gps, "GPGSV", 16)
};

// Satelliten-Datenstruktur
struct {
  bool active;
  int prn;  // PRN-Nummer des Satelliten
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

bool alarmTriggered = false;

//Vibration und LED-Muster auslösen
void triggerVibrationPattern(const int pattern[], int len) {
  for (int i = 0; i < len; i++) {
    if (i % 2 == 0) {  // Gerade Indizes -> AN
      M5.Axp.SetVibration(true);
      M5.Axp.SetLed(true);
    } else {  // Ungerade Indizes -> AUS
      M5.Axp.SetVibration(false);
      M5.Axp.SetLed(false);
    }
    delay(pattern[i]);
  }
  
  // Nach dem Muster sicherstellen, dass alles aus ist
  M5.Axp.SetVibration(false);
  M5.Axp.SetLed(false);
}

// Alarm visuell anzeigen
void showAlarm(const char *pngFile, int textCursorX, int textCursorY, const char *message) {
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setCursor(textCursorX, textCursorY);
  M5.Lcd.print(message);

  M5.Lcd.drawPngFile(SD, pngFile, 22, 31);
}
void checkForAlarms(float CO, float NH3, float NO2, float EMF, float radiation) {
  alarmTriggered = false;  // Lokales Flag zurücksetzen

  // CO-Warnung
  if (CO > CO_THRESHOLD) {
    showAlarm("/hazard.png", 13, 79, " !!!DANGER!!!");
    int patternCO[] = { 400, 100, 200, 100, 400 };
    triggerVibrationPattern(patternCO, sizeof(patternCO) / sizeof(patternCO[0]));
    alarmTriggered = true;
  }

  // NH3-Warnung
  if (NH3 > NH3_THRESHOLD) {
    showAlarm("/hazard.png", 13, 90, " !!!DANGER!!!");
    int patternNH3[] = { 400, 100, 400, 100, 200 };
    triggerVibrationPattern(patternNH3, sizeof(patternNH3) / sizeof(patternNH3[0]));
    alarmTriggered = true;
  }

  // NO2-Warnung
  if (NO2 > NO2_THRESHOLD) {
    showAlarm("/hazard.png", 13, 101, " !!!DANGER!!!");
    int patternNO2[] = { 200, 100, 200, 100, 400 };
    triggerVibrationPattern(patternNO2, sizeof(patternNO2) / sizeof(patternNO2[0]));
    alarmTriggered = true;
  }

  // EMF-Warnung
  if (EMF > EMF_THRESHOLD) {
    showAlarm("/EMF.png", 13, 112, " !!!DANGER!!!");
    int patternEMF[] = { 200, 100, 200, 100, 100 };
    triggerVibrationPattern(patternEMF, sizeof(patternEMF) / sizeof(patternEMF[0]));
    alarmTriggered = true;
  }

  // Strahlungswarnung
  if (radiation > RADIATION_THRESHOLD) {
    M5.Lcd.drawPngFile(SD, "/radiation3.png", 237, 31);
    int patternRadiation[] = { 200, 100, 400, 100, 200 };
    triggerVibrationPattern(patternRadiation, sizeof(patternRadiation) / sizeof(patternRadiation[0]));
    alarmTriggered = true;
  }
}

// Speichert die x- und y-Koordinaten sowie die Kreisgröße der Satelliten für das Löschen alter Kreise
int oldX[MAX_SATELLITES] = { 0 };
int oldY[MAX_SATELLITES] = { 0 };
int oldCircleSize1[MAX_SATELLITES] = { 0 };
int oldCircleSize2[MAX_SATELLITES] = { 0 };

bool displayOn = true;  // Status des Displays

void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
void displayMillisecondProgress();
void printStr(const char *str, int len);

String savedCity = "N/A";
void updateNearestCity(float latitude, float longitude) {
  findNearestCity(latitude, longitude, savedCity, nearestCountry);
}

// Interrupt-Funktion: Zähle Geigerzähler-Impulse und setze das Flag
void IRAM_ATTR countPulse() {
  pulseCount++;  // Impuls zählen
  drawBitmapFlag = true;  // Bitmap-Zeichen-Flag setzen
}
unsigned long lastUpdateTime = 0;            // Letztes Update der Mondphase
const unsigned long updateInterval = 60000;  // 60 Sekunden für das Mondphase-Update

int calculateCET(TinyGPSDate &date, TinyGPSTime &time) {
  int year = date.year();
  int month = date.month();
  int day = date.day();

  // Zeller's Kongruenz zur Berechnung des Wochentags (0 = Samstag, 1 = Sonntag, ..., 6 = Freitag)
  int m = month;
  int y = year;

  if (m < 3) {
    m += 12;
    y -= 1;
  }

  int k = y % 100;
  int c = y / 100;
  int weekday = (day + (13 * (m + 1)) / 5 + k + (k / 4) + (c / 4) - (2 * c)) % 7;
  weekday = (weekday + 6) % 7;  // Anpassen: 0 = Sonntag, 1 = Montag, ..., 6 = Samstag

  // Funktion zum Berechnen des letzten Sonntags eines Monats
  auto getLastSunday = [](int y, int m) -> int {
    // Ersten Tag des nächsten Monats berechnen
    int nextMonth = (m == 12) ? 1 : m + 1;
    int nextYear = (m == 12) ? y + 1 : y;
    struct tm firstNextMonth = { 0, 0, 0, 1, nextMonth - 1, nextYear - 1900 };
    time_t firstNextMonthTime = mktime(&firstNextMonth);
    time_t lastSundayTime = firstNextMonthTime - (24 * 3600);  // Einen Tag zurück

    while (localtime(&lastSundayTime)->tm_wday != 0) {  // Bis Sonntag gefunden wird
      lastSundayTime -= 24 * 3600;
    }
    return localtime(&lastSundayTime)->tm_mday;
  };

  // Sommerzeit: Letzter Sonntag im März bis letzter Sonntag im Oktober
  int lastSundayMarch = getLastSunday(year, 3);
  int lastSundayOctober = getLastSunday(year, 10);

  if ((month > 3 && month < 10) || (month == 3 && day >= lastSundayMarch) || (month == 10 && day < lastSundayOctober)) {
    return 2;  // Sommerzeit (UTC+2)
  } else {
    return 1;  // Winterzeit (UTC+1)
  }
}

void setup() {
  Serial.begin(115200);
  M5.begin();
  pinMode(geigerPin, INPUT);                                              // Setze PIN 26 als Eingang
  attachInterrupt(digitalPinToInterrupt(geigerPin), countPulse, RISING);  // Rufe countPulse() bei steigender Flanke auf
  M5.Lcd.fillScreen(BLACK);
  MySerial.begin(38400, SERIAL_8N1, 13, 14);  // RX = 13, TX = 14, Baudrate auf 38400 setzen

  Wire.begin(32, 33);

  bme.begin();                                // Sensor initialisieren
  lastPressure = bme.getPressure_HP() / 100;  // Ersten Druckwert setzen

  ss.begin(GPSBaud);
  // I2C Initialisierung
  if (!ads.begin()) {
    Serial.println("Fehler: ADS1115 nicht gefunden!");
    while (1);
  }
  ads.setGain(GAIN_ONE);  // Verstärkung setzen (1x = ±4.096V)

  // Initialisiere den Rate-Puffer mit neutralen Y-Werten (mittlere Höhe)
  for (int i = 0; i < RATE_GRAPH_WIDTH; i++) {
    rateGraphBuffer[i] = 90;  // Setze Startwert auf die Mitte des Bereichs
  }
  if (!SD.begin(TFCARD_CS_PIN, SPI, 40000000UL)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden! Neustart...");
    ESP.restart();
  }

  // Koordinaten aus Datei lesen
  File myFile = SD.open("/home_coordinates.txt");
  if (myFile) {
    String coordinates = myFile.readStringUntil('\n');  // Lese gespeicherte Werte
    myFile.close();

    int commaIndex = coordinates.indexOf(',');
    if (commaIndex > 0) {
      homeLat = coordinates.substring(0, commaIndex).toFloat();
      homeLon = coordinates.substring(commaIndex + 1).toFloat();
      updateNearestCity(homeLat, homeLon);
    }
  } else {
    Serial.println("Keine gespeicherten Koordinaten gefunden.");
  }

  bme.setTempCal(-1);

  Serial.println("Starte GPS...");

  bool pngDrawn = false;  // set this variable to 'false' to ensure that the PNG has not yet been drawn

  while (!pngDrawn) {            // Loop that keeps running until the display is tapped
    if (M5.Touch.ispressed()) {  // If the display was typed, set pngDrawn to 'true'
      pngDrawn = true;
    } else {  // Else draw the PNG
      M5.update();

      M5.Lcd.drawPngFile(SD, "/radar1.png", 0, 40);

      File myFile = SD.open("/home_coordinates.txt", FILE_READ);
      if (myFile) {
        String line = myFile.readStringUntil('\n');
        int commaIndex = line.indexOf(",");
        if (commaIndex != -1) {
          homeLat = line.substring(0, commaIndex).toDouble();
          homeLon = line.substring(commaIndex + 1).toDouble();
        } else {
          Serial.println("Fehler: Dateiinhalt ungültig!");
        }
        myFile.close();
      } else {
        Serial.println("Datei '/home_coordinates.txt' nicht gefunden!");
      }

      //Finde nächste Stadt anhand der gespeicherten Position
      findNearestCity(homeLat, homeLon, nearestCity, nearestCountry);

      M5.Lcd.drawPngFile(SD, "/GARPAX.png", 250, 0);
      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setTextColor(DARKCYAN, BLACK);
      M5.Lcd.setCursor(10, 40);
      M5.Lcd.print("MULITISENS-GARPAX");
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.setCursor(100, 80);
      M5.Lcd.print("SAVED COORDINATES");
      M5.Lcd.setFreeFont(&SFChromeFendersCondensed16pt7b);
      M5.Lcd.setCursor(133, 164);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.print(nearestCity);
      M5.Lcd.setCursor(160, 130);
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.print(nearestCountry);
      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setCursor(126, 205);
      M5.Lcd.print("LATT: N ");
      M5.Lcd.setTextFont(1);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(196, 190);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setCursor(206, 205);
      M5.Lcd.print(homeLat, 6);
      M5.Lcd.setCursor(120, 228);
      M5.Lcd.print("LONG: E ");
      M5.Lcd.setTextFont(1);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(195, 214);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setCursor(206, 228);
      M5.Lcd.print(homeLon, 6);
    }
  }
  for (int i = 0; i < 4; ++i) {
    satNumber[i].begin(gps, "GPGSV", 4 + 4 * i);
    elevation[i].begin(gps, "GPGSV", 5 + 4 * i);
    azimuth[i].begin(gps, "GPGSV", 6 + 4 * i);
    snr[i].begin(gps, "GPGSV", 7 + 4 * i);
  }
  //GRAPHIC

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.drawRoundRect(0, 0, 320, 240, 8, CYAN);
  M5.Lcd.drawRoundRect(4, 24, 312, 104, 6, CYAN);
  M5.Lcd.drawRoundRect(4, 133, 312, 101, 6, CYAN);
  M5.Lcd.drawRoundRect(8, 28, 90, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(224, 28, 88, 96, 4, BLUE);
  M5.Lcd.drawRoundRect(8, 136, 90, 96, 4, BLUE);

  M5.Lcd.setTextFont(1);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(11, 233);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print("> ");
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print(savedCity);  // Hier wird die gespeicherte Position angezeigt
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(" <");
  M5.Lcd.print("> HOME POS <");
  M5.Lcd.setCursor(230, 233);
  M5.Lcd.print("> SVALBARD <");
}

// Funktion zur Berechnung eines Farbverlaufs von Rot (niedrig) nach Grün (hoch)
uint16_t getGradientColor(float value, float minValue, float maxValue) {
  // Normiere den Wert auf den Bereich [0, 1]
  float normalized = (value - minValue) / (maxValue - minValue);
  normalized = constrain(normalized, 0.0, 1.0);  // Begrenze auf [0, 1]

  // Umkehrung der Farben: Niedrige Werte = Rot, Hohe Werte = Grün
  uint8_t red = 255 * (1.0 - normalized);  // Rot wird intensiver bei niedrigeren Werten
  uint8_t green = 255 * normalized;        // Grün wird intensiver bei höheren Werten
  uint8_t blue = 0;                        // Blau bleibt konstant (0)

  // Konvertiere RGB in 16-Bit-Farbe (565-Format)
  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}

void drawRateGraph(float doseRate) {
  static float sumDoseRate = 0;
  static int countSamples = 0;

  sumDoseRate += doseRate;
  countSamples++;

  if (countSamples >= 7) {                                     // Alle 7 Sekunden einen neuen Punkt setzen
    int newY = 80 - ((sumDoseRate / countSamples) * 10);       // Berechne neuen Y-Wert
    newY = constrain(newY, 40, 80);                            // Begrenze Y-Wert auf Diagrammbereich
    rateGraphBuffer[rateGraphIndex] = newY;                    // Speichere neuen Y-Wert
    rateGraphIndex = (rateGraphIndex + 1) % RATE_GRAPH_WIDTH;  // Zyklischer Puffer

    sumDoseRate = 0;
    countSamples = 0;
  }

  // Lösche den Bereich für das Diagramm
  M5.Lcd.fillRect(225, 31, 86, 60, BLACK);  // Startpunkt jetzt bei 223

  // Zeichne **horizontale Skalenlinien** für Dosiswerte
  for (int y = 40; y <= 120; y += 10) {  // Hauptlinien alle 10 µSv/h
    M5.Lcd.drawFastHLine(225, y, 85, 0x0320);
  }

  // Zeichne **vertikale Linien** für Zeitachse jetzt alle 8 Pixel
  for (int x = 225; x <= 309; x += 14) {
    M5.Lcd.drawFastVLine(x, 40, 40, 0x0320);
  }

  // Zeichne das Diagramm mit Farbverlauf
  for (int i = 0; i < RATE_GRAPH_WIDTH - 1; i++) {
    int x1 = 226 + i;  // Verschoben nach links
    int y1 = rateGraphBuffer[(rateGraphIndex + i) % RATE_GRAPH_WIDTH];
    int x2 = 226 + (i + 1);
    int y2 = rateGraphBuffer[(rateGraphIndex + i + 1) % RATE_GRAPH_WIDTH];

    // Berechne den Farbverlauf basierend auf dem Y-Wert (unten = Rot, oben = Grün)
    uint16_t color = getGradientColor(y1, 40, 80);

    // Zeichne die Linie mit dem berechneten Farbverlauf
    M5.Lcd.drawLine(x1, y1, x2, y2, color);
  }
}

void drawAverageGraph(float avgDose) {
  // Buffer aktualisieren: Durchschnittswerte für die letzten 60 Minuten in 16 Säulen
  avgGraphBuffer[avgGraphIndex] = avgDose * 10;                                     // Skaliere die Dosis (1 µSv/h = 10 Pixel)
  avgGraphBuffer[avgGraphIndex] = constrain(avgGraphBuffer[avgGraphIndex], 0, 68);  // Begrenze Höhe der Säule
  avgGraphIndex = (avgGraphIndex + 1) % AVG_GRAPH_WIDTH;                            // Zyklischer Puffer für 16 Säulen

  M5.Lcd.fillRect(226, 90, 84, 33, BLACK);

  // Zeichne die Säulen mit Abstand von 5 Pixeln
  for (int i = 0; i < AVG_GRAPH_WIDTH; i++) {
    int x = 226 + i * 5;  // X-Position für jede Säule (4 Pixel Breite + 1 Pixel Abstand)
    int height = avgGraphBuffer[(avgGraphIndex + i) % AVG_GRAPH_WIDTH];
    int y = 120 - height;  // Y-Position basierend auf der Höhe der Säule

    // Wähle Farbe basierend auf Dosiswert
    uint16_t color;
    float dose = height / 10.0;  // Rückrechnen auf µSv/h
    if (dose < 0.5) {
      color = GREEN;
    } else if (dose < 1.0) {
      color = YELLOW;
    } else if (dose < 2.0) {
      color = ORANGE;
    } else {
      color = RED;
    }
    
    // Zeichne die Säule
    M5.Lcd.fillRect(x, y, 4, height, color);
  }
}

void displayValues(float doseRate, float averageDose) {

  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(227, 31);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("DR:");
  if (doseRate > 99) {
    doseRate = 99;
  }
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.printf("%.2f uSv/h", doseRate);

  M5.Lcd.setCursor(227, 82);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("AD:");
  if (averageDose > 99) {
    averageDose = 99;
  }
  uint16_t avgColor = (averageDose < 0.5) ? GREEN : (averageDose < 1.0) ? YELLOW
                                                  : (averageDose < 2.0) ? ORANGE
                                                                        : RED;
  M5.Lcd.setTextColor(avgColor, BLACK);
  M5.Lcd.printf("%.2f uSv/h", averageDose);
}

int lastSecond = -1;  // Um Sekundenänderungen zu verfolgen

void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  unsigned long currentMillis = millis();

  if (currentMillis - lastGPSTimeUpdate >= gpsUpdateInterval) {
    lastGPSTimeUpdate = currentMillis;

    if (!d.isValid() || !t.isValid()) {
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.setCursor(6, 6);
      M5.Lcd.print(F(">> NO GPS SIGNAL <<"));
    } else {
      char sz[32];
      sprintf(sz, "%02d.%02d.%02d ", d.year(), d.month(), d.day());
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.setCursor(6, 6);
      M5.Lcd.setTextSize(2);
      M5.Lcd.print(sz);

      // UTC-Zeit vom GPS empfangen und in CET/MESZ umrechnen
      int cetOffset = calculateCET(d, t);
      int hourCET = t.hour() + cetOffset;
      if (hourCET >= 24) hourCET -= 24;  // Zeit korrekt anpassen

      // Zeit mit CET anzeigen
      unsigned long ms = currentMillis % 1000;
      sprintf(sz, "%02d:%02d:%02d", hourCET, t.minute(), t.second());
      M5.Lcd.print(sz);
    }
  }
}

// Funktion zur Ausgabe eines Strings mit fester Länge
void printStr(const char *str, int len) {
  int slen = strlen(str);  // Länge des Strings berechnen
  for (int i = 0; i < len; ++i) {
    Serial.print(i < slen ? str[i] : ' ');  // Falls der String kürzer als len ist, Leerzeichen hinzufügen
  }
}

// Funktion zum Aktualisieren des Icons
void updateWeatherIcon(const String &newIcon) {
  if (newIcon != lastWeatherIcon) {
    M5.Lcd.fillRoundRect(100, 26, 24, 24, 4, BLACK);
    if (SD.exists(newIcon.c_str())) {
      M5.Lcd.drawPngFile(SD, newIcon.c_str(), 100, 26);
    } else {
      Serial.print("Fehler: Wetter-Icon nicht gefunden -> ");
      Serial.println(newIcon);
    }
    lastWeatherIcon = newIcon;
  }
}

// Funktion zum Aktualisieren des Pfeil-Icons für Luftdrucktendenz
void updateArrowIcon(const String &arrowIcon) {
  if (arrowIcon != lastArrowIcon) {
    M5.Lcd.fillRect(82, 54, 9, 9, BLACK);

    if (!arrowIcon.isEmpty() && SD.exists(arrowIcon.c_str())) {
      M5.Lcd.drawPngFile(SD, arrowIcon.c_str(), 81, 53);
    } else {
      Serial.print("Fehler: Pfeil-Icon nicht gefunden -> ");
      Serial.println(arrowIcon);
    }

    lastArrowIcon = arrowIcon;
  }
}

// Funktion zum Aktualisieren der Wetteranzeige mit Druck-Tendenz
void updateWeatherDisplay() {
  bme.readSensor();

  float pressure = bme.getPressure_HP() / 100;
  String weatherIcon;
  String arrowIcon = lastArrowIcon;  // Standardmäßig das letzte Icon behalten

  M5.Lcd.setTextSize(1);
  M5.Lcd.drawRoundRect(12, 31, 82, 11, 2, 0x00AF);
  M5.Lcd.setCursor(16, 33);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("T:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getTemperature_C(), 1);
  M5.Lcd.println(" C");
  M5.Lcd.drawRoundRect(12, 42, 82, 11, 2, 0x00AF);
  M5.Lcd.setCursor(16, 44);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("H:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getHumidity(), 0);
  M5.Lcd.println(" %");
  M5.Lcd.drawRoundRect(12, 53, 82, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(12, 64, 82, 12, 2, 0x00AF);
  M5.Lcd.setCursor(16, 55);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("P:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(pressure, 0);
  M5.Lcd.println(" HPa");
  M5.Lcd.setCursor(16, 67);
  if (pressure <= 970) {
    M5.Lcd.print(">> STORM <<");
    weatherIcon = "/weather/05.png";
  } else if (pressure <= 1001) {
    weatherIcon = (bme.getTemperature_C() > 1) ? "/weather/04.png" : "/weather/03.png";
  } else if (pressure <= 1010) {
    M5.Lcd.print("> OVERCAST <");
    weatherIcon = "/weather/02.png";
  } else if (pressure <= 1020) {
    M5.Lcd.print(">> CLOUDY <<");
    weatherIcon = "/weather/01.png";
  } else {
    M5.Lcd.print(">> CLEAR <<");
    weatherIcon = "/weather/00.png";
  }

  // Druck-Trend bestimmen
  if (lastPressure >= 0) {
    float pressureDiff = pressure - lastPressure;  // Differenz berechnen

    if (abs(pressureDiff) >= 0.5) {  // Nur ändern, wenn Unterschied ≥ 0.5 hPa
      if (pressureDiff > 0) {
        arrowIcon = "/icons/up.png";
      } else {
        arrowIcon = "/icons/down.png";
      }
    }
  }

  // Icons aktualisieren
  updateWeatherIcon(weatherIcon);
  updateArrowIcon(arrowIcon);

  // Letzten Druckwert speichern
  lastPressure = pressure;
}

void loop() {
  M5.update();  // Touch-Events aktualisieren
  //M5.Lcd.fillRect(198, 25, 23, 23, WHITE);  // Lösche alten Wert
  //unsigned long loopStart = millis();       // Startzeit für gleichmäßige Updates
  // GPS-Daten sofort einlesen, sobald sie ankommen
  while (MySerial.available()) {
    gps.encode(MySerial.read());
  }
  printDateTime(gps.date, gps.time);  // Zeigt Echtzeit mit Millisekunden an
  updateTimeFromGPS();                // Aktualisiert Zeit & Mondphase
  if (drawBitmapFlag) {
    drawPNGGeigerSignal();   // Zeichne das Bitmap
    drawBitmapFlag = false;  // Zurücksetzen des Flags
  }

  if (gps.date.isValid() && gps.time.isValid()) {
    
    // Datum und Zeit aus GPS auslesen
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();

    // Sommerzeitberechnung (CET) aufrufen
    int cet = calculateCET(gps.date, gps.time);  // Berechnung von Sommerzeit (CET)

    // Je nach Sommerzeit (CET) die Uhrzeit umstellen
    int hour = gps.time.hour() + ((cet == 2) ? 2 : 1);  // UTC+2 für Sommerzeit, UTC+1 für Winterzeit

    // Überprüfen, ob es Zeit für ein Update der Mondphase ist
    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdateTime >= updateInterval) {
      
      // Mondphase berechnen und anzeigen
      time_t gps_time = mktime(&timeinfo);                       // UNIX-Zeit berechnen
      moonData_t moonData = moon.getPhase(gps_time);             // Korrekte Funktion nutzen
      int phaseIndex = (int)(moonData.angle / 360.0 * 28) % 28;  // Umrechnung auf 28 Phasen

      // Mondphase-File erzeugen
      char filename[32];
      snprintf(filename, sizeof(filename), "%s%02d.png", MOON_PHASES_PATH, phaseIndex);

      // Mondphase auf dem Display anzeigen

      M5.Lcd.fillRoundRect(199, 25, 24, 24, 6, BLACK);  // Vorherige Anzeige löschen
      M5.Lcd.drawPngFile(SD, filename, 200, 29);        // Mondphase anzeigen

      // Zeit des letzten Updates setzen
      lastUpdateTime = currentMillis;
    }
  }
  if (M5.Touch.ispressed()) {                            // Prüfe, ob der Bildschirm berührt wird
    TouchPoint_t touchPoint = M5.Touch.getPressPoint();  // Hol die Berührungskoordinaten

    // Bereich unten links (10x10 mm) zum Ein- und Ausschalten
    if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 140 && touchPoint.y <= 240) {
      if (displayOn) {
        M5.Lcd.sleep();  // Display ausschalten
        displayOn = false;
      } else {
        M5.Lcd.wakeup();  // Display einschalten
        displayOn = true;
      }
    }
  }
  M5.Lcd.fillRect(198, 104, 23, 23, BLACK);  // Lösche alten Wert
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= 1000) {
    lastUpdate = now;

    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    doseRate = count / calibrationFactor;

    static float sumDose = 0;     // Summe der Dosiswerte
    static int countSamples = 0;  // Anzahl der Samples pro Säule
    sumDose += doseRate;
    countSamples++;

    // Alle 3,75 Minuten (225 Sekunden) aktualisieren
    if (countSamples >= 225) {  // Alle 3,75 Minuten
      float avgDose = sumDose / countSamples;
      drawAverageGraph(avgDose);
      sumDose = 0;
      countSamples = 0;
    }
    drawRateGraph(doseRate);
    if (countSamples > 0) {
      displayValues(doseRate, sumDose / countSamples);
    } else {
      displayValues(doseRate, 0);
    }
  }
  float radiation = doseRate;  // Strahlung in µSv/h

  //GPS INI
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  //SATELLITE TRACKER
  if (ss.available() > 0) {
    gps.encode(ss.read());
    if (totalGPGSVMessages.isUpdated()) {
      for (int i = 0; i < 4; ++i) {
        int no = atoi(satNumber[i].value());
        Serial.print(F("SatNumber is "));
        Serial.println(no);

        if (no >= 1 && no <= MAX_SATELLITES) {
          sats[no - 1].elevation = atoi(elevation[i].value());
          sats[no - 1].azimuth = atoi(azimuth[i].value());
          sats[no - 1].snr = atoi(snr[i].value());
          sats[no - 1].active = true;
        }
      }

      int totalMessages = atoi(totalGPGSVMessages.value());
      int currentMessage = atoi(messageNumber.value());
      if (totalMessages == currentMessage)
        Serial.print(F("Sats="));
      Serial.print(gps.satellites.value());
      Serial.print(F(" Nums="));
      for (int i = 0; i < MAX_SATELLITES; ++i) {
        if (sats[i].active) {
          Serial.print(i + 1);
          Serial.print(F(" "));
        }
      }
      GPSnotReady = ((gps.location.lat() == 0) && (gps.location.lng() == 0));
    }
  }
  
  //LAST SAVED COORDINATES
  File myFile = SD.open("/home_coordinates.txt", FILE_READ);
  if (myFile) {
    String line = myFile.readStringUntil('\n');
    int commaIndex = line.indexOf(",");
    if (commaIndex != -1) {
      homeLat = line.substring(0, commaIndex).toDouble();
      homeLon = line.substring(commaIndex + 1).toDouble();
    }
    myFile.close();
  }
  //TOUCH BUTTONS
  if (M5.BtnA.wasPressed()) {
    M5.Lcd.fillRoundRect(16, 233, 280, 9, 4, BLACK);
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.drawFastHLine(21, 233, 270, CYAN);
    M5.Lcd.drawFastHLine(21, 239, 270, CYAN);

    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("POSITION SAVED");

    if (gps.location.isValid()) {
      homeLat = gps.location.lat();
      homeLon = gps.location.lng();

      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.setCursor(108, 157);
      M5.Lcd.print("LATT:N");
      M5.Lcd.setTextSize(1);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setTextSize(2);
      M5.Lcd.print(homeLat, 6);
      M5.Lcd.setCursor(108, 177);
      M5.Lcd.print("LONG:E");
      M5.Lcd.setTextSize(1);
      M5.Lcd.print("\xF7   ");
      M5.Lcd.setTextSize(2);
      M5.Lcd.print(homeLon, 6);

      File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
      if (myFile) {
        myFile.print(homeLat, 6);
        myFile.print(",");
        myFile.println(homeLon, 6);
        myFile.close();
        M5.Lcd.setTextSize(1);
        updateNearestCity(homeLat, homeLon);
        displaySavedLocation();
        M5.Lcd.print("> HOME POS <");
        M5.Lcd.setCursor(230, 233);
        M5.Lcd.print("> SVALBARD <");
      } else {
        Serial.println("ERROR WRITE FILE");
      }
    } else {
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(108, 137);
      M5.Lcd.print("STATUS NOT SAVED");
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.setCursor(108, 157);
      M5.Lcd.print("NO GPS");
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(18, 233);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.print("  N/A  ");
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.print("<");
      M5.Lcd.drawFastHLine(64, 233, 100, CYAN);
      M5.Lcd.drawFastHLine(64, 239, 100, CYAN);
      M5.Lcd.setCursor(120, 233);
      M5.Lcd.print("> HOME POS <");
      M5.Lcd.setCursor(230, 233);
      M5.Lcd.print("> SVALBARD <");
    }
    delay(3000);
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  }
  if (M5.BtnB.wasPressed()) {
    M5.Lcd.fillRoundRect(16, 233, 280, 9, 4, BLACK);
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.drawFastHLine(21, 233, 270, CYAN);
    M5.Lcd.drawFastHLine(21, 239, 270, CYAN);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("HOME COORDINATES");
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT:N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("51.861120");
    M5.Lcd.setTextSize(2);
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG:E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(" 8.289310");
    delay(3000);
    M5.Lcd.setTextSize(1);
    File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(51.861120, 6);
      myFile.print(",");
      myFile.println(8.289310, 6);
      myFile.close();
      homeLat = 51.861120;
      homeLon = 8.289310;
      updateNearestCity(homeLat, homeLon);
      displaySavedLocation();  // Anzeige aktualisieren
      M5.Lcd.print("> HOME POS <");
      M5.Lcd.setCursor(230, 233);
      M5.Lcd.print("> SVALBARD <");
      M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    } else {
      Serial.println("ERROR WRITE FILE");
    }
  }

  if (M5.BtnC.wasPressed()) {
    M5.Lcd.fillRoundRect(16, 233, 280, 9, 4, BLACK);
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.drawFastHLine(21, 233, 270, CYAN);
    M5.Lcd.drawFastHLine(21, 239, 270, CYAN);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("SVALBARD GSV");
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATT:N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("78.235680");
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG:E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("15.491320");
    delay(3000);
    M5.Lcd.setTextSize(1);
    File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(78.235680, 6);
      myFile.print(",");
      myFile.println(15.491320, 6);
      myFile.close();
      homeLat = 78.235680;
      homeLon = 15.491320;
      updateNearestCity(homeLat, homeLon);
      displaySavedLocation();  // Anzeige aktualisieren
      M5.Lcd.print("> HOME POS <");
      M5.Lcd.setCursor(230, 233);
      M5.Lcd.print("> SVALBARD <");
      M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    } else {
      Serial.println("ERROR WRITE FILE");
    }
  }

  unsigned long distanceToHome =
    (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),
                                                gps.location.lng(), homeLat, homeLon);  // 1000;
  printInt(distanceToHome, gps.location.isValid(), 9);

  double courseToHome = TinyGPSPlus::courseTo(
    gps.location.lat(), gps.location.lng(), homeLat, homeLon);
  printFloat(courseToHome, gps.location.isValid(), 7, 2);

  const char *cardinalToHome = TinyGPSPlus::cardinal(courseToHome);
  printStr(gps.location.isValid() ? cardinalToHome : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);

  M5.Lcd.drawCircle(161, 76, 47, DARKGREEN);
  M5.Lcd.drawCircle(161, 76, 16, DARKGREEN);
  M5.Lcd.drawCircle(161, 76, 32, DARKGREEN);
  M5.Lcd.drawFastHLine(115, 76, 94, DARKGREEN);
  M5.Lcd.drawFastVLine(161, 30, 94, DARKGREEN);
  M5.Lcd.drawLine(129, 44, 193, 108, DARKGREEN);
  M5.Lcd.drawLine(128, 109, 193, 44, DARKGREEN);


  // SAT DISPLAY
  int activeSatellites = 0;
  for (int i = 0; i < MAX_SATELLITES; ++i) {
    if (sats[i].active) {
      activeSatellites++;
    }
  }

  // Falls Satelliten aktiv sind, Position berechnen
  if (activeSatellites > 0) {
    int centerX = 161, centerY = 76;
    float rad_fac = 3.14159265359 / 180;

    for (int i = 0; i < MAX_SATELLITES; ++i) {
      if (sats[i].active && sats[i].snr > 1) {  // Prüfen, ob Satellit gültig ist
        float az_r = sats[i].azimuth * rad_fac;
        float e = 42 * (90 - sats[i].elevation / 2) / 90;
        int x = centerX - (sin(az_r) * e);  // Spiegelung auf die andere Seite
        int y = centerY - (cos(az_r) * e);

        // PRN-basierte Farbauswahl:
        uint16_t circleColor;
        int prn = sats[i].snr;  // Annahme, dass PRN die SNR-Nummer ist

        // Falls PRN größer als 40 ist, überspringe diesen Satelliten
        if (prn > 40) {
          continue;  // Überspringt die aktuelle Iteration der Schleife
        }

        if (prn >= 1 && prn <= 10) {
          circleColor = DARKGREEN;  // GPS-Satelliten (PRN 1 bis 20)
        } else if (prn >= 11 && prn <= 20) {
          circleColor = GREEN;  // GLONASS-Satelliten (PRN 21 bis 30)
        } else if (prn >= 21 && prn <= 30) {
          circleColor = GREENYELLOW;  // Galileo-Satelliten (PRN 31 bis 40)
        } else if (prn >= 31 && prn <= 40) {
          circleColor = YELLOW;  // Galileo-Satelliten (PRN 31 bis 40)
        }
        
        // Falls alter Punkt existiert, löschen
        if (oldX[i] > 0 && oldY[i] > 0) {
          // Lösche den alten Kreis
          M5.Lcd.drawCircle(oldX[i], oldY[i], oldCircleSize1[i], BLACK);  // Äußeren Kreis löschen
          M5.Lcd.fillCircle(oldX[i], oldY[i], oldCircleSize2[i], BLACK);  // Inneren Kreis löschen
        }

        // Größe des Kreises basierend auf SNR anpassen (größere Kreise für bessere SNR)
        int circleSize = map(sats[i].snr, 1, 40, 2, 8);  // SNR zwischen 20 und 60 auf eine Größe von 5 bis 12 anpassen

        // Zeichne den neuen Kreis
        M5.Lcd.drawCircle(x, y, circleSize, circleColor);  // Äußeren Kreis zeichnen
        M5.Lcd.fillCircle(x, y, 1, WHITE);                 // Inneren Kreis zeichnen (kleiner als der äußere Kreis)

        // Neue Position und Größe speichern
        oldX[i] = x;
        oldY[i] = y;
        oldCircleSize1[i] = circleSize;
        oldCircleSize2[i] = 1;  // Innerer Kreis ist kleiner als der äußere
      } else {
        
        // Falls Satellit nicht mehr aktiv ist, alten Punkt löschen
        if (oldX[i] > 0 && oldY[i] > 0) {
          M5.Lcd.drawCircle(oldX[i], oldY[i], oldCircleSize1[i], BLACK);  // Äußeren Kreis löschen
          M5.Lcd.fillCircle(oldX[i], oldY[i], oldCircleSize2[i], BLACK);  // Inneren Kreis löschen
          oldX[i] = oldY[i] = -1;                                         // Setze auf -1 statt 0, um Probleme zu vermeiden
          oldCircleSize1[i] = oldCircleSize2[i] = 0;                      // Lösche gespeicherte Werte
        }
      }
    }
  }
  M5.Lcd.fillRoundRect(17, 148, 72, 72, 2, BLACK);
  double relCourse = courseToHome - gps.course.deg();
  if (relCourse < 0) {
    relCourse = relCourse + 360;
  }
  if (relCourse >= 360) {
  }
  relCourse = relCourse - 360;
  if (distanceToHome < 10) {
    M5.Lcd.drawCircle(52, 184, 30, GREEN);
    M5.Lcd.drawCircle(52, 184, 20, GREENYELLOW);
    M5.Lcd.drawCircle(52, 184, 10, YELLOW);
    M5.Lcd.drawCircle(52, 184, 2, WHITE);
    M5.Lcd.drawFastVLine(52, 200, 20, YELLOW);
    M5.Lcd.drawFastVLine(52, 149, 20, YELLOW);
    M5.Lcd.drawFastHLine(68, 184, 20, YELLOW);
    M5.Lcd.drawFastHLine(17, 184, 20, YELLOW);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(50, 140);
    M5.Lcd.print("N");
    M5.Lcd.setCursor(11, 181);
    M5.Lcd.print("W");
    M5.Lcd.setCursor(89, 181);
    M5.Lcd.print("E");
  } else {

    x = (sin(relCourse * rad_fac) * 26);
    y = (cos(relCourse * rad_fac) * 26);
    q = sin(relCourse * rad_fac);
    r = cos(relCourse * rad_fac);
    int x1 = 53 + x;
    int y1 = 184 - y;
    int x2 = 53 - x;
    int y2 = 184 + y;
    int q1 = 53 + q;
    int r1 = 184 - r;
    int q2 = 53 - q;
    int r2 = 184 + r;

    double arrowAngle = relCourse + 200;
    x = (sin(arrowAngle * rad_fac) * 48);
    y = (cos(arrowAngle * rad_fac) * 48);
    q = (sin(arrowAngle * rad_fac) * 48);
    r = (cos(arrowAngle * rad_fac) * 48);
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;
    if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
      M5.Lcd.drawCircle(52, 184, 30, NAVY);
      M5.Lcd.drawCircle(52, 184, 20, 0x00AF);
      M5.Lcd.drawCircle(52, 184, 10, BLUE);
      M5.Lcd.drawCircle(52, 184, 2, CYAN);
      M5.Lcd.drawFastVLine(52, 200, 20, DARKCYAN);
      M5.Lcd.drawFastVLine(52, 149, 20, DARKCYAN);
      M5.Lcd.drawFastHLine(68, 184, 20, DARKCYAN);
      M5.Lcd.drawFastHLine(17, 184, 20, DARKCYAN);
    } else {
      M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
      M5.Lcd.drawLine(q1, r1, x2, y2, YELLOW);
    }
    arrowAngle = relCourse + 160;
    x = (sin(arrowAngle * rad_fac) * 48);
    y = (cos(arrowAngle * rad_fac) * 48);
    q = (sin(arrowAngle * rad_fac) * 48);
    r = (cos(arrowAngle * rad_fac) * 48);
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;
    if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
      ;
    } else {
      M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
      M5.Lcd.drawLine(q1, r1, x2, y2, YELLOW);
    }
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.setCursor(50, 140);
    M5.Lcd.print("N");
    M5.Lcd.setCursor(11, 181);
    M5.Lcd.print("W");
    M5.Lcd.setCursor(89, 181);
    M5.Lcd.print("E");
  }
  const char *satIcon = gps.location.isValid() ? "/SAT2.png" : "/SAT1.png";
  M5.Lcd.drawPngFile(SD, satIcon, 100, 98);

  if (gps.location.isValid()) {
    Serial.printf("Latitude: %.6f\nLongitude: %.6f\n", gps.location.lat(), gps.location.lng());
  } else {
    Serial.println("Kein GPS-Fix!");
  }

  //COORDINATES
  if (distanceToHome > 10) {
    M5.Lcd.setCursor(14, 140);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print(relCourse + 360, 0);
    M5.Lcd.println("\xF7  ");
    M5.Lcd.setTextSize(1);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.setCursor(66, 140);
    M5.Lcd.print(TinyGPSPlus::cardinal(relCourse + 360));
    M5.Lcd.print("  ");
  } else {
    M5.Lcd.fillRoundRect(13, 140, 32, 12, 1, BLACK);
    M5.Lcd.fillRoundRect(60, 140, 32, 12, 1, BLACK);
  }

  // Berechne CET-Offset
  int cetOffset = calculateCET(gps.date, gps.time);
  int cetHour = gps.time.hour() + cetOffset;

  if (cetHour >= 24) cetHour -= 24;  // Überlaufkorrektur
  if (cetHour < 0) cetHour += 24;    // Unterlaufkorrektur

  updateWeatherDisplay();

  int16_t raw_CO = ads.readADC_SingleEnded(0);   // Kanal A0 = CO
  int16_t raw_NH3 = ads.readADC_SingleEnded(1);  // Kanal A1 = NH3
  int16_t raw_NO2 = ads.readADC_SingleEnded(2);  // Kanal A2 = NO2
  int16_t raw_EMF = ads.readADC_SingleEnded(3);  // Kanal A3 = EMF

  // Umrechnung von Rohdaten (0–32767) in echte Werte
  float voltage_CO = raw_CO * 0.125 / 1000.0;  // Spannung in Volt umwandeln (125µV pro LSB)
  float voltage_NH3 = raw_NH3 * 0.125 / 1000.0;
  float voltage_NO2 = raw_NO2 * 0.125 / 1000.0;
  float voltage_EMF = raw_EMF * 0.125 / 1000.0;

  // Plausible ppm-Werte berechnen
  float CO = voltage_CO * 15.0;    // 10.0 Beispiel-Skalierung
  float NH3 = voltage_NH3 * 10.0;  // 5.0 Beispiel-Skalierung
  float NO2 = voltage_NO2 * 40.0;  // 40.0 Beispiel-Skalierung
  float EMF = voltage_EMF * 5.0;   // 5.0 Beispiel-Skalierung
  if (CO > 99) {
    CO = 99;
  }
  if (CO > 20) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, RED);
    M5.Lcd.setCursor(14, 79);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(CO);
    M5.Lcd.print(" ppm");
  } else if (CO > 10) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 79);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(CO);
    M5.Lcd.print(" ppm");
  } else {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 79);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(CO);
    M5.Lcd.print(" ppm");
  }
  if (NH3 > 99) {
    NH3 = 99;
  } else if (NH3 > 15) {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, RED);
    M5.Lcd.setCursor(14, 90);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(NH3);
    M5.Lcd.print(" ppm");
  } else if (NH3 > 5) {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 90);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(NH3);
    M5.Lcd.print(" ppm");
  } else {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 90);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(NH3);
    M5.Lcd.print(" ppm");
  }
  if (NO2 > 99) {
    NO2 = 99;
  } else if (NO2 > 5) {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, RED);
    M5.Lcd.setCursor(14, 101);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(NO2);
    M5.Lcd.print(" ppm");
  } else if (NO2 > 2) {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 101);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(NO2);
    M5.Lcd.print(" ppm");
  } else {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 101);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(NO2);
    M5.Lcd.print(" ppm");
  }
  if (EMF > 99) {
    EMF = 99;
  } else if (EMF > 40) {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, RED);
    M5.Lcd.setCursor(14, 112);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(EMF);
    M5.Lcd.print("uT");
  } else if (EMF > 30) {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 112);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(EMF);
    M5.Lcd.print(" uT");
  } else {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 112);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(EMF);
    M5.Lcd.print(" uT");
  }

  // Alarmprüfung
  checkForAlarms(CO, NH3, NO2, EMF, radiation);

  // BATTERY
  float batVoltage = M5.Axp.GetBatVoltage();
  float batPercentage = (batVoltage < 3.20) ? 0 : (batVoltage - 3.20) * 100 / (4.2 - 3.2);  // Normalisiert auf 100%
  batPercentage = constrain(batPercentage, 0, 100);                                         // Begrenzen auf 0 - 100%
  bool isCharging = M5.Axp.isACIN();

  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(240, 5);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.print(batVoltage, 2);
  M5.Lcd.println("V");

  // Bestimme Farbe basierend auf dem Ladezustand
  uint16_t batteryColor;
  if (batPercentage < 20) {
    batteryColor = RED;
  } else if (batPercentage < 50) {
    batteryColor = YELLOW;
  } else {
    batteryColor = GREEN;
  }

  // Batterieanzeige zeichnen
  M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);  // Hintergrund löschen
  M5.Lcd.drawRoundRect(275, 7, 35, 14, 3, GREENYELLOW);
  M5.Lcd.drawRoundRect(309, 11, 4, 6, 2, GREENYELLOW);
  //M5.Lcd.fillRoundRect(277, 9, (batPercentage / 3.2), 10, 2, batteryColor);

  M5.Lcd.setCursor(240, 14);
  M5.Lcd.setTextColor(batteryColor, BLACK);
  M5.Lcd.print(batPercentage, 0);
  M5.Lcd.print("% ");

  //LOAD BATTERY
  if (isCharging) {
    M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);
    M5.Lcd.fillRoundRect(277, 9, (batPercentage / 3.2), 10, 2, batteryColor);
    //M5.Lcd.fillRoundRect(277, 9, (batPercentage) / 3.2, 10, 1, GREEN);
    M5.Lcd.fillTriangle(291, 14, 288, 14, 295, 7, RED);
    M5.Lcd.fillTriangle(296, 13, 291, 13, 289, 20, RED);
  }

  //GPS

  // LATTITUDE
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(106, 138);
  M5.Lcd.print("LATT:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(gps.location.lat() < 0 ? "S" : "N");
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("\xF7");
  M5.Lcd.setTextSize(2);

  if (gps.location.isValid() && gps.location.lat() != 0.0) {
    static double smoothLat = 0.0;
    // const float alpha = 0.2;  // Glättungsfaktor für bessere Anpassung
    // smoothLat = (alpha * gps.location.lat()) + ((1 - alpha) * smoothLat);
    smoothLat = gps.location.lat();  // Direkte GPS-Werte verwenden
    char latBuffer[12];
    snprintf(latBuffer, sizeof(latBuffer), "%.6f", smoothLat);
    M5.Lcd.setCursor(186, 138);
    M5.Lcd.print(latBuffer);
    M5.Lcd.fillRect(293, 138, 23, 14, BLACK);
    M5.Lcd.fillRect(315, 138, 4, 14, BLACK);
    M5.Lcd.drawFastVLine(315, 138, 16, CYAN);
  } else {
    M5.Lcd.setCursor(200, 138);
    M5.Lcd.print("---");
  }
  // LONGITUDE
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(106, 157);
  M5.Lcd.print("LONG:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(gps.location.lng() < 0 ? "W" : "E");
  M5.Lcd.setTextSize(1);
  M5.Lcd.print("\xF7");
  M5.Lcd.setTextSize(2);

  if (gps.location.isValid() && gps.location.lng() != 0.0) {
    static double smoothLon = 0.0;
    // const float alpha = 0.2;
    // smoothLon = (alpha * gps.location.lng()) + ((1 - alpha) * smoothLon);
    smoothLon = gps.location.lng();  // Richtigen Wert für Longitude setzen
    char lngBuffer[12];
    snprintf(lngBuffer, sizeof(lngBuffer), "%.6f", smoothLon);
    M5.Lcd.setCursor(198, 157);
    M5.Lcd.print(lngBuffer);
    M5.Lcd.fillRect(293, 157, 18, 14, BLACK);
  } else {
    M5.Lcd.setCursor(200, 157);
    M5.Lcd.print("---");
  }
  // ALTITUDE
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(106, 176);
  M5.Lcd.print("ALTI:");
  M5.Lcd.setTextColor(CYAN, BLACK);

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");
  } else {
    M5.Lcd.print(gps.altitude.meters(), 2);
    M5.Lcd.print("m    ");
  }
  // SPEED
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(106, 196);
  M5.Lcd.print("SPED:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    M5.Lcd.print("---");
  } else {
    M5.Lcd.print(gps.speed.kmph(), 0);
    M5.Lcd.print("km/h    ");
  }
  // HOME DISTANCE
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(106, 215);
  M5.Lcd.print("HOME:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  if ((gps.location.lat() == 0) && (gps.location.lng() == 0)) {
    M5.Lcd.print("---");
  } else if (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) < 1000) {
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon), 0);
    M5.Lcd.print("m ");
    M5.Lcd.fillRect(200, 215, 110, 14, BLACK);  // Löscht mögliche Artefakte
  } else {
    M5.Lcd.print(TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000, 2);
    M5.Lcd.print("km ");
  }
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(16, 222);
  // how long to the destination
  float distance_km = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000;
  float speed_kmh = 3;                              // Geschwindigkeit in km/h
  float hours_per_day = 8;                          // Anzahl der Stunden, die pro Tag gelaufen wird
  float speed_per_day = speed_kmh * hours_per_day;  // Tagesdistanz

  float travel_time_hours = distance_km / speed_kmh;     // Zeit in Stunden
  float travel_time_days = distance_km / speed_per_day;  // Zeit in Tagen
  float travel_time_minutes = travel_time_hours * 60;

  M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);

  if (travel_time_days > 1.5) {
    M5.Lcd.print(travel_time_days, 1);
    M5.Lcd.print(" DAYS");
  } else if (travel_time_days >= 1) {
    M5.Lcd.print(travel_time_days, 1);
    M5.Lcd.print(" DAY");
  } else if (travel_time_hours >= 1.5) {
    M5.Lcd.print(travel_time_hours, 1);
    M5.Lcd.print(" HOURS");
  } else if (travel_time_hours >= 1) {
    M5.Lcd.print(travel_time_hours, 1);
    M5.Lcd.print(" HOUR");
  } else if (travel_time_minutes > 1.5) {
    M5.Lcd.print(travel_time_minutes, 0);
    M5.Lcd.print(" MINUTES");
  } else if (travel_time_minutes >= 0.5) {
    M5.Lcd.print(travel_time_minutes, 0);
    M5.Lcd.print(" MINUTE");
  } else {
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.print(" DESTINATION");
  }

  float currentLat = gps.location.lat();
  float currentLon = gps.location.lng();

  static String lastCity = "";     // Speichert vorherige Stadt
  static String lastCountry = "";  // Speichert vorheriges Land

  if (currentLat != 0.0 && currentLon != 0.0) {  // Nur wenn GPS gültige Werte liefert
    String city, country;
    findNearestCity(currentLat, currentLon, city, country);  // Ruft Stadt + Land ab
    // Überprüfe, ob sich der Name geändert hat
    if (city != lastCity || country != lastCountry) {
      M5.Lcd.fillRect(10, 127, 297, 8, BLACK);
      M5.Lcd.drawFastHLine(21, 127, 286, CYAN);
      M5.Lcd.drawFastHLine(21, 133, 286, CYAN);
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.setCursor(10, 127);
      M5.Lcd.setTextSize(1);
      M5.Lcd.print("> ");
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.print(city);  // Stadt anzeigen
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.print(" - ");
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.print(country);  // Land anzeigen
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.print(" <");
      lastCity = city;
      lastCountry = country;
    }
  }
  Serial.print(F("Sats="));
  Serial.print(gps.satellites.value());
  Serial.print(F(" Nums="));
  for (int i = 0; i < MAX_SATELLITES; ++i)
    if (sats[i].active) {
      Serial.print(i + 1);
      Serial.print(F(" "));
    }
}
//LOOP END
void displaySavedLocation() {
  M5.Lcd.fillRect(21, 233, 100, 10, BLACK);  // Löscht alten Text
  M5.Lcd.setCursor(21, 233);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print(savedCity);  // Zeigt gespeicherte Stadt an
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(" <");
}
void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (MySerial.available()) {
      gps.encode(MySerial.read());
    }
    printDateTime(gps.date, gps.time);  // GPS-Zeit aktualisieren
  } while (millis() - start < ms);
}
static void printFloat(float val, bool valid, int len, int prec) {
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  } else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
}
static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
}
