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

float volumeFactor = 0.2;       // Startlautstärke
static int brightnessStep = 5;  // Start bei max (0 = AUS, 6 = max)
unsigned long lastVolumeAdjustTime = 0;
const unsigned long volumeAdjustInterval = 200;  // 200ms = 5x pro Sekunde
File wavFile;
uint8_t buffer[1024];
// Hilfsfunktion zum WAV abspielen
bool playSound(const char *filename) {
  String fullpath = "/sounds/";
  fullpath += filename;

  wavFile = SD.open(fullpath.c_str());
  if (!wavFile) {
    return false;
  }

  struct __attribute__((packed)) wav_header_t {
    char RIFF[4];
    uint32_t chunk_size;
    char WAVEfmt[8];
    uint32_t fmt_chunk_size;
    uint16_t audiofmt;
    uint16_t channel;
    uint32_t sample_rate;
    uint32_t byte_per_sec;
    uint16_t block_size;
    uint16_t bit_per_sample;
  };
  struct __attribute__((packed)) sub_chunk_t {
    char identifier[4];
    uint32_t chunk_size;
  };

  // Header prüfen
  wav_header_t wheader;
  if (wavFile.read((uint8_t *)&wheader, sizeof(wheader)) != sizeof(wheader)) return false;
  if (memcmp(wheader.RIFF, "RIFF", 4) || memcmp(wheader.WAVEfmt, "WAVEfmt ", 8) || wheader.audiofmt != 1 || wheader.sample_rate != 44100 || wheader.bit_per_sample != 16 || wheader.channel != 1) {

    return false;
  }

  sub_chunk_t c;
  while (true) {
    if (wavFile.read((uint8_t *)&c, sizeof(c)) != sizeof(c)) return false;
    if (memcmp(c.identifier, "data", 4) == 0) break;
    wavFile.seek(wavFile.position() + c.chunk_size);
  }

  // Wiedergabe starten
  while (wavFile.available()) {
    int len = wavFile.read(buffer, sizeof(buffer));

    // Lautstärke anpassen (16-Bit Samples skalieren)
    for (int i = 0; i < len; i += 2) {
      int16_t *sample = (int16_t *)&buffer[i];
      *sample = *sample * volumeFactor;
    }

    M5.Spk.PlaySound(buffer, len);
  }
  wavFile.close();
  return true;
}
bool soundEnabled = true;
bool hadGpsFix = false;   // Merkt sich, ob vorher ein Fix vorhanden war
bool hasArrived = false;  // global definieren

void updateVolumeBar() {
  int level = round(volumeFactor * 6);  // 0 bis 5 (6 Stufen)
  int barHeight = level * 34;           // 33 Pixel je Stufe

  // Nur den inneren Bereich löschen – Rahmen bleibt!
  M5.Lcd.fillRect(317, 24, 1, 204, BLACK);

  // Balken nur zeichnen, wenn Lautstärke > 0
  if (level > 0) {
    M5.Lcd.fillRect(317, 230 - barHeight, 1, barHeight, GREEN);
  }
}
unsigned long lastGpsPingTime = 0;
const unsigned long gpsPingInterval = 30000;  // 30 Sekunden
void disablePowerLed() {
  M5.Axp.SetLed(false);  // dauerhaft grün-LED aus
}

// Seedbanks definieren
struct Location {
  const char *name;
  float lat;
  float lon;
};

Location seedBanks[] = {
  { "CGN WAGENINGEN NETHERLANDS", 51.9863, 5.6680 },
  { "IPK GATERSLEBEN GERMANY", 51.8247, 11.2792 },
  { "NORDGEN PLANTS ALNARP SWEDEN", 55.6601, 13.0834 },
  { "INRAE PARIS FRANCE", 48.8611, 2.3062 },
  { "MILLENIUMSEED BANK LONDON ENGLAND", 51.0688, -0.0899 },
  { "GERMOPLASMA MADRID SPAIN", 40.4425, -3.7291 },
  { "GLOBAL SEED VAULT SVALBARD NORWAY", 78.1409, 15.2929 }
};

const int numLocations = sizeof(seedBanks) / sizeof(seedBanks[0]);

moonPhase moon;
float lastPressure = -1;
float lastPressures[10] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };  // Speicher für die letzten 10 Messungen
float lastValidDoseRate = 0.0;                                         // Speichert den letzten gültigen Wert der Dosisrate

int pressureIndex = 0;  // Aktuelle Position im Array
String lastWeatherIcon = "";
String lastArrowIcon = "";

bool selectingCountry = true;
bool selectingCity = false;

BME280_I2C bme(0x76);  //i2C PA_SDA 32,PA_SCL 33
TinyGPSPlus gps;
HardwareSerial MySerial(1);  // Verwende den zweiten Hardware-Serial-Port
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

unsigned long lastGPSTimeUpdate = 0;    // Letzte Aktualisierung der Uhrzeit
const int gpsUpdateInterval = 100;      // GPS-Anzeige nur alle 100 ms aktualisieren
const float alpha = 0.2;                // Glättungsfaktor für GPS (zwischen 0 und 1)
const float CO_THRESHOLD = 30.0;        // CO: gefährlich ab 30 ppm
const float NH3_THRESHOLD = 25.0;       // NH3: gefährlich ab 25 ppm
const float NO2_THRESHOLD = 10.0;       // NO2: gefährlich ab 10 ppm
const float EMF_THRESHOLD = 20.0;       // EMF: gefährlich ab 20
const float RADIATION_THRESHOLD = 5.0;  // Strahlung: gefährlich ab 5.0 µSv/h

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
    delay(500);
  } else {
    Serial.println("Fehler: radiation.png nicht gefunden!");
  }
}

float doseRate = 0.0;     // Dosis in µSv/h
float averageDose = 0.0;  // Durchschnittliche Dosis

const float calibrationFactor = 153.0;  //153.0; für J613/J614  // Kalibrierung: CPM pro µSv/h
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
static bool wasAlarmTriggered = false;  // Merkt sich, ob vorher Alarm war

// Vibration und LED-Muster auslösen
void triggerVibrationPattern(const int pattern[], int len) {
  for (int i = 0; i < len; i++) {
    if (i % 2 == 0) {
      M5.Axp.SetVibration(true);
      M5.Axp.SetLed(true);
    } else {
      M5.Axp.SetVibration(false);
      M5.Axp.SetLed(false);
    }
    delay(pattern[i]);
  }
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
  alarmTriggered = false;

  // CO-Warnung
  if (CO > CO_THRESHOLD) {
    playSound("alarm11.wav");
    showAlarm("/hazard.png", 13, 79, " !!!DANGER!!!");
    int patternCO[] = { 400, 100, 200, 100, 400 };
    triggerVibrationPattern(patternCO, sizeof(patternCO) / sizeof(patternCO[0]));
    alarmTriggered = true;
  }

  // NH3-Warnung
  if (NH3 > NH3_THRESHOLD) {
    playSound("alarm11.wav");
    showAlarm("/hazard.png", 13, 90, " !!!DANGER!!!");
    int patternNH3[] = { 400, 100, 400, 100, 200 };
    triggerVibrationPattern(patternNH3, sizeof(patternNH3) / sizeof(patternNH3[0]));
    alarmTriggered = true;
  }

  // NO2-Warnung
  if (NO2 > NO2_THRESHOLD) {
    playSound("alarm11.wav");
    showAlarm("/hazard.png", 13, 101, " !!!DANGER!!!");
    int patternNO2[] = { 200, 100, 200, 100, 400 };
    triggerVibrationPattern(patternNO2, sizeof(patternNO2) / sizeof(patternNO2[0]));
    alarmTriggered = true;
  }

  // EMF-Warnung
  if (EMF > EMF_THRESHOLD) {
    playSound("beep19.wav");
    showAlarm("/EMF.png", 13, 112, " !!!DANGER!!!");
    int patternEMF[] = { 200, 100, 200, 100, 100 };
    triggerVibrationPattern(patternEMF, sizeof(patternEMF) / sizeof(patternEMF[0]));
    alarmTriggered = true;
  }

  // Strahlung
  if (radiation > RADIATION_THRESHOLD && !alarmTriggered) {
    playSound("alarm07.wav");
    M5.Lcd.drawPngFile(SD, "/radiation3.png", 237, 31);
    int patternRadiation[] = { 200, 100, 400, 100, 200 };
    triggerVibrationPattern(patternRadiation, sizeof(patternRadiation) / sizeof(patternRadiation[0]));
    alarmTriggered = true;
  }

  // Zurücksetzen der Fläche nur beim Übergang von Alarm → Normal
  if (wasAlarmTriggered && !alarmTriggered) {
    M5.Lcd.fillRoundRect(9, 29, 87, 94, 4, BLACK);  // Artefakte entfernen
  }

  wasAlarmTriggered = alarmTriggered;  // Letzten Zustand merken
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
  pulseCount++;           // Impuls zählen
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

int activeSatellites = 0;
double currentLat = gps.location.lat();
double currentLon = gps.location.lng();
struct TargetLocation {
  float lat;
  float lon;
  String city;
  String country;
};

std::vector<TargetLocation> targetList;
const int maxTargets = 7;
unsigned long buttonAPressTime = 0;
bool buttonAHeld = false;

// SD speichern
void saveTargetList() {
  SD.remove("/last_targets.txt");  // ✅ Datei löschen, um sauber neu zu schreiben
  File file = SD.open("/last_targets.txt", FILE_WRITE);
  if (!file) {
    Serial.println("❌ Fehler beim Öffnen von last_targets.txt zum Schreiben.");
    return;
  }

  for (const auto &target : targetList) {
    file.printf("%.6f,%.6f,%s,%s\n", target.lat, target.lon, target.city.c_str(), target.country.c_str());
  }

  file.close();
  Serial.println("✅ Letzte Ziele gespeichert.");
}

// SD laden
void loadTargetList() {
  targetList.clear();

  File file = SD.open("/last_targets.txt");
  if (!file) {
    Serial.println("📂 Keine gespeicherten Ziele gefunden.");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      float lat, lon;
      char city[32], country[32];
      if (sscanf(line.c_str(), "%f,%f,%31[^,],%31s", &lat, &lon, city, country) == 4) {
        TargetLocation t = { lat, lon, String(city), String(country) };
        targetList.push_back(t);
      }
    }
  }

  file.close();
  Serial.printf("📌 %d gespeicherte Ziele geladen.\n", targetList.size());
}
// 📌 Hilfsfunktion für GPS-Validität
bool isGPSValid() {
  return gps.location.isValid() && gps.location.age() < 5000;
}

/////////////////////////////////////////////////////////////////////////

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
  M5.Spk.begin();
  ss.begin(GPSBaud);
  disablePowerLed();
  // I2C Initialisierung
  if (!ads.begin()) {
    Serial.println("Fehler: ADS1115 nicht gefunden!");
    while (1)
      ;
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

  loadTargetList();  // ⬅️ Direkt beim Start aus SD laden

  M5.Axp.SetLcdVoltage(3167);
  bme.setTempCal(-1);

  // Koordinaten aus Datei lesen
  File myFile = SD.open("/home_coordinates.txt");
  if (myFile) {
    String coordinates = myFile.readStringUntil('\n');  // Lese gespeicherte Werte
    myFile.close();

    int commaIndex = coordinates.indexOf(',');
    if (commaIndex > 0) {
      homeLat = coordinates.substring(0, commaIndex).toFloat();
      homeLon = coordinates.substring(commaIndex + 1).toFloat();
      updateNearestCity(homeLat, homeLon);  // Stelle sicher, dass der Zielort hier gesetzt wird
    }
  } else {
    Serial.println("Keine gespeicherten Koordinaten gefunden.");
  }

  playSound("beep27.wav");

  bool pngDrawn = false;  // set this variable to 'false' to ensure that the PNG has not yet been drawn

  while (!pngDrawn) {  // Loop that keeps running until the display is tapped
    // 🔁 Wird aufgerufen, solange noch kein Touch erfolgt ist
    if (M5.Touch.ispressed()) {
      pngDrawn = true;  // ⬅️ Touch erkannt, PNG nur einmal zeichnen
    } else {
      M5.update();

      // 🔽 Hintergrundbild
      M5.Lcd.drawPngFile(SD, "/radar1.png", 0, 40);

      // 🔽 Lade die Liste der Zielorte (oder die zuletzt gespeicherten Koordinaten)
      File myFile = SD.open("/home_coordinates.txt", FILE_READ);
      if (myFile) {
        String line = myFile.readStringUntil('\n');
        int commaIndex = line.indexOf(",");
        if (commaIndex != -1) {
          homeLat = line.substring(0, commaIndex).toDouble();
          homeLon = line.substring(commaIndex + 1).toDouble();
          // Stelle sicher, dass der Zielort hier gesetzt wird
          updateNearestCity(homeLat, homeLon);  // Update der Stadt und des Landes mit den Koordinaten
        } else {
          Serial.println("❌ Fehler: Dateiinhalt ungültig!");
        }
        myFile.close();
      } else {
        Serial.println("📂 Datei '/home_coordinates.txt' nicht gefunden!");
      }

      // 🧠 Stadt & Land ermitteln – hier sicherstellen, dass der Zielort verwendet wird
      findNearestCity(homeLat, homeLon, nearestCity, nearestCountry);

      // 🎨 Anzeige der Koordinaten und Zielorte
      M5.Lcd.drawPngFile(SD, "/GARPAX.png", 250, 0);
      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setTextColor(DARKCYAN, BLACK);
      M5.Lcd.setCursor(10, 40);
      M5.Lcd.print("MULITISENS-GARPAX");

      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.setCursor(100, 80);
      M5.Lcd.print("SAVED COORDINATES");

      // **Anzeige des gespeicherten Zielorts**
      M5.Lcd.setFreeFont(&SFChromeFendersCondensed16pt7b);
      M5.Lcd.setCursor(133, 164);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.print(nearestCity);

      M5.Lcd.setCursor(160, 130);
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.print(nearestCountry);

      // Zeige Koordinaten an
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
  //M5.Lcd.print("> HOME POS <");
  M5.Lcd.setCursor(234, 233);
  M5.Lcd.print("> SELECT <");
  updateVolumeBar();  // Lautstärkebalken sofort anzeigen beim Start
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
  static bool graphNeedsUpdate = false;

  sumDoseRate += doseRate;
  countSamples++;

  if (countSamples >= 7.23) {
    int newY = 80 - ((sumDoseRate / countSamples) * 3000);
    newY = constrain(newY, 40, 80);
    rateGraphBuffer[rateGraphIndex] = newY;
    rateGraphIndex = (rateGraphIndex + 1) % RATE_GRAPH_WIDTH;

    sumDoseRate = 0;
    countSamples = 0;
    graphNeedsUpdate = true;
  }

  if (!graphNeedsUpdate) return;  // Kein Update → nicht zeichnen

  graphNeedsUpdate = false;

  // Zeichne nur, wenn nötig:
  M5.Lcd.fillRect(225, 31, 86, 60, BLACK);  // löschen

  for (int y = 40; y <= 120; y += 10) {
    M5.Lcd.drawFastHLine(225, y, 85, 0x0320);
  }

  for (int x = 225; x <= 309; x += 14) {
    M5.Lcd.drawFastVLine(x, 40, 40, 0x0320);
  }

  for (int i = 0; i < RATE_GRAPH_WIDTH - 1; i++) {
    int x1 = 226 + i;
    int y1 = rateGraphBuffer[(rateGraphIndex + i) % RATE_GRAPH_WIDTH];
    int x2 = 226 + (i + 1);
    int y2 = rateGraphBuffer[(rateGraphIndex + i + 1) % RATE_GRAPH_WIDTH];

    uint16_t color = getGradientColor(y1, 40, 80);
    M5.Lcd.drawLine(x1, y1, x2, y2, color);
  }
}

void drawAverageGraph(float avgDose) {
  static bool avgGraphNeedsUpdate = false;

  int scaledHeight = map(avgDose, 0, 10, 0, 68);
  if (scaledHeight < 1 && avgDose > 0) scaledHeight = 1;
  avgGraphBuffer[avgGraphIndex] = constrain(scaledHeight, 0, 68);
  avgGraphIndex = (avgGraphIndex + 1) % AVG_GRAPH_WIDTH;

  avgGraphNeedsUpdate = true;

  if (!avgGraphNeedsUpdate) return;
  avgGraphNeedsUpdate = false;

  M5.Lcd.fillRect(226, 90, 84, 33, BLACK);  // Bereich löschen

  for (int i = 0; i < AVG_GRAPH_WIDTH; i++) {
    int x = 226 + i * 5;
    int height = avgGraphBuffer[(avgGraphIndex + i) % AVG_GRAPH_WIDTH];
    int y = 120 - height;

    float dose = height / 10.0;
    uint16_t color =
      (dose < 1.0) ? GREEN : (dose < 2.0) ? YELLOW
                           : (dose < 5.0) ? ORANGE
                                          : RED;

    M5.Lcd.fillRect(x, y, 4, height, color);
  }
}
void updateDoseRate() {
  unsigned long count = pulseCount;
  pulseCount = 0;

  float doseRate = count / calibrationFactor;

  if (doseRate <= 0.01) {
    doseRate = lastValidDoseRate;  // Verwende den letzten gültigen Wert, wenn der Wert zu niedrig ist
  } else {
    lastValidDoseRate = doseRate;  // Speichere den neuen gültigen Wert
  }

  // Weitere Berechnungen und Update für Diagramme
  static float sumDose = 0;
  static int countSamples = 0;
  sumDose += doseRate;
  countSamples++;

  if (countSamples >= 225) {  // Alle 3,75 Minuten aktualisieren
    float avgDose = sumDose / countSamples;
    drawAverageGraph(avgDose);
    sumDose = 0;
    countSamples = 0;
  }

  drawRateGraph(doseRate);  // Zeichne den aktuellen Graphen
  if (countSamples > 0) {
    displayValues(doseRate, sumDose / countSamples);
  } else {
    displayValues(doseRate, 0);
  }
}

void displayValues(float doseRate, float averageDose) {
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(227, 31);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("DR:");

  // Wenn der Wert zu niedrig ist, den letzten gültigen Wert verwenden
  if (doseRate <= 0.01) {
    doseRate = lastValidDoseRate;  // Letzten gültigen Wert anzeigen
  } else {
    lastValidDoseRate = doseRate;  // Speichere den neuen gültigen Wert
  }

  if (doseRate > 99) doseRate = 99;
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.printf("%.2f uSv/h", doseRate);

  M5.Lcd.setCursor(227, 82);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("AD:");
  if (averageDose > 99) averageDose = 99;

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
      // Uhrzeit vorbereiten
      sprintf(sz, "%02d:%02d:%02d", hourCET, t.minute(), t.second());
      // Textfarbe nach Gültigkeit
      uint16_t timeColor = (d.isValid() && t.isValid()) ? CYAN : RED;
      // Zeitbereich löschen & anzeigen
      M5.Lcd.fillRect(232, 6, 8, 14, BLACK);
      M5.Lcd.setTextColor(timeColor, BLACK);
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

float getAveragePressure() {
  float sum = 0;
  int count = 0;

  for (int i = 0; i < 10; i++) {
    if (lastPressures[i] > 0) {
      sum += lastPressures[i];
      count++;
    }
  }
  return (count > 0) ? sum / count : -1;
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
  // Durchschnitt berechnen
  float avgPressure = getAveragePressure();
  float pressure = bme.getPressure_HP() / 100;
  lastPressures[pressureIndex] = pressure;
  pressureIndex = (pressureIndex + 1) % 10;  // Immer zyklisch zwischen 0-4

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
    M5.Lcd.print(">> STORM <<");  // Schwerer Sturm
    weatherIcon = "/weather/07.png";
  } else if (pressure <= 985) {
    M5.Lcd.print(">> WIND <<");  // Sehr windig / Stürmisch
    weatherIcon = "/weather/06.png";
  } else if (pressure <= 1000) {
    if (bme.getTemperature_C() > 1) {
      M5.Lcd.print(" >> RAIN <<");  // Regen
      weatherIcon = "/weather/05.png";
    } else {
      M5.Lcd.print(" >> SNOW <<");  // Schnee
      weatherIcon = "/weather/04.png";
    }
  } else if (pressure <= 1010) {
    M5.Lcd.print("> CLOUDY <");  // Bewölkt
    weatherIcon = "/weather/03.png";
  } else if (pressure <= 1020) {
    M5.Lcd.print("> OVERCAST <");  // Bedeckt
    weatherIcon = "/weather/02.png";
  } else if (pressure <= 1030) {
    M5.Lcd.print(">> CLEAR <<");  // Klarer Himmel
    weatherIcon = "/weather/01.png";
  } else {
    M5.Lcd.print(">> SUNNY <<");  // Sonnig, hoher Druck
    weatherIcon = "/weather/00.png";
  }

  // Druck-Trend bestimmen
  if (lastPressures[(pressureIndex + 9) % 10] > 0) {  // Vergleich mit ältestem Wert
    float pressureDiff = avgPressure - lastPressures[(pressureIndex + 9) % 10];

    if (abs(pressureDiff) >= 0.2) {  // Trend-Änderung nur ab 0.2 hPa
      arrowIcon = (pressureDiff > 0) ? "/icons/up.png" : "/icons/down.png";
    }
  }

  // Icons aktualisieren
  updateWeatherIcon(weatherIcon);
  updateArrowIcon(arrowIcon);
}

////////////////////////////////////////////////////////////////

void loop() {
  M5.update();  // Touch-Events aktualisieren

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

      // Sicherstellen, dass GPS-Zeit gültig ist
      if (timeinfo.tm_year >= 120) {  // 120 = Jahr 2020 (weil tm_year ab 1900 zählt)

        // Mondphase berechnen und anzeigen
        time_t gps_time = mktime(&timeinfo);            // UNIX-Zeit berechnen
        moonData_t moonData = moon.getPhase(gps_time);  // Mondphase berechnen
        int phaseIndex = (int)(moonData.angle / 360.0 * 30) % 30;

        // Mondphase-Dateiname erzeugen
        char filename[32];
        snprintf(filename, sizeof(filename), "%s%02d.png", MOON_PHASES_PATH, phaseIndex);

        // Alte Anzeige löschen und neue anzeigen
        M5.Lcd.fillRoundRect(199, 25, 24, 24, 6, BLACK);
        M5.Lcd.drawPngFile(SD, filename, 200, 29);

        // Zeit des letzten Updates setzen
        lastUpdateTime = currentMillis;

      } else {
        Serial.println("Fehler: Ungültige GPS-Zeit, Mondphase nicht aktualisiert.");
      }
    }
  }

  // === VOLUME ===
  static int volumeStep = 1;                 // Startstufe (0 = lautlos, 5 = max)
  volumeStep = constrain(volumeStep, 0, 6);  // Sicherheit
  volumeFactor = volumeStep / 6.0;

  // === VOLUMEBALKEN ZEICHNEN ===
  const int barHeight = 202;
  const int barWidth = 1;
  const int barX = 317, barY = 230;  // Balken endet bei Y=120
  const int steps = 6;
  const uint16_t barColor = WHITE;

  // Hintergrund löschen
  M5.Lcd.fillRect(barX, barY - barHeight, barWidth, barHeight, BLACK);

  // Gefüllter Balkenanteil
  int fillHeight = (volumeStep * barHeight) / steps;
  M5.Lcd.fillRect(barX, barY - fillHeight, barWidth, fillHeight, barColor);

  // Trennlinien zwischen den 6 Abschnitten (5 Linien)
  for (int i = 1; i < steps; ++i) {
    int y = barY - (i * barHeight / steps);
    M5.Lcd.drawFastHLine(barX, y, barWidth, BLACK);
  }

  if (M5.Touch.ispressed()) {
    TouchPoint_t touchPoint = M5.Touch.getPressPoint();

    // 🔊 Lautstärke erhöhen (oben rechts)
    if (touchPoint.x >= 220 && touchPoint.x <= 320 && touchPoint.y >= 0 && touchPoint.y <= 100) {
      if (volumeStep < 6) {
        volumeStep++;
        volumeFactor = volumeStep / 6.0;

        Serial.printf("🔊 Lautstärke hoch: %.2f\n", volumeFactor);
      }
    }

    // 🔉 Lautstärke verringern (unten rechts)
    if (touchPoint.x >= 220 && touchPoint.x <= 320 && touchPoint.y >= 140 && touchPoint.y <= 240) {
      if (volumeStep > 0) {
        volumeStep--;
        volumeFactor = volumeStep / 6.0;

        Serial.printf("🔉 Lautstärke runter: %.2f\n", volumeFactor);
      }
    }

    brightnessStep = constrain(brightnessStep, 0, 6);
    int brightnessVoltage = map(brightnessStep, 0, 6, 2500, 3300);
    M5.Axp.SetLcdVoltage(brightnessVoltage);

    // === TOUCH: HELLIGKEIT STEUERUNG ===
    if (M5.Touch.ispressed()) {
      TouchPoint_t touchPoint = M5.Touch.getPressPoint();

      // ☀️ Helligkeit erhöhen (oben links)
      if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 0 && touchPoint.y <= 100) {
        if (brightnessStep < 6) {
          brightnessStep++;
          brightnessVoltage = map(brightnessStep, 0, 6, 2500, 3300);
          M5.Axp.SetLcdVoltage(brightnessVoltage);
        }
      }

      // 🌑 Helligkeit verringern (unten links)
      if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 140 && touchPoint.y <= 240) {
        if (brightnessStep > 0) {
          brightnessStep--;
          brightnessVoltage = map(brightnessStep, 0, 6, 2500, 3300);
          M5.Axp.SetLcdVoltage(brightnessVoltage);
        }
      }
    }
  }

  // === HELLIGKEITSBALKEN ZEICHNEN ===
  const int brightBarHeight = 202;
  const int brightBarWidth = 1;
  const int brightBarX = 2, brightBarY = 230;  // Links unten
  const int brightSteps = 6;
  const uint16_t brightBarColor = WHITE;

  M5.Lcd.fillRect(brightBarX, brightBarY - brightBarHeight, brightBarWidth, brightBarHeight, BLACK);

  int brightFillHeight = (brightnessStep * brightBarHeight) / brightSteps;
  M5.Lcd.fillRect(brightBarX, brightBarY - brightFillHeight, brightBarWidth, brightFillHeight, brightBarColor);

  for (int i = 1; i < brightSteps; ++i) {
    int y = brightBarY - (i * brightBarHeight / brightSteps);
    M5.Lcd.drawFastHLine(brightBarX, y, brightBarWidth, BLACK);
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

  // Taste A gedrückt halten oder kurz drücken
  if (M5.BtnA.isPressed()) {
    if (!buttonAHeld) {
      buttonAPressTime = millis();
      buttonAHeld = true;
    } else if (millis() - buttonAPressTime > 2000) {
      // 🔥 LANGES Drücken → Menü anzeigen
      showTargetSelectionMenu();
      buttonAHeld = false;
    }
  } else if (buttonAHeld && M5.BtnA.wasReleased()) {


    // 🟢 KURZES Drücken → Position speichern
    if (millis() - buttonAPressTime <= 2000) {

      unsigned long waitStart = millis();
      bool gotNewFix = false;

      M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(108, 137);
      M5.Lcd.print("WAITING FOR GPS");
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(108, 165);
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.print("Looking for position...");
      playSound("beep31.wav");
      while (millis() - waitStart < 3000) {
        while (Serial1.available()) {
          gps.encode(Serial1.read());
        }

        if (gps.location.isValid() && gps.location.age() < 2000) {
          gotNewFix = true;
          Serial.println("✅ GPS fix accepted.");
          break;
        } else {
          Serial.printf("⏳ GPS age: %lu ms\n", gps.location.age());
        }
        delay(100);
      }

      if (gotNewFix) {
        homeLat = gps.location.lat();
        homeLon = gps.location.lng();

        updateNearestCity(homeLat, homeLon);  // 🌍 Stadt/Land ermitteln

        // 🛡️ Stadt & Land in lokale Kopie sichern, bevor irgendwas sie ändert
        String currentCity = nearestCity;
        String currentCountry = nearestCountry;

        TargetLocation newTarget = {
          homeLat,
          homeLon,
          currentCity,
          currentCountry
        };

        targetList.insert(targetList.begin(), newTarget);
        if (targetList.size() > maxTargets) targetList.pop_back();
        saveTargetList();

        // 💾 Datei für Startbildschirm aktualisieren
        File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
        if (myFile) {
          myFile.print(homeLat, 6);
          myFile.print(",");
          myFile.println(homeLon, 6);
          myFile.close();

          // ❌ Kein zweiter updateNearestCity-Aufruf hier!
          displaySavedLocation();
        }

        // 🧾 Debug-Ausgabe
        Serial.printf("📝 Saved: %f,%f → %s, %s\n", homeLat, homeLon, currentCity.c_str(), currentCountry.c_str());

        // ✅ Feedback anzeigen
        M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setCursor(108, 137);
        M5.Lcd.print("POSITION SAVED");
        M5.Lcd.fillRect(64, 234, 170, 5, BLACK);
        M5.Lcd.drawFastHLine(64, 233, 170, CYAN);
        M5.Lcd.drawFastHLine(64, 239, 170, CYAN);
        M5.Lcd.setCursor(108, 157);
        M5.Lcd.setTextColor(CYAN, BLACK);
        M5.Lcd.setTextSize(1);
        displaySavedLocation();
        playSound("beep12.wav");
        delay(2000);

      } else {
        // ❌ Kein aktuelles Signal → Fehleranzeige
        M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.setCursor(108, 137);
        M5.Lcd.print("STATUS NOT SAVED");

        M5.Lcd.setTextColor(RED, BLACK);
        M5.Lcd.setCursor(108, 157);
        M5.Lcd.print("NO GPS FIX");

        M5.Lcd.setTextSize(1);
        M5.Lcd.setCursor(18, 233);
        M5.Lcd.setTextColor(YELLOW, BLACK);
        M5.Lcd.print("  N/A  ");
        M5.Lcd.setTextColor(CYAN, BLACK);
        M5.Lcd.print("<");
        M5.Lcd.fillRect(64, 233, 170, 9, BLACK);
        M5.Lcd.drawFastHLine(64, 233, 170, CYAN);
        M5.Lcd.drawFastHLine(64, 239, 170, CYAN);
        M5.Lcd.setCursor(141, 233);
        M5.Lcd.print("> HOME POS <");
        M5.Lcd.setCursor(236, 233);
        M5.Lcd.print("> SELECT   ");
        M5.Lcd.setCursor(300, 233);
        M5.Lcd.print("<");

        delay(3000);
      }

      // 🧼 Anzeige zurücksetzen
      M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    }

    buttonAHeld = false;
  }

  if (M5.BtnB.wasPressed()) {
    playSound("beep10.wav");
    M5.Lcd.fillRect(16, 233, 284, 9, BLACK);
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.drawFastHLine(21, 233, 270, CYAN);
    M5.Lcd.drawFastHLine(21, 239, 270, CYAN);
    M5.Lcd.setTextSize(1);
    displaySavedLocation();  // Anzeige aktualisieren
    M5.Lcd.print("> HOME POS <");
    M5.Lcd.setCursor(236, 233);
    M5.Lcd.print("> SELECT   ");
    M5.Lcd.setCursor(300, 233);
    M5.Lcd.print("<");

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
      M5.Lcd.setCursor(236, 233);
      M5.Lcd.print("> SELECT ");
      M5.Lcd.setCursor(300, 233);
      M5.Lcd.print("<");
      M5.Lcd.fillRoundRect(100, 136, 212, 94, 4, BLACK);
    } else {
      Serial.println("ERROR WRITE FILE");
    }
  }

  if (M5.BtnC.wasPressed()) {
    playSound("beep18.wav");
    // Hintergrundbox anzeigen
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(1);

    int topY = 140;
    int spacingY = 13;
    int boxX = 105;

    for (int i = 0; i < numLocations; i++) {
      int y = topY + i * spacingY;
      M5.Lcd.setCursor(boxX, y);
      M5.Lcd.print(seedBanks[i].name);
    }

    // Warten auf Berührung
    unsigned long startTime = millis();
    bool selected = false;
    while ((millis() - startTime < 10000) && !selected) {
      M5.update();
      M5.Touch.update();

      if (M5.Touch.ispressed()) {
        TouchPoint_t p = M5.Touch.getPressPoint();

        for (int i = 0; i < numLocations; i++) {
          int y = topY + i * spacingY;
          if (p.y > y && p.y < (y + spacingY)) {
            playSound("beep20.wav");
            // Auswahl visuell markieren
            M5.Lcd.fillRect(boxX - 3, y - 3, 210, spacingY, BLUE);  // Hervorhebung
            M5.Lcd.setTextColor(WHITE, BLUE);
            M5.Lcd.setCursor(boxX, y);
            M5.Lcd.print(seedBanks[i].name);
            delay(300);  // Kurze visuelle Bestätigung
            float lat = seedBanks[i].lat;
            float lon = seedBanks[i].lon;

            // Koordinaten speichern
            File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
            if (myFile) {
              myFile.print(lat, 6);
              myFile.print(",");
              myFile.println(lon, 6);
              myFile.close();

              homeLat = lat;
              homeLon = lon;
              updateNearestCity(homeLat, homeLon);
              displaySavedLocation();

              // Hinweis unten links anzeigen
              //M5.Lcd.fillRoundRect(16, 233, 220, 9, 4, BLACK);
              M5.Lcd.fillRect(16, 233, 284, 9, BLACK);
              M5.Lcd.drawFastHLine(21, 233, 268, CYAN);
              M5.Lcd.drawFastHLine(21, 239, 268, CYAN);
              M5.Lcd.setCursor(20, 233);
              M5.Lcd.setTextColor(YELLOW, BLACK);
              //M5.Lcd.print("> ");
              M5.Lcd.print(seedBanks[i].name);
              M5.Lcd.setTextColor(CYAN, BLACK);
              M5.Lcd.print(" <");

              //M5.Lcd.print("> HOME POS <");
              M5.Lcd.setCursor(238, 233);
              M5.Lcd.print("> SELECT   ");
              M5.Lcd.setCursor(300, 233);
              M5.Lcd.print("<");
              selected = true;
              delay(1000);
              break;
            } else {
              Serial.println("ERROR WRITE FILE");
            }
          }
        }
      }
    }
    // Menü schließen
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
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

  // SAT DISPLAY
  activeSatellites = 0;
  for (int i = 0; i < MAX_SATELLITES; ++i) {
    if (sats[i].active) {
      activeSatellites++;
    }
  }

  // Radar zeichnen (aufrufen!)
  drawRadarDisplay();
  // === RICHTUNGSANZEIGE (Kompass-Zeiger mit Freeze bei Langsamfahrt) ===
  static float lastValidCourse = 0;

  float currentCourse = gps.course.deg();
  float currentSpeed = gps.speed.kmph();

  if (gps.course.isValid() && gps.speed.isValid() && currentSpeed > 5.0) {
    lastValidCourse = currentCourse;
  }

  // Berechnung der relativen Richtung mit eingefrorenem Kurs
  float relCourse = courseToHome - lastValidCourse;
  if (relCourse < 0) relCourse += 360;
  if (relCourse >= 360) relCourse -= 360;

  M5.Lcd.fillCircle(52, 184, 32, BLACK);  // Kompassfläche löschen

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
    float angle = relCourse * rad_fac;
    int x = sin(angle) * 26;
    int y = cos(angle) * 26;
    int q = sin(angle);
    int r = cos(angle);
    int x1 = 53 + x;
    int y1 = 184 - y;
    int x2 = 53 - x;
    int y2 = 184 + y;
    int q1 = 53 + q;
    int r1 = 184 - r;
    int q2 = 53 - q;
    int r2 = 184 + r;

    double arrowAngle1 = relCourse + 200;
    x = sin(arrowAngle1 * rad_fac) * 48;
    y = cos(arrowAngle1 * rad_fac) * 48;
    q = sin(arrowAngle1 * rad_fac) * 48;
    r = cos(arrowAngle1 * rad_fac) * 48;
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;

    uint16_t arrowColor = (currentSpeed <= 3.0) ? ORANGE : YELLOW;

    if ((gps.location.lat() != 0) || (gps.location.lng() != 0)) {
      M5.Lcd.drawLine(x1, y1, x2, y2, arrowColor);
      M5.Lcd.drawLine(q1, r1, x2, y2, arrowColor);
    } else {
      M5.Lcd.drawCircle(52, 184, 30, NAVY);
      M5.Lcd.drawCircle(52, 184, 20, 0x00AF);
      M5.Lcd.drawCircle(52, 184, 10, BLUE);
      M5.Lcd.drawCircle(52, 184, 2, BLUE);
      M5.Lcd.drawFastVLine(52, 200, 20, DARKCYAN);
      M5.Lcd.drawFastVLine(52, 149, 20, DARKCYAN);
      M5.Lcd.drawFastHLine(68, 184, 20, DARKCYAN);
      M5.Lcd.drawFastHLine(17, 184, 20, DARKCYAN);
    }
    double arrowAngle2 = relCourse + 160;
    x = sin(arrowAngle2 * rad_fac) * 48;
    y = cos(arrowAngle2 * rad_fac) * 48;
    q = sin(arrowAngle2 * rad_fac) * 48;
    r = cos(arrowAngle2 * rad_fac) * 48;
    x2 = x1 + x;
    y2 = y1 - y;
    q2 = q1 + q;
    r2 = r1 - r;

    if ((gps.location.lat() != 0) || (gps.location.lng() != 0)) {
      M5.Lcd.drawLine(x1, y1, x2, y2, arrowColor);
      M5.Lcd.drawLine(q1, r1, x2, y2, arrowColor);
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

  // ⏱️ GPS-Ping bei aktivem Fix alle 30 Sekunden
  if (gps.location.isValid() && millis() - lastGpsPingTime >= gpsPingInterval) {
    //playSound("beep29.wav");  // oder ein anderer kurzer Ton
    playSound("sonar09.wav");
    Serial.println("📍 GPS-Ping (Fix OK)");
    lastGpsPingTime = millis();
  }

  const char *satIcon = gps.location.isValid() ? "/SAT2.png" : "/SAT1.png";
  M5.Lcd.drawPngFile(SD, satIcon, 100, 98);

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
  float CO = constrain(voltage_CO * 15.0, 0, 99);
  float NH3 = constrain(voltage_NH3 * 10.0, 0, 99);
  float NO2 = constrain(voltage_NO2 * 40.0, 0, 99);
  float EMF = constrain(voltage_EMF * 5.0, 0, 99);

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

  // ==== BATTERY DATA ====
  float batVoltage = M5.Axp.GetBatVoltage();
  float batPercentage = (batVoltage < 3.20) ? 0 : (batVoltage - 3.20) * 100 / (4.2 - 3.2);
  batPercentage = constrain(batPercentage, 0, 100);
  bool isCharging = M5.Axp.isACIN();

  // ==== Anzeige Setup ====
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(240, 5);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.print(batVoltage, 2);
  M5.Lcd.println("V");

  // ==== Farbe festlegen ====
  uint16_t batteryColor = (batPercentage < 20) ? RED : (batPercentage < 50 ? YELLOW : GREEN);

  // ==== Batterie-Rahmen ====
  M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);  // Hintergrund löschen
  M5.Lcd.drawRoundRect(275, 7, 35, 14, 3, GREENYELLOW);
  M5.Lcd.drawRoundRect(309, 11, 4, 6, 2, GREENYELLOW);

  // ==== Batterie-Füllung (immer anzeigen!) ====
  int fillWidth = map(batPercentage, 0, 100, 0, 31);
  M5.Lcd.fillRoundRect(277, 9, fillWidth, 10, 2, batteryColor);

  // ==== Ladeanimation (nur wenn geladen wird) ====
  if (isCharging) {

    M5.Lcd.fillTriangle(291, 14, 288, 14, 295, 7, RED);   // Pfeil oben
    M5.Lcd.fillTriangle(296, 13, 291, 13, 289, 20, RED);  // Pfeil unten
  }

  // ==== Textanzeige % ====
  M5.Lcd.setCursor(240, 14);
  M5.Lcd.setTextColor(batteryColor, BLACK);
  M5.Lcd.print(batPercentage, 0);
  M5.Lcd.print("% ");

  // ==== GPS-Anzeige mit Signalprüfung ====

  // 📡 LATTITUDE
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(104, 138);
  M5.Lcd.print("LATT:");

  if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites >= 3) {
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.fillRect(301, 138, 14, 14, BLACK);
    M5.Lcd.setCursor(163, 138);
    M5.Lcd.print(gps.location.lat() < 0 ? "S" : "N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7");
    M5.Lcd.setTextSize(2);
    char latBuf[16];
    dtostrf(gps.location.lat(), 10, 6, latBuf);
    M5.Lcd.print(latBuf);
  } else {
    M5.Lcd.setCursor(215, 138);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("---");
  }

  // 📡 LONGITUDE
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(104, 157);
  M5.Lcd.print("LONG:");

  if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites >= 3) {
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.fillRect(301, 157, 14, 14, BLACK);
    M5.Lcd.setCursor(163, 157);
    M5.Lcd.print(gps.location.lng() < 0 ? "W" : "E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7");
    M5.Lcd.setTextSize(2);
    char lngBuf[16];
    dtostrf(gps.location.lng(), 10, 6, lngBuf);
    M5.Lcd.print(lngBuf);
  } else {
    M5.Lcd.setCursor(215, 157);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print("---");
  }

  // ⛰ ALTITUDE
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(104, 176);
  M5.Lcd.print("ALTI:");

  static String lastAltitude = "";
  String altDisplay = "---";
  if (gps.altitude.isValid() && gps.location.isValid() && activeSatellites >= 3) {
    char altBuf[12];
    dtostrf(gps.altitude.meters(), 6, 2, altBuf);
    altDisplay = String(altBuf) + " m";
  }

  if (altDisplay != lastAltitude) {
    M5.Lcd.fillRect(250, 176, 65, 14, BLACK);
    M5.Lcd.setCursor(168, 176);
    M5.Lcd.setTextColor(altDisplay == "---" ? RED : CYAN, BLACK);
    M5.Lcd.print(altDisplay);
    lastAltitude = altDisplay;
  }

  // 🏃 SPEED
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(104, 196);
  M5.Lcd.print("SPED:");

  static String lastSpeed = "";
  String speedDisplay = "---";
  if (gps.speed.isValid() && gps.location.isValid() && activeSatellites >= 3) {
    char speedBuf[12];
    dtostrf(gps.speed.kmph(), 5, 1, speedBuf);
    speedDisplay = String(speedBuf) + " km/h";
  }

  if (speedDisplay != lastSpeed) {
    M5.Lcd.fillRect(250, 196, 65, 14, BLACK);
    M5.Lcd.setCursor(168, 196);
    M5.Lcd.setTextColor(speedDisplay == "---" ? RED : CYAN, BLACK);
    M5.Lcd.print(speedDisplay);
    lastSpeed = speedDisplay;
  }

  // 🎯 DESTINATION
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setCursor(104, 215);
  M5.Lcd.print("DEST:");

  static String lastDest = "";
  String distanceStr = "---";
  if (gps.location.isValid() && activeSatellites >= 3) {
    double distance = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), homeLat, homeLon);

    char buffer[10];
    if (distance < 1000) {
      snprintf(buffer, sizeof(buffer), "%d", (int)distance);
      distanceStr = String(buffer) + " m";
    } else {
      dtostrf(distance / 1000.0, 6, 2, buffer);
      distanceStr = String(buffer) + " km";
    }
  }

  if (distanceStr != lastDest) {
    M5.Lcd.fillRect(195, 215, 120, 14, BLACK);
    M5.Lcd.setCursor(164, 215);
    M5.Lcd.setTextColor(distanceStr == "---" ? RED : CYAN, BLACK);
    M5.Lcd.print(distanceStr);
    lastDest = distanceStr;
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
    hasArrived = false;  // Reset falls wieder entfernt

  } else {
    if (!hasArrived) {
      playSound("beep15.wav");  // 🎵 DESTINATION erreicht!
      Serial.println("🎯 DESTINATION erreicht!");
      hasArrived = true;
    }
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

//LOOP END//////////////////////////////////

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

void drawRadarDisplay() {
  const int centerX = 161, centerY = 76;
  const float rad_fac = 3.14159265359 / 180;

  // Nur wenn echte Koordinaten + mindestens 1 aktiver Satellit
  if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 0) {
    uint16_t radarColor = DARKGREEN;
    M5.Lcd.drawCircle(centerX, centerY, 47, radarColor);
    M5.Lcd.drawCircle(centerX, centerY, 16, radarColor);
    M5.Lcd.drawCircle(centerX, centerY, 32, radarColor);
    M5.Lcd.drawFastHLine(centerX - 46, centerY, 94, radarColor);
    M5.Lcd.drawFastVLine(centerX, centerY - 46, 94, radarColor);
    M5.Lcd.drawLine(129, 44, 193, 108, radarColor);
    M5.Lcd.drawLine(128, 109, 193, 44, radarColor);

    for (int i = 0; i < MAX_SATELLITES; ++i) {
      // Entferne alte Positionen, wenn sie ungültig oder nicht mehr angezeigt werden sollen
      if (oldX[i] >= 0 && oldY[i] >= 0) {
        M5.Lcd.drawCircle(oldX[i], oldY[i], oldCircleSize1[i], BLACK);
        M5.Lcd.fillCircle(oldX[i], oldY[i], oldCircleSize2[i], BLACK);
        oldX[i] = oldY[i] = -1;
        oldCircleSize1[i] = oldCircleSize2[i] = 0;
      }

      // Nur Satelliten anzeigen, die eine gültige SNR (Signal-to-Noise-Ratio) haben und eine gültige Position besitzen
      if (sats[i].active && sats[i].snr > 1 && sats[i].snr <= 40 && sats[i].elevation > 0) {
        float az_r = sats[i].azimuth * rad_fac;
        float e = 42 * (90 - sats[i].elevation / 2) / 90;
        int x = centerX - (sin(az_r) * e);
        int y = centerY - (cos(az_r) * e);

        // Nur anzeigen, wenn die berechnete Position innerhalb des Bildschirmbereichs liegt
        if (x >= 0 && y >= 0 && x < 320 && y < 240) {
          uint16_t circleColor = (sats[i].snr <= 10) ? YELLOW : (sats[i].snr <= 20) ? GREENYELLOW
                                                              : (sats[i].snr <= 30) ? GREEN
                                                                                    : DARKGREEN;

          int circleSize = map(sats[i].snr, 1, 40, 2, 8);
          M5.Lcd.drawCircle(x, y, circleSize, circleColor);
          M5.Lcd.fillCircle(x, y, 1, WHITE);

          // Speichern der aktuellen Position für die nächste Iteration
          oldX[i] = x;
          oldY[i] = y;
          oldCircleSize1[i] = circleSize;
          oldCircleSize2[i] = 1;
        }
      }
    }
  } else {

    // Wenn GPS-Signal schwach oder ungültig ist, Radar in grauer Farbe anzeigen
    uint16_t radarColor = DARKGREY;
    M5.Lcd.drawCircle(centerX, centerY, 47, radarColor);
    M5.Lcd.drawCircle(centerX, centerY, 16, radarColor);
    M5.Lcd.drawCircle(centerX, centerY, 32, radarColor);
    M5.Lcd.drawFastHLine(centerX - 46, centerY, 94, radarColor);
    M5.Lcd.drawFastVLine(centerX, centerY - 46, 94, radarColor);
    M5.Lcd.drawLine(129, 44, 193, 108, radarColor);
    M5.Lcd.drawLine(128, 109, 193, 44, radarColor);

    // Entferne alle alten Satellitenpositionen, wenn kein gültiges GPS-Signal vorhanden ist
    for (int i = 0; i < MAX_SATELLITES; ++i) {
      if (oldX[i] >= 0 && oldY[i] >= 0) {
        M5.Lcd.drawCircle(oldX[i], oldY[i], oldCircleSize1[i], BLACK);
        M5.Lcd.fillCircle(oldX[i], oldY[i], oldCircleSize2[i], BLACK);
        oldX[i] = oldY[i] = -1;
        oldCircleSize1[i] = oldCircleSize2[i] = 0;
      }
    }
  }
}

// Funktion zum Einlesen der gespeicherten Koordinaten aus der Datei
void loadTargetsFromFile() {
  File myFile = SD.open("/home_coordinates.txt", FILE_READ);

  if (myFile) {
    targetList.clear();  // Leeren der Liste, bevor neue Ziele geladen werden
    while (myFile.available()) {
      String line = myFile.readStringUntil('\n');
      int commaIndex = line.indexOf(',');

      if (commaIndex != -1) {
        String latStr = line.substring(0, commaIndex);
        String lonStr = line.substring(commaIndex + 1);

        float lat = latStr.toFloat();
        float lon = lonStr.toFloat();

        String city = "CityName";        // Beispiel-Stadt (hier könnte eine tatsächliche Geocoding-API verwendet werden)
        String country = "CountryName";  // Beispiel-Land (hier könnte eine tatsächliche Geocoding-API verwendet werden)

        // Füge den neuen Zielort in die Liste ein
        TargetLocation newTarget = { lat, lon, city, country };
        targetList.push_back(newTarget);
      }
    }
    myFile.close();
  } else {
    Serial.println("Fehler beim Öffnen der Datei zum Lesen!");
  }
}

// Funktion zum Anzeigen der Zielauswahl
void showTargetSelectionMenu() {
  playSound("beep09.wav");

  M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  M5.Lcd.setTextSize(1);

  int topY = 140;
  int spacingY = 13;
  int boxX = 105;

  for (int i = 0; i < targetList.size() && i < maxTargets; i++) {
    int y = topY + i * spacingY;
    M5.Lcd.setCursor(boxX, y);

    if (i == 0) {
      M5.Lcd.setTextColor(YELLOW, BLACK);
    } else {
      M5.Lcd.setTextColor(GREEN, BLACK);
    }

    if (targetList[i].city.length() > 0) {
      M5.Lcd.printf("%s, %s", targetList[i].city.c_str(), targetList[i].country.c_str());
    } else {
      M5.Lcd.print("...");
    }
  }

  unsigned long startTime = millis();
  bool selected = false;

  while ((millis() - startTime < 10000) && !selected) {
    M5.update();
    M5.Touch.update();

    if (M5.Touch.ispressed()) {
      TouchPoint_t p = M5.Touch.getPressPoint();

      for (int i = 0; i < targetList.size() && i < maxTargets; i++) {
        int y = topY + i * spacingY;
        if (p.y > y && p.y < (y + spacingY)) {
          playSound("beep22.wav");

          M5.Lcd.fillRect(boxX - 3, y - 3, 210, spacingY, BLUE);
          M5.Lcd.setTextColor(WHITE, BLUE);
          M5.Lcd.setCursor(boxX, y);
          M5.Lcd.printf("%s, %s", targetList[i].city.c_str(), targetList[i].country.c_str());
          delay(300);

          // Ziel setzen
          homeLat = targetList[i].lat;
          homeLon = targetList[i].lon;
          updateNearestCity(homeLat, homeLon);

          // ✅ Koordinaten speichern, damit sie beim Neustart angezeigt werden
          File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
          if (myFile) {
            myFile.print(homeLat, 6);
            myFile.print(",");
            myFile.println(homeLon, 6);
            myFile.close();
          }

          // Anzeige unten aktualisieren
          M5.Lcd.fillRect(16, 233, 284, 9, BLACK);
          M5.Lcd.drawFastHLine(21, 233, 268, CYAN);
          M5.Lcd.drawFastHLine(21, 239, 268, CYAN);
          M5.Lcd.setCursor(20, 233);
          M5.Lcd.setTextColor(YELLOW, BLACK);
          M5.Lcd.printf("%s, %s", targetList[i].city.c_str(), targetList[i].country.c_str());
          M5.Lcd.setTextColor(CYAN, BLACK);
          M5.Lcd.print(" <");
          M5.Lcd.setCursor(238, 233);
          M5.Lcd.print("> SELECT   ");
          M5.Lcd.setCursor(300, 233);
          M5.Lcd.print("<");

          selected = true;
          delay(1000);
          break;
        }
      }
    }
  }

  M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
}
