#include <M5Core2.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <DFRobot_QMC5883.h>
#include <Adafruit_ADS1X15.h>
#include <cactus_io_BME280_I2C.h>
#include <MiCS6814-I2C.h>
#include "FONT1.h"
#include "FONT2.h"
#include "CITIES.h"
#include "moonPhase.h"
bool silentMode = false;        // Steuerung laut/lautlos
bool uiReady = false;           // UI vollständig aufgebaut?
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

  // 🔋 Akkustand-Warnsystem: Zustand über mehrere Schleifen behalten
  static int lastNotifiedStep = -1;      // Zuletzt gemeldeter Warnwert
  static bool firstBatteryCheck = true;  // Nur beim allerersten Durchlauf

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
  const char *soundFile;
};

Location seedBanks[] = {
  { "CGN WAGENINGEN NETHERLANDS", 51.9863, 5.6680, "cgn.wav" },
  { "IPK GATERSLEBEN GERMANY", 51.8247, 11.2792, "ipk.wav" },
  { "NORDGEN PLANTS ALNARP SWEDEN", 55.6601, 13.0834, "nordgen.wav" },
  { "INRAE PARIS FRANCE", 48.8611, 2.3062, "inrae.wav" },
  { "MILLENIUMSEED BANK LONDON ENGLAND", 51.0688, -0.0899, "millenium.wav" },
  { "GERMOPLASMA MADRID SPAIN", 40.4425, -3.7291, "germoplasma.wav" },
  { "GLOBAL SEED VAULT SVALBARD NORWAY", 78.1409, 15.2929, "global.wav" }
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

    // GPS-Daten in struct tm speichern
    timeinfo.tm_year = gps.date.year() - 1900;  // tm_year zählt ab 1900
    timeinfo.tm_mon = gps.date.month() - 1;     // tm_mon zählt ab 0 (Jan = 0)
    timeinfo.tm_mday = gps.date.day();
    timeinfo.tm_hour = gps.time.hour();
    timeinfo.tm_min = gps.time.minute();
    timeinfo.tm_sec = gps.time.second();

    // struct tm in UNIX-Zeit umwandeln
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
  if (SD.exists("/graphics/radiation.png")) {
    M5.Lcd.drawPngFile(SD, "/graphics/radiation.png", 198, 101);
    delay(500);
  } else {
    Serial.println("Fehler: radiation.png nicht gefunden!");
  }
}
float doseRate = 0.0;     // Dosis in µSv/h
float averageDose = 0.0;  // Durchschnittliche Dosis

const float calibrationFactor = 153.0;  //140-160 for SBM-20 153.0; for J613/J614  // calibration: CPM/µSv/h
// Historie für Durchschnittswerte
#define RATE_GRAPH_WIDTH 83
#define AVG_GRAPH_WIDTH 17
#define MOON_PHASES_PATH "/phases/"  // Ordner für die PNG-Dateien auf der SD-Karte
DFRobot_QMC5883 compass(&Wire, 0x0D);
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

const float rad_fac = 0.0174532925;
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
static bool wasAlarmTriggered = false;  // Stores whether an alarm was previously triggered

// Trigger vibration (and optional LED) pattern
void triggerVibrationPattern(const int pattern[], int len) {
  for (int i = 0; i < len; i++) {
    if (i % 2 == 0) {
      M5.Axp.SetVibration(true);
      // M5.Axp.SetLed(true);
    } else {
      M5.Axp.SetVibration(false);
      // M5.Axp.SetLed(false);
    }
    delay(pattern[i]);
  }
  M5.Axp.SetVibration(false);
  M5.Axp.SetLed(false);
}

// Show alarm visually with icon and message
void showAlarm(const char *pngFile, int textCursorX, int textCursorY, const char *message) {
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.setCursor(textCursorX, textCursorY);
  M5.Lcd.print(message);
  M5.Lcd.drawPngFile(SD, pngFile, 22, 31);
}

// Main alarm logic for all sensor types
void checkForAlarms(float CO, float NH3, float NO2, float EMF, float radiation) {
  alarmTriggered = false;

  // CO warning
  if (CO > CO_THRESHOLD) {
    if (!silentMode) playSound("alarmgas.wav");
    showAlarm("/graphics/hazard.png", 13, 79, " !!!DANGER!!!");
    int patternCO[] = { 400, 100, 200, 100, 400 };
    triggerVibrationPattern(patternCO, sizeof(patternCO) / sizeof(patternCO[0]));
    alarmTriggered = true;
  }

  // NH3 warning
  if (NH3 > NH3_THRESHOLD) {
    if (!silentMode) playSound("alarmgas.wav");
    showAlarm("/graphics/hazard.png", 13, 90, " !!!DANGER!!!");
    int patternNH3[] = { 400, 100, 400, 100, 200 };
    triggerVibrationPattern(patternNH3, sizeof(patternNH3) / sizeof(patternNH3[0]));
    alarmTriggered = true;
  }

  // NO2 warning
  if (NO2 > NO2_THRESHOLD) {
    if (!silentMode) playSound("alarmgas.wav");
    showAlarm("/graphics/hazard.png", 13, 101, " !!!DANGER!!!");
    int patternNO2[] = { 200, 100, 200, 100, 400 };
    triggerVibrationPattern(patternNO2, sizeof(patternNO2) / sizeof(patternNO2[0]));
    alarmTriggered = true;
  }

  // EMF warning
  if (EMF > EMF_THRESHOLD) {
    if (!silentMode) playSound("alarmemf.wav");
    showAlarm("/graphics/EMF.png", 13, 112, " !!!DANGER!!!");
    int patternEMF[] = { 200, 100, 200, 100, 100 };
    triggerVibrationPattern(patternEMF, sizeof(patternEMF) / sizeof(patternEMF[0]));
    alarmTriggered = true;
  }

  // Radiation warning (only if no other alarm was triggered yet)
  if (radiation > RADIATION_THRESHOLD && !alarmTriggered) {
    if (!silentMode) playSound("alarmradiation.wav");
    M5.Lcd.drawPngFile(SD, "graphics/radiation3.png", 237, 31);
    int patternRadiation[] = { 200, 100, 400, 100, 200 };
    triggerVibrationPattern(patternRadiation, sizeof(patternRadiation) / sizeof(patternRadiation[0]));
    alarmTriggered = true;
  }

  // Clear alert visuals if transitioning from alarm to normal state
  if (wasAlarmTriggered && !alarmTriggered) {
    M5.Lcd.fillRoundRect(9, 29, 87, 94, 4, BLACK);  // Remove graphical artifacts
  }

  wasAlarmTriggered = alarmTriggered;  // Store last state
}

// Arrays to store previous satellite display data (for clearing old positions)
int oldX[MAX_SATELLITES] = { 0 };
int oldY[MAX_SATELLITES] = { 0 };
int oldCircleSize1[MAX_SATELLITES] = { 0 };
int oldCircleSize2[MAX_SATELLITES] = { 0 };

bool displayOn = true;  // Current display state

// Forward declarations
void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
void displayMillisecondProgress();
void printStr(const char *str, int len);

// Store nearest city name
String savedCity = "N/A";

// Update nearest city and country from coordinates
void updateNearestCity(float latitude, float longitude) {
  findNearestCity(latitude, longitude, savedCity, nearestCountry);
}


// Interrupt function: Count Geiger counter pulses and set the drawing flag
void IRAM_ATTR countPulse() {
  pulseCount++;           // Count pulse
  drawBitmapFlag = true;  // Set bitmap draw flag
}

unsigned long lastUpdateTime = 0;            // Last update time for moon phase
const unsigned long updateInterval = 60000;  // Update interval: 60 seconds

// Calculate Central European Time offset (CET / CEST)
int calculateCET(TinyGPSDate &date, TinyGPSTime &time) {
  int year = date.year();
  int month = date.month();
  int day = date.day();

  // Zeller's congruence for calculating weekday (0=Sunday, ..., 6=Saturday)
  int m = month;
  int y = year;
  if (m < 3) {
    m += 12;
    y -= 1;
  }

  int k = y % 100;
  int c = y / 100;
  int weekday = (day + (13 * (m + 1)) / 5 + k + (k / 4) + (c / 4) - (2 * c)) % 7;
  weekday = (weekday + 6) % 7;  // Adjust: 0=Sunday, ..., 6=Saturday

  // Lambda function to get the last Sunday of a given month
  auto getLastSunday = [](int y, int m) -> int {
    int nextMonth = (m == 12) ? 1 : m + 1;
    int nextYear = (m == 12) ? y + 1 : y;
    struct tm firstNextMonth = { 0, 0, 0, 1, nextMonth - 1, nextYear - 1900 };
    time_t firstNextMonthTime = mktime(&firstNextMonth);
    time_t lastSundayTime = firstNextMonthTime - (24 * 3600);  // Go back one day

    while (localtime(&lastSundayTime)->tm_wday != 0) {  // Until Sunday is found
      lastSundayTime -= 24 * 3600;
    }
    return localtime(&lastSundayTime)->tm_mday;
  };

  // Daylight saving time (DST): last Sunday in March to last Sunday in October
  int lastSundayMarch = getLastSunday(year, 3);
  int lastSundayOctober = getLastSunday(year, 10);

  if ((month > 3 && month < 10) || (month == 3 && day >= lastSundayMarch) || (month == 10 && day < lastSundayOctober)) {
    return 2;  // Summer time (UTC+2)
  } else {
    return 1;  // Winter time (UTC+1)
  }
}

// Fixed reference coordinates (e.g., home location)
const double latHomeFixed = 51.861120;
const double lonHomeFixed = 8.289310;

// Calculate distance between two GPS coordinates using Haversine formula
double distanceBetween(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;  // Earth radius in meters
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

int activeSatellites = 0;
double currentLat = gps.location.lat();
double currentLon = gps.location.lng();

// Structure for storing a target location
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

// Save target list to SD card
void saveTargetList() {
  SD.remove("/last_targets.txt");  // ✅ Delete existing file before writing new one
  File file = SD.open("/last_targets.txt", FILE_WRITE);
  if (!file) {
    Serial.println("❌ Failed to open last_targets.txt for writing.");
    return;
  }

  for (const auto &target : targetList) {
    file.printf("%.6f,%.6f,%s,%s\n", target.lat, target.lon, target.city.c_str(), target.country.c_str());
  }

  file.close();
  Serial.println("✅ Target list saved.");
}

// Load target list from SD card
void loadTargetList() {
  targetList.clear();

  File file = SD.open("/last_targets.txt");
  if (!file) {
    Serial.println("📂 No saved targets found.");
    return;
  }

  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      double lat, lon;
      int firstComma = line.indexOf(',');
      int secondComma = line.indexOf(',', firstComma + 1);
      int thirdComma = line.indexOf(',', secondComma + 1);

      if (firstComma == -1 || secondComma == -1 || thirdComma == -1) {
        Serial.println("❌ Invalid line in last_targets.txt:");
        Serial.println(line);
        continue;
      }

      lat = line.substring(0, firstComma).toDouble();
      lon = line.substring(firstComma + 1, secondComma).toDouble();
      String city = line.substring(secondComma + 1, thirdComma);
      String country = line.substring(thirdComma + 1);

      TargetLocation t = { lat, lon, city, country };
      targetList.push_back(t);
    }
  }

  file.close();
  Serial.printf("📌 %d targets loaded from SD.\n", targetList.size());
}

/////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  M5.begin();
  M5.Axp.SetSpkEnable(true);  // Aktiviert den internen Audio-Verstärker
  M5.Spk.begin();     // Speaker
  
  pinMode(geigerPin, INPUT);                                              // Set pin 26 as input
  attachInterrupt(digitalPinToInterrupt(geigerPin), countPulse, RISING);  // Interrupt on rising edge

  M5.Lcd.fillScreen(BLACK);

  MySerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

  Wire.begin(32, 33);                         // I2C pins
 
 // QMC5883 Compass initialisieren
  compass.begin();
  compass.setRange(QMC5883_RANGE_2GA);
  compass.setMeasurementMode(QMC5883_CONTINOUS);
  compass.setDataRate(QMC5883_DATARATE_50HZ);
  compass.setSamples(QMC5883_SAMPLES_8);
   // BME280 initialisieren
  bme.begin();                            

  // Start GPS serial
  ss.begin(GPSBaud);  
  
  // Optional: turn off onboard LED
  disablePowerLed();  

  // ADS1115 i2C Modul initialisieren
  ads.begin();


  // Initialize graph buffer with mid-line Y-values
  for (int i = 0; i < RATE_GRAPH_WIDTH; i++) {
    rateGraphBuffer[i] = 90;
  }

  if (!SD.begin(TFCARD_CS_PIN, SPI, 40000000UL)) {
    Serial.println("SD card initialization failed! Restarting...");
    ESP.restart();
  }

  loadTargetList();  // ⬅️ Load previously saved locations from SD

  M5.Axp.SetLcdVoltage(3167);  // Set LCD backlight voltage
  bme.setTempCal(-4);          // Temperature offset calibration

  // Load home coordinates from file
  File myFile = SD.open("/home_coordinates.txt");
  if (myFile) {
    String coordinates = myFile.readStringUntil('\n');
    myFile.close();

    int commaIndex = coordinates.indexOf(',');
    if (commaIndex > 0) {
      homeLat = coordinates.substring(0, commaIndex).toFloat();
      homeLon = coordinates.substring(commaIndex + 1).toFloat();
      updateNearestCity(homeLat, homeLon);
    }
  } else {
    Serial.println("No stored coordinates found.");
  }

  //if (!silentMode) playSound("beep50.wav");
if (!silentMode) playSound("beep62.wav");
  // Splash screen with timeout or touch
  bool pngDrawn = false;
  unsigned long startMillis = millis();
  const unsigned long timeout = 10000;  // 10-second timeout

  while (!pngDrawn) {
    if (M5.Touch.ispressed()) {
      delay(50);
      pngDrawn = true;  // ⬅️ Touch detected
    } else if (millis() - startMillis >= timeout) {
      Serial.println("⏱️ Continuing automatically after 10 seconds.");
      pngDrawn = true;  // ⬅️ Timeout reached
    } else {
      M5.update();

      // 🔽 Draw splash screen image
      M5.Lcd.drawPngFile(SD, "/graphics/radar2.png", 0, 40);

      // Display coordinates and nearest city info
      File myFile = SD.open("/home_coordinates.txt", FILE_READ);
      if (myFile) {
        String line = myFile.readStringUntil('\n');
        int commaIndex = line.indexOf(",");
        if (commaIndex != -1) {
          homeLat = line.substring(0, commaIndex).toDouble();
          homeLon = line.substring(commaIndex + 1).toDouble();
          updateNearestCity(homeLat, homeLon);
        } else {
          Serial.println("❌ Error: Invalid file content!");
        }
        myFile.close();
      } else {
        Serial.println("📂 File '/home_coordinates.txt' not found!");
      }

      findNearestCity(homeLat, homeLon, nearestCity, nearestCountry);

      // 🎨 Display coordinates and target location info
      M5.Lcd.drawPngFile(SD, "/graphics/garpax.png", 250, 0);

      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setTextColor(DARKCYAN, BLACK);
      M5.Lcd.setCursor(10, 40);
      M5.Lcd.print("MULITISENS-GARPAX");

      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.setCursor(100, 80);
      M5.Lcd.print("SAVED COORDINATES:");

      // Display saved city name
      M5.Lcd.setFreeFont(&SFChromeFendersCondensed16pt7b);
      M5.Lcd.setCursor(133, 164);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.print(nearestCity);

      // Display saved country
      M5.Lcd.setCursor(160, 130);
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.print(nearestCountry);

      // Display latitude and longitude
      M5.Lcd.setFreeFont(&CONTF___12pt7b);
      M5.Lcd.setTextColor(CYAN, BLACK);

      // Latitude
      M5.Lcd.setCursor(126, 205);
      M5.Lcd.print("LATI: N ");
      M5.Lcd.setCursor(196, 190);
      M5.Lcd.print("\xF7 ");
      M5.Lcd.setCursor(206, 205);
      M5.Lcd.print(homeLat, 6);

      // Longitude
      M5.Lcd.setCursor(120, 228);
      M5.Lcd.print("LONG: E ");
      M5.Lcd.setCursor(195, 214);
      M5.Lcd.print("\xF7 ");
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
  M5.Lcd.print(savedCity);  // Display saved location
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(" <");
  M5.Lcd.setCursor(238, 233);
        M5.Lcd.print(">  SELECT  ");
        M5.Lcd.setCursor(300, 233);
        M5.Lcd.print("<");

  updateVolumeBar();  // Show volume bar on boot
}

uint16_t getGradientColor(float value, float minValue, float maxValue) {
  float normalized = (value - minValue) / (maxValue - minValue);
  normalized = constrain(normalized, 0.0, 1.0);

  uint8_t red = 255 * (1.0 - normalized);
  uint8_t green = 255 * normalized;
  uint8_t blue = 0;

  return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
}
void drawRateGraph(float doseRate) {
  static float sumDoseRate = 0;
  static int countSamples = 0;
  static bool graphNeedsUpdate = false;

  static float filteredDose = 0;
  float alpha = 0.05;
  filteredDose = alpha * doseRate + (1 - alpha) * filteredDose;
  sumDoseRate += filteredDose;
  countSamples++;

  // Update graph after ~7.23 samples
  if (countSamples >= 7.23) {
    int newY = 80 - ((sumDoseRate / countSamples) * 3000);
    newY = constrain(newY, 40, 80);

    rateGraphBuffer[rateGraphIndex] = newY;
    rateGraphIndex = (rateGraphIndex + 1) % RATE_GRAPH_WIDTH;

    sumDoseRate = 0;
    countSamples = 0;
    graphNeedsUpdate = true;
  }

  if (!graphNeedsUpdate) return;
  graphNeedsUpdate = false;


  // Clear graph area
  M5.Lcd.fillRect(225, 31, 86, 60, BLACK);

  // Draw grid
  for (int y = 40; y <= 120; y += 10)
    M5.Lcd.drawFastHLine(225, y, 85, 0x0320);
  for (int x = 225; x <= 309; x += 14)
    M5.Lcd.drawFastVLine(x, 40, 40, 0x0320);

  // Plot graph line
  for (int i = 0; i < RATE_GRAPH_WIDTH - 1; i++) {
    int x1 = 226 + i;
    int y1 = rateGraphBuffer[(rateGraphIndex + i) % RATE_GRAPH_WIDTH];
    int x2 = 226 + i + 1;
    int y2 = rateGraphBuffer[(rateGraphIndex + i + 1) % RATE_GRAPH_WIDTH];

    uint16_t color = getGradientColor(y1, 40, 80);
    M5.Lcd.drawLine(x1, y1, x2, y2, color);
  }
}


void drawAverageGraph(float avgDose) {
  int scaledHeight = map(avgDose, 0, 10, 0, 68);
  if (scaledHeight < 1 && avgDose > 0) scaledHeight = 1;

  avgGraphBuffer[avgGraphIndex] = constrain(scaledHeight, 0, 68);
  avgGraphIndex = (avgGraphIndex + 1) % AVG_GRAPH_WIDTH;

  // Clear average graph area
  M5.Lcd.fillRect(226, 90, 84, 33, BLACK);

  // Draw vertical bars
  for (int i = 0; i < AVG_GRAPH_WIDTH; i++) {
    int x = 226 + i * 5;
    int height = avgGraphBuffer[(avgGraphIndex + i) % AVG_GRAPH_WIDTH];
    int y = 120 - height;

    float dose = height / 10.0;
    uint16_t color = (dose < 1.0) ? GREEN : (dose < 2.0) ? YELLOW
                                          : (dose < 5.0) ? ORANGE
                                                         : RED;

    M5.Lcd.fillRect(x, y, 4, height, color);
  }
}

void updateDoseRate() {
  unsigned long count = pulseCount;
  pulseCount = 0;

  float doseRate = count / calibrationFactor;

  // Use last valid value if doseRate is too low
  if (doseRate <= 0.01) {
    doseRate = lastValidDoseRate;
  } else {
    lastValidDoseRate = doseRate;
  }

  // Accumulate for averaging
  static float sumDose = 0;
  static int countSamples = 0;
  sumDose += doseRate;
  countSamples++;

  // Update average graph every ~3.75 minutes
  if (countSamples >= 225) {
    float avgDose = sumDose / countSamples;
    drawAverageGraph(avgDose);
    sumDose = 0;
    countSamples = 0;
  }

  drawRateGraph(doseRate);
  displayValues(doseRate, countSamples > 0 ? sumDose / countSamples : 0);
}


void displayValues(float doseRate, float averageDose) {
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(227, 31);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("DR:");

  // Display last valid dose rate if current is too low
  if (doseRate <= 0.01)
    doseRate = lastValidDoseRate;
  else
    lastValidDoseRate = doseRate;

  doseRate = min(doseRate, 99.0f);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.printf("%.2f uSv/h", doseRate);

  M5.Lcd.setCursor(227, 82);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("AD:");

  averageDose = min(averageDose, 99.0f);
  uint16_t avgColor = (averageDose < 0.5) ? GREEN : (averageDose < 1.0) ? YELLOW
                                                  : (averageDose < 2.0) ? ORANGE
                                                                        : RED;

  M5.Lcd.setTextColor(avgColor, BLACK);
  M5.Lcd.printf("%.2f uSv/h", averageDose);
}

//🕒
int lastSecond = -1;  // To track changes in seconds

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

      // Convert UTC to local time (CET/CEST)
      int cetOffset = calculateCET(d, t);
      int hourCET = t.hour() + cetOffset;
      if (hourCET >= 24) hourCET -= 24;  // Zeit korrekt anpassen

      unsigned long ms = currentMillis % 1000;
      sprintf(sz, "%02d:%02d:%02d", hourCET, t.minute(), t.second());
      uint16_t timeColor = (d.isValid() && t.isValid()) ? CYAN : RED;

      M5.Lcd.fillRect(232, 6, 8, 14, BLACK);
      M5.Lcd.setTextColor(timeColor, BLACK);
      M5.Lcd.print(sz);
    }
  }
}
//🖨️
void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i = 0; i < len; ++i) {
    Serial.print(i < slen ? str[i] : ' ');  // Pad with spaces if string is too short
  }
}
//🌡️
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
//🌤️
void updateWeatherIcon(const String &newIcon) {
  if (newIcon != lastWeatherIcon) {
    M5.Lcd.fillRoundRect(100, 26, 24, 24, 4, BLACK);
    if (SD.exists(newIcon.c_str())) {
      M5.Lcd.drawPngFile(SD, newIcon.c_str(), 100, 26);
    } else {
      Serial.print("Error: Weather icon not found -> ");
      Serial.println(newIcon);
    }
    lastWeatherIcon = newIcon;
  }
}
//🧭
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

// ☁️
void updateWeatherDisplay() {
  bme.readSensor();
  float avgPressure = getAveragePressure();
  float pressure = bme.getPressure_HP() / 100;
  lastPressures[pressureIndex] = pressure;
  pressureIndex = (pressureIndex + 1) % 10;  // Immer zyklisch zwischen 0-4

  String weatherIcon;
  String arrowIcon = lastArrowIcon;  // Standardmäßig das letzte Icon behalten

  M5.Lcd.setTextSize(1);

  // Temperature
  M5.Lcd.drawRoundRect(12, 31, 82, 11, 2, 0x00AF);
  M5.Lcd.setCursor(16, 33);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("T:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getTemperature_C(), 1);
  M5.Lcd.println(" C");

  // Humidity
  M5.Lcd.drawRoundRect(12, 42, 82, 11, 2, 0x00AF);
  M5.Lcd.setCursor(16, 44);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("H:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(bme.getHumidity(), 0);
  M5.Lcd.println(" %");

  // Pressure
  M5.Lcd.drawRoundRect(12, 53, 82, 11, 2, 0x00AF);
  M5.Lcd.drawRoundRect(12, 64, 82, 12, 2, 0x00AF);
  M5.Lcd.setCursor(16, 55);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.print("P:");
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.print(pressure, 0);
  M5.Lcd.println(" HPa");

  // Forecast description and icon
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
      arrowIcon = (pressureDiff > 0) ? "/graphics/up.png" : "/graphics/down.png";
    }
  }

  // Icons aktualisieren
  updateWeatherIcon(weatherIcon);
  updateArrowIcon(arrowIcon);
}
void handleSilentTouch() {

  int centerX = 161;
  int centerY = 76;
  int radius = 47;

  if (M5.Touch.ispressed()) {
    delay(50);
    TouchPoint_t p = M5.Touch.getPressPoint();
    int dx = p.x - centerX;
    int dy = p.y - centerY;

    if ((dx * dx + dy * dy) <= (radius * radius)) {
      silentMode = !silentMode;
      drawSilentIcon();
      delay(300);
    }
  }
  uiReady = true;
}

////////////////////////////////////////////////////////////////

void loop() {
  M5.update();  // Update touch events

  // Read GPS data as soon as it arrives
  while (MySerial.available()) {
    gps.encode(MySerial.read());
  }

  printDateTime(gps.date, gps.time);  // Display real-time clock with milliseconds
  updateTimeFromGPS();                // Update time and moon phase from GPS

  if (drawBitmapFlag) {
    drawPNGGeigerSignal();   // Draw the Geiger bitmap
    drawBitmapFlag = false;  // Reset the flag
  }

  // If GPS date and time are valid
  if (gps.date.isValid() && gps.time.isValid()) {
    // Extract date from GPS
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();

    // Get CET offset (summer/winter time adjustment)
    int cet = calculateCET(gps.date, gps.time);

    // Adjust hour according to CET
    int hour = gps.time.hour() + ((cet == 2) ? 2 : 1);  // UTC+2 for DST, UTC+1 for standard time

    unsigned long currentMillis = millis();

    // Check if it's time to update moon phase
    if (currentMillis - lastUpdateTime >= updateInterval) {
      // Make sure GPS time is valid
      if (timeinfo.tm_year >= 120) {                    // 120 = year 2020 (tm_year starts from 1900)
        time_t gps_time = mktime(&timeinfo);            // Convert to UNIX time
        moonData_t moonData = moon.getPhase(gps_time);  // Calculate moon phase
        int phaseIndex = (int)(moonData.angle / 360.0 * 30) % 30;

        // Generate moon phase filename
        char filename[32];
        snprintf(filename, sizeof(filename), "%s%02d.png", MOON_PHASES_PATH, phaseIndex);

        // Clear old icon and display new one
        M5.Lcd.fillRoundRect(199, 25, 24, 24, 6, BLACK);
        M5.Lcd.drawPngFile(SD, filename, 200, 29);

        lastUpdateTime = currentMillis;  // Save update time
      } else {
        Serial.println("Error: Invalid GPS time, moon phase not updated.");
      }
    }
  }
  handleSilentTouch();  // Check touch silence zone

  // === VOLUME ===
  static int volumeStep = 1;                 // Initial volume level (0 = mute, 6 = max)
  volumeStep = constrain(volumeStep, 0, 6);  // Safety bounds
  volumeFactor = volumeStep / 6.0;

  // === DRAW VOLUME BAR ===
  const int barHeight = 202;
  const int barWidth = 2;
  const int barX = 317, barY = 230;  // Bottom coordinate
  const int steps = 6;
  const uint16_t barColor = WHITE;

  M5.Lcd.fillRect(barX, barY - barHeight, barWidth, barHeight, BLACK);  // Clear background

  // Draw filled volume level
  int fillHeight = (volumeStep * barHeight) / steps;
  M5.Lcd.fillRect(barX, barY - fillHeight, barWidth, fillHeight, barColor);

  // Draw separator lines
  for (int i = 1; i < steps; ++i) {
    int y = barY - (i * barHeight / steps);
    M5.Lcd.drawFastHLine(barX, y, barWidth, BLACK);
  }

  // === TOUCH INTERACTION ===
  if (M5.Touch.ispressed()) {
    delay(50);  // Debounce
    TouchPoint_t touchPoint = M5.Touch.getPressPoint();

    // 🔊 Increase volume (top right)
    if (touchPoint.x >= 220 && touchPoint.x <= 320 && touchPoint.y >= 0 && touchPoint.y <= 100) {
      if (volumeStep < 6) {
        volumeStep++;
        volumeFactor = volumeStep / 6.0;
        playSound("volumeup5.wav");
        Serial.printf("🔊 Volume up: %.2f\n", volumeFactor);
      }
    }

    // 🔉 Decrease volume (bottom right)
    if (touchPoint.x >= 220 && touchPoint.x <= 320 && touchPoint.y >= 140 && touchPoint.y <= 240) {
      if (volumeStep > 0) {
        volumeStep--;
        volumeFactor = volumeStep / 6.0;
        playSound("volumedown5.wav");
      }
    }

    // === TOUCH: BRIGHTNESS CONTROL ===
    // ☀️ Increase brightness (top left)
    if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 0 && touchPoint.y <= 100) {
      if (brightnessStep < 6) {
        brightnessStep++;
        int brightnessVoltage = map(brightnessStep, 0, 6, 2500, 3300);
        M5.Axp.SetLcdVoltage(brightnessVoltage);
        playSound("displaybrighter5.wav");
      }
    }

    // 🌑 Decrease brightness (bottom left)
    if (touchPoint.x >= 0 && touchPoint.x <= 100 && touchPoint.y >= 140 && touchPoint.y <= 240) {
      if (brightnessStep > 0) {
        brightnessStep--;
        int brightnessVoltage = map(brightnessStep, 0, 6, 2500, 3300);
        M5.Axp.SetLcdVoltage(brightnessVoltage);
        playSound("displaydarker5.wav");
      }
    }
  }


  // === DRAW BRIGHTNESS BAR ===
  const int brightBarHeight = 202;
  const int brightBarWidth = 2;
  const int brightBarX = 1, brightBarY = 230;  // Bottom left corner
  const int brightSteps = 6;
  const uint16_t brightBarColor = WHITE;

  M5.Lcd.fillRect(brightBarX, brightBarY - brightBarHeight, brightBarWidth, brightBarHeight, BLACK);

  int brightFillHeight = (brightnessStep * brightBarHeight) / brightSteps;
  M5.Lcd.fillRect(brightBarX, brightBarY - brightFillHeight, brightBarWidth, brightFillHeight, brightBarColor);

  for (int i = 1; i < brightSteps; ++i) {
    int y = brightBarY - (i * brightBarHeight / brightSteps);
    M5.Lcd.drawFastHLine(brightBarX, y, brightBarWidth, BLACK);
  }

  // Clear previous dose value
  M5.Lcd.fillRect(198, 104, 23, 23, BLACK);

  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  if (now - lastUpdate >= 1000) {
    lastUpdate = now;

    // Disable interrupts to safely read pulse count
    noInterrupts();
    unsigned long count = pulseCount;
    pulseCount = 0;
    interrupts();

    doseRate = count / calibrationFactor;

    static float sumDose = 0;     // Sum of dose rates
    static int countSamples = 0;  // Sample count per column
    sumDose += doseRate;
    countSamples++;

    // Update graph every 225 seconds (3.75 minutes)
    if (countSamples >= 225) {
      float avgDose = sumDose / countSamples;
      drawAverageGraph(avgDose);
      sumDose = 0;
      countSamples = 0;
    }

    drawRateGraph(doseRate);

    if (countSamples > 0) {
      displayValues(doseRate, sumDose / countSamples);  // Show both current and average
      displayValues(doseRate, 0);                       // Show current only (optional)
    }
  }

  float radiation = doseRate;  // Radiation in µSv/h

  // === GPS INITIALIZATION DISPLAY ===
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

  // === SATELLITE TRACKER ===
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

  // === LAST SAVED COORDINATES FROM SD ===
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

  // === BUTTON A: Press & Hold ===
  if (M5.BtnA.isPressed()) {
    if (!buttonAHeld) {
      buttonAPressTime = millis();
      buttonAHeld = true;
    } else if (millis() - buttonAPressTime > 2000) {
      // 🔥 Long press → Show target selection menu
      showTargetSelectionMenu();
      buttonAHeld = false;
    }
  } else if (buttonAHeld && M5.BtnA.wasReleased()) {

    // 🟢 Short press → Save current position
    if (millis() - buttonAPressTime <= 2000) {
      playSound("beep10.wav");

      unsigned long waitStart = millis();
      bool gotNewFix = false;

      M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.setCursor(108, 137);
      M5.Lcd.print("WAITING FOR GPS");

      M5.Lcd.setTextSize(1);
      while (millis() - waitStart < 3000) {
        while (Serial1.available()) {
          gps.encode(Serial1.read());
        }

        if (gps.location.isValid() && gps.location.age() < 5000) {
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

        updateNearestCity(homeLat, homeLon);  // 🌍 Determine city & country

        // 🛡️ Store city & country in local copy before anything changes them
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

        // 💾 Update file for home coordinates (used on startup)
        File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
        if (myFile) {
          myFile.print(homeLat, 6);
          myFile.print(",");
          myFile.println(homeLon, 6);
          myFile.close();

          // ❌ Do NOT call updateNearestCity again here
          displaySavedLocation();
        }

        // 🧾 Debug output
        Serial.printf("📝 Saved: %f,%f → %s, %s\n", homeLat, homeLon, currentCity.c_str(), currentCountry.c_str());

        // ✅ Feedback
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

        delay(500);
        playSound("beep31.wav");
        delay(500);
        playSound("positionsaved.wav");
        delay(500);
        playSound("beep20.wav");
      } else {
        // ❌ No valid fix → error message
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
        M5.Lcd.setCursor(238, 233);
        M5.Lcd.print(">  SELECT  ");
        M5.Lcd.setCursor(300, 233);
        M5.Lcd.print("<");

        playSound("beep04.wav");
        playSound("positionnotsaved.wav");
        delay(500);
      }

      // 🧼 Clear feedback display
      M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    }

    buttonAHeld = false;
  }


  if (M5.BtnB.wasPressed()) {
    delay(50);  // Debounce delay
    playSound("beep10.wav");

    // Clear UI area
    M5.Lcd.fillRect(16, 233, 284, 9, BLACK);
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.drawFastHLine(21, 233, 270, CYAN);
    M5.Lcd.drawFastHLine(21, 239, 270, CYAN);

    // Display "HOME COORDINATES" heading
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(108, 137);
    M5.Lcd.print("HOME COORDINATES");

    // Latitude
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(108, 157);
    M5.Lcd.print("LATI:N");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("51.861120");

    // Longitude
    M5.Lcd.setCursor(108, 177);
    M5.Lcd.print("LONG:E");
    M5.Lcd.setTextSize(1);
    M5.Lcd.print("\xF7 ");
    M5.Lcd.setTextSize(2);
    M5.Lcd.print(" 8.289310");

    // Audio feedback
    delay(500);
    playSound("beep31.wav");
    delay(500);
    playSound("homecoordinatessaved.wav");
    delay(500);
    playSound("beep20.wav");

    // Save hardcoded home coordinates
    File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
    if (myFile) {
      myFile.print(51.861120, 6);
      myFile.print(",");
      myFile.println(8.289310, 6);
      myFile.close();

      homeLat = 51.861120;
      homeLon = 8.289310;

      updateNearestCity(homeLat, homeLon);

      // Refresh home location UI line
      M5.Lcd.setTextSize(1);
      displaySavedLocation();
      M5.Lcd.setCursor(141, 233);
        M5.Lcd.print("> HOME POS <");
        M5.Lcd.setCursor(238, 233);
        M5.Lcd.print(">  SELECT  ");
        M5.Lcd.setCursor(300, 233);
        M5.Lcd.print("<");

      // Clear info box
      M5.Lcd.fillRoundRect(100, 136, 212, 94, 4, BLACK);
    } else {
      Serial.println("ERROR WRITE FILE");
    }
  }


  if (M5.BtnC.wasPressed()) {
    delay(50);  // 50 ms debounce
    playSound("beep10.wav");
    playSound("selectyourdestination.wav");

    // Draw background box for the menu
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(1);

    int topY = 140;
    int spacingY = 13;
    int boxX = 105;

    // Display the list of available locations
    for (int i = 0; i < numLocations; i++) {
      int y = topY + i * spacingY;
      M5.Lcd.setCursor(boxX, y);
      M5.Lcd.print(seedBanks[i].name);
    }

    // Wait for touch input (max 10 seconds)
    unsigned long startTime = millis();
    bool selected = false;
    while ((millis() - startTime < 10000) && !selected) {
      M5.update();
      M5.Touch.update();

      if (M5.Touch.ispressed()) {
        delay(50);  // debounce
        TouchPoint_t p = M5.Touch.getPressPoint();

        for (int i = 0; i < numLocations; i++) {
          int y = topY + i * spacingY;

          if (p.y > y && p.y < (y + spacingY)) {
            playSound("beep20.wav");

            // Highlight selected item
            M5.Lcd.fillRect(boxX - 3, y - 3, 210, spacingY, BLUE);
            M5.Lcd.setTextColor(WHITE, BLUE);
            M5.Lcd.setCursor(boxX, y);
            M5.Lcd.print(seedBanks[i].name);

            delay(300);

            float lat = seedBanks[i].lat;
            float lon = seedBanks[i].lon;

            // Save selected coordinates to SD file
            File myFile = SD.open("/home_coordinates.txt", FILE_WRITE);
            if (myFile) {
              myFile.printf("%.6f,%.6f\n", lat, lon);
              myFile.close();

              // Update internal coordinates and UI
              homeLat = lat;
              homeLon = lon;
              updateNearestCity(homeLat, homeLon);
              displaySavedLocation();

              // Update footer with selected name
              M5.Lcd.fillRect(16, 233, 284, 9, BLACK);
              M5.Lcd.drawFastHLine(21, 233, 268, CYAN);
              M5.Lcd.drawFastHLine(21, 239, 268, CYAN);
              M5.Lcd.setCursor(20, 233);
              M5.Lcd.setTextColor(YELLOW, BLACK);
              M5.Lcd.print(seedBanks[i].name);
              M5.Lcd.setTextColor(CYAN, BLACK);
              M5.Lcd.print(" <");
              M5.Lcd.setCursor(238, 233);
              M5.Lcd.print(">  SELECT  ");
              M5.Lcd.setCursor(300, 233);
              M5.Lcd.print("<");
              playSound(seedBanks[i].soundFile);
              delay(250);
              selected = true;
              delay(1000);
            } else {
              Serial.println("❌ ERROR: Could not write home_coordinates.txt");
            }

            break;  // Exit loop after selection
          }
        }
      }
    }

    // Clear the selection menu
    M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  }

  // Calculate the distance from current location to home coordinates
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

  // === Satellite Display ===
  activeSatellites = 0;
  for (int i = 0; i < MAX_SATELLITES; ++i) {
    if (sats[i].active) {
      activeSatellites++;
    }
  }

  // Draw radar and silent mode icon
  drawRadarDisplay();
  drawSilentIcon();

  // === Direction Pointer with Compass Arrow ===Keeps direction stable when moving slowly

  static float lastValidCourse = 0;

  float currentCourse = gps.course.deg();
  float currentSpeed = gps.speed.kmph();

  // Only update course if speed is above threshold
  if (gps.course.isValid() && gps.speed.isValid() && currentSpeed > 5.0) {
    lastValidCourse = currentCourse;
  }

  // Calculate relative direction to home
  float relCourse = courseToHome - lastValidCourse;
  if (relCourse < 0) relCourse += 360;
  if (relCourse >= 360) relCourse -= 360;

// --------- Kompass einlesen (vor Anzeige, z. B. in loop()) ----------
sVector_t mag = compass.readRaw();  // Magnetfeld auslesen
float headingMag = atan2((float)mag.YAxis, (float)mag.XAxis) * 180.0 / PI;
if (headingMag < 0) headingMag += 360.0;

// Glättung für magnetischen Kurs (optional)
static float filteredHeadingMag = 0;
float alpha = 0.1;
filteredHeadingMag = alpha * headingMag + (1 - alpha) * filteredHeadingMag;

// Kursquelle wählen
float effectiveCourse = (currentSpeed <= 3.0) ? filteredHeadingMag : relCourse;
uint16_t arrowColor = (currentSpeed <= 3.0) ? ORANGE : YELLOW;
float angle = effectiveCourse * rad_fac;


// ---------- Anzeige ----------

static bool compassInitDone = false;
static float lastAngle = -999;

if (distanceToHome < 10) {
  // ---------- ECHTER KOMPASS BEI ZIEL ----------

  // Magnetfeld lesen
  sVector_t mag = compass.readRaw();  // (X, Y, Z)
  float heading = atan2((float)mag.YAxis, (float)mag.XAxis) * 180.0 / PI;
  if (heading < 0) heading += 360.0;

  // Glättung (optional)
  static float smoothedHeading = heading;
  float alpha = 0.1;
  smoothedHeading = alpha * heading + (1 - alpha) * smoothedHeading;
  
  // ⬇️ Korrekturwinkel (Offset) anwenden
  float compassCorrection = 0.0;  // <- HIER ANPASSEN! z. B. -10° oder +5°
  float correctedHeading = smoothedHeading + compassCorrection;
  if (correctedHeading < 0) correctedHeading += 360;
  if (correctedHeading >= 360) correctedHeading -= 360;

  float compassAngle = correctedHeading * DEG_TO_RAD;

    // Zentrum
    int x0 = 52;
    int y0 = 184;
    int len = 26;               // Länge der Nadel
    int sideOffset = 3;         // Breite der Nadel

   // // Winkel in Radiant
   // float compassAngle = smoothedHeading * rad_fac;

// Nur aktualisieren bei signifikanter Änderung
  if (abs(smoothedHeading - lastAngle) > 1.0) {
    lastAngle = smoothedHeading;
   
    // Nadelkoordinaten berechnen
    int x1 = x0 + sin(compassAngle) * len;         // Nordspitze
    int y1 = y0 - cos(compassAngle) * len;
    int x2 = x0 - sin(compassAngle) * len;         // Südspitze
    int y2 = y0 + cos(compassAngle) * len;
    int xL = x0 + cos(compassAngle) * sideOffset;
    int yL = y0 + sin(compassAngle) * sideOffset;
    int xR = x0 - cos(compassAngle) * sideOffset;
    int yR = y0 - sin(compassAngle) * sideOffset;

    // Hintergrund (nur einmal zeichnen)
    if (!compassInitDone) {
    //  M5.Lcd.fillCircle(x0, y0, 36, BLACK);           // Nur einmal löschen
    M5.Lcd.fillCircle(x0, y0, 28, BLACK);
    M5.Lcd.drawCircle(x0, y0, 30, DARKGREEN);
  
  // Teilstriche für N, E, S, W (alle 90°) – groß & beschriftet
  for (int i = 0; i < 360; i += 90) {
    float rad = i * DEG_TO_RAD;
    int x1 = x0 + cos(rad) * 30;
    int y1 = y0 + sin(rad) * 30;
    int x2 = x0 + cos(rad) * 35;
    int y2 = y0 + sin(rad) * 35;
    M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
  }

  // Mittelgroße Striche (alle 45°, außer den bereits gezeichneten 90°)
  for (int i = 45; i < 360; i += 90) {
    float rad = i * DEG_TO_RAD;
    int x1 = x0 + cos(rad) * 30;
    int y1 = y0 + sin(rad) * 30;
    int x2 = x0 + cos(rad) * 33;
    int y2 = y0 + sin(rad) * 33;
    M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
  }

  // Kleine Teilstriche (alle 15°, außer Vielfache von 45°)
  for (int i = 0; i < 360; i += 15) {
    if (i % 45 == 0) continue;  // bereits gezeichnet
    float rad = i * DEG_TO_RAD;
    int x1 = x0 + cos(rad) * 30;
    int y1 = y0 + sin(rad) * 30;
    int x2 = x0 + cos(rad) * 31;
    int y2 = y0 + sin(rad) * 31;
    M5.Lcd.drawLine(x1, y1, x2, y2, GREEN);
  }
      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.setCursor(50, 140);
      M5.Lcd.print("N");
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.setCursor(11, 181);
      M5.Lcd.print("W");
      M5.Lcd.setCursor(89, 181);
      M5.Lcd.print("E");
      M5.Lcd.setCursor(50, 214);
      M5.Lcd.print("S");
      
      compassInitDone = true;
    }
else {
      //M5.Lcd.fillCircle(x0, y0, 28, BLACK);
    M5.Lcd.fillCircle(x0, y0, 28, BLACK);
    }
    // Kompassnadel zeichnen (ungeändert)
    M5.Lcd.fillTriangle(x1, y1, xL, yL, xR, yR, RED);    // NORD-Dreieck
    M5.Lcd.fillTriangle(x2, y2, xL, yL, xR, yR, CYAN);  // SÜD-Dreieck
  }

} else {

// ----------- Richtungspfeil-Modus ------------
compassInitDone = false;  // erzwinge Neuzeichnung bei Rückkehr

// === NEU: Kurs glätten ===
static float smoothedCourse = 0;
float alpha = 0.2;
smoothedCourse = alpha * effectiveCourse + (1 - alpha) * smoothedCourse;

// === NEU: Nur neu zeichnen bei signifikanter Änderung ===
static float lastDrawnCourse = -999;
if (abs(smoothedCourse - lastDrawnCourse) < 2.0) return;
lastDrawnCourse = smoothedCourse;

float angle = smoothedCourse * DEG_TO_RAD;
float rad_fac = DEG_TO_RAD;  // falls extern nicht definiert

// === Anzeige-Bereich löschen ===
//M5.Lcd.fillCircle(52, 184, 36, BLACK);
M5.Lcd.fillCircle(52, 184, 28, BLACK);
int x0 = 52;
int y0 = 184;

M5.Lcd.drawCircle(x0, y0, 30, DARKGREEN);

// Teilstriche für N, E, S, W (alle 90°)
for (int i = 0; i < 360; i += 90) {
  float rad = i * DEG_TO_RAD;
  int x1 = x0 + cos(rad) * 30;
  int y1 = y0 + sin(rad) * 30;
  int x2 = x0 + cos(rad) * 35;
  int y2 = y0 + sin(rad) * 35;
  M5.Lcd.drawLine(x1, y1, x2, y2, YELLOW);
}

// Mittelgroße Striche (alle 45° außer 90er)
for (int i = 45; i < 360; i += 90) {
  float rad = i * DEG_TO_RAD;
  int x1 = x0 + cos(rad) * 30;
  int y1 = y0 + sin(rad) * 30;
  int x2 = x0 + cos(rad) * 33;
  int y2 = y0 + sin(rad) * 33;
  M5.Lcd.drawLine(x1, y1, x2, y2, ORANGE);
}

// Kleine Teilstriche (alle 15°, außer Vielfache von 45°)
for (int i = 0; i < 360; i += 15) {
  if (i % 45 == 0) continue;
  float rad = i * DEG_TO_RAD;
  int x1 = x0 + cos(rad) * 30;
  int y1 = y0 + sin(rad) * 30;
  int x2 = x0 + cos(rad) * 31;
  int y2 = y0 + sin(rad) * 31;
  M5.Lcd.drawLine(x1, y1, x2, y2, ORANGE);
}

// Richtungspfeil berechnen
int x = sin(angle) * 26;
int y = cos(angle) * 26;
int q = sin(angle);
int r = cos(angle);
int x1 = 53 + x;
int y1 = 184 - y;
int q1 = 53 + q;
int r1 = 184 - r;

// Pfeilflügel 1
double arrowAngle1 = smoothedCourse + 200;
x = sin(arrowAngle1 * rad_fac) * 48;
y = cos(arrowAngle1 * rad_fac) * 48;
q = sin(arrowAngle1 * rad_fac) * 48;
r = cos(arrowAngle1 * rad_fac) * 48;
int x2 = x1 + x;
int y2 = y1 - y;
int q2 = q1 + q;
int r2 = r1 - r;

if ((gps.location.lat() != 0) || (gps.location.lng() != 0)) {
  M5.Lcd.drawLine(x1, y1, x2, y2, arrowColor);
  M5.Lcd.drawLine(q1, r1, x2, y2, arrowColor);
} else {
  // Kein gültiger GPS-Fix → Kompassanzeige
  M5.Lcd.drawCircle(52, 184, 30, NAVY);
  M5.Lcd.drawCircle(52, 184, 20, 0x00AF);
  M5.Lcd.drawCircle(52, 184, 10, BLUE);
  M5.Lcd.drawCircle(52, 184, 2, BLUE);
  M5.Lcd.drawFastVLine(52, 200, 20, DARKCYAN);
  M5.Lcd.drawFastVLine(52, 149, 20, DARKCYAN);
  M5.Lcd.drawFastHLine(68, 184, 20, DARKCYAN);
  M5.Lcd.drawFastHLine(17, 184, 20, DARKCYAN);
}

// Pfeilflügel 2
double arrowAngle2 = smoothedCourse + 160;
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

// Kompassbeschriftung
M5.Lcd.setTextSize(1);
M5.Lcd.setTextColor(RED, BLACK);
M5.Lcd.setCursor(50, 140);
M5.Lcd.print("N");
M5.Lcd.setTextColor(GREEN, BLACK);
M5.Lcd.setCursor(10, 181);
M5.Lcd.print("W");
M5.Lcd.setCursor(89, 181);
M5.Lcd.print("E");
M5.Lcd.setCursor(50, 214);
      M5.Lcd.print("S");
}

  // === Periodic GPS ping every 30 seconds (if fix available) ===
  if (gps.location.isValid() && millis() - lastGpsPingTime >= gpsPingInterval) {
    //playSound("sonar29.wav");  // oder ein anderer kurzer Ton
    if (!silentMode) playSound("sonar05.wav");
    //if (!silentMode) playSound("sonar34.wav");
    Serial.println("📍 GPS-Ping (Fix OK)");
    lastGpsPingTime = millis();
  }
  // Show satellite status icon
  const char *satIcon = gps.location.isValid() ? "/graphics/saton.png" : "/graphics/satoff.png";
  M5.Lcd.drawPngFile(SD, satIcon, 100, 98);

  // === Show direction text if distance > 10 meters ===
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
    // Clear coordinate display area
    M5.Lcd.fillRoundRect(13, 140, 32, 12, 1, BLACK);
    M5.Lcd.fillRoundRect(60, 140, 32, 12, 1, BLACK);
  }

  // === Timezone offset (Central European Time) ===
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
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(CO);
    M5.Lcd.print(" ppm");
  } else if (CO > 10) {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("CO :");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(CO);
    M5.Lcd.print(" ppm");
  } else {
    M5.Lcd.drawRoundRect(12, 77, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 79);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
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
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(NH3);
    M5.Lcd.print(" ppm");
  } else if (NH3 > 5) {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 90);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NH3:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(NH3);
    M5.Lcd.print(" ppm");
  } else {
    M5.Lcd.drawRoundRect(12, 88, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 90);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
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
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(NO2);
    M5.Lcd.print(" ppm");
  } else if (NO2 > 2) {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 101);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("NO2:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(NO2);
    M5.Lcd.print(" ppm");
  } else {
    M5.Lcd.drawRoundRect(12, 99, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 101);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
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
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.print(EMF);
    M5.Lcd.print("uT");
  } else if (EMF > 30) {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, ORANGE);
    M5.Lcd.setCursor(14, 112);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
    M5.Lcd.print("EMF:");
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print(EMF);
    M5.Lcd.print(" uT");
  } else {
    M5.Lcd.drawRoundRect(12, 110, 82, 11, 2, 0x00AF);
    M5.Lcd.setCursor(14, 112);
    M5.Lcd.setTextColor(DARKCYAN, BLACK);
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

  if (!isCharging) {
    // ==== EINMALIGE Warnsounds bei bestimmten Akkuständen ====
    static int lastNotifiedStep = 101;  // Anfang: Noch kein Alarm gespielt
    static bool firstBatteryCheck = true;
    int warnSteps[] = { 50, 40, 30, 20, 10, 5 };
    const int numWarnSteps = sizeof(warnSteps) / sizeof(warnSteps[0]);

    if (firstBatteryCheck) {
      // Beim ersten Mal NICHTS abspielen, sondern aktuellen Zustand merken
      for (int i = 0; i < numWarnSteps; i++) {
        if (batPercentage <= warnSteps[i]) {
          lastNotifiedStep = warnSteps[i];
          break;
        }
      }
      firstBatteryCheck = false;
    } else {
      for (int i = 0; i < numWarnSteps; i++) {
        if (batPercentage <= warnSteps[i] && lastNotifiedStep > warnSteps[i]) {
          char filename[20];
          sprintf(filename, "battery%d.wav", warnSteps[i]);  // z. B. battery30.wav
          playSound(filename);
          lastNotifiedStep = warnSteps[i];
          break;  // Nur einen Ton pro Schleife
        }
      }
    }
  }

  // ==== Batterie-Farbe ====
  uint16_t batteryColor = (batPercentage < 20) ? RED : (batPercentage < 50 ? YELLOW : GREEN);

  // ==== Akku-Füllstand für Grafik berechnen ====
  int fillWidth = map(batPercentage, 0, 100, 0, 31);

  // ==== Nur neu zeichnen, wenn sich die Breite geändert hat ====
  static int lastFillWidth = -1;
  if (fillWidth != lastFillWidth) {
    // Rahmen und Füllung nur bei Änderung
    M5.Lcd.fillRoundRect(276, 8, 33, 12, 2, BLACK);  // Hintergrund löschen
    M5.Lcd.drawRoundRect(275, 7, 35, 14, 3, GREENYELLOW);
    M5.Lcd.drawRoundRect(309, 11, 4, 6, 2, GREENYELLOW);

    M5.Lcd.fillRoundRect(277, 9, fillWidth, 10, 2, batteryColor);  // Neue Füllung

    lastFillWidth = fillWidth;  // Zustand merken
  }

  if (isCharging) {
    M5.Lcd.fillTriangle(291, 14, 288, 14, 295, 7, RED);   // Pfeil oben
    M5.Lcd.fillTriangle(296, 13, 291, 13, 289, 20, RED);  // Pfeil unten
  }

  // ==== Textanzeige ====
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(240, 5);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.print(batVoltage, 2);
  M5.Lcd.println("V");

  M5.Lcd.setCursor(240, 14);
  M5.Lcd.setTextColor(batteryColor, BLACK);
  M5.Lcd.print(batPercentage, 0);
  M5.Lcd.print("% ");


  // ==== GPS-Anzeige mit Signalprüfung ====

  // 📡 LATITUDE
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(104, 138);
  M5.Lcd.print("LATI:");

  if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites >= 3) {
  //if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 1) {
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.fillRect(301, 138, 14, 14, BLACK);
    M5.Lcd.setCursor(164, 138);
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
  M5.Lcd.setTextColor(DARKCYAN, BLACK);
  M5.Lcd.setCursor(104, 157);
  M5.Lcd.print("LONG:");

  if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites >= 3) {
  //if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 1) {
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.fillRect(301, 157, 14, 14, BLACK);
    M5.Lcd.setCursor(164, 157);
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
  // ANTI ARTEFACT
  M5.Lcd.drawLine(315, 138, 315, 228, CYAN);
  M5.Lcd.drawLine(319, 138, 319, 228, CYAN);
  
  // ⛰ ALTITUDE 
M5.Lcd.setTextColor(DARKCYAN, BLACK);
M5.Lcd.setCursor(104, 176);
M5.Lcd.print("ALTI:");

static String lastAltitude = "";
String altDisplay = "---";

if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 1) {
  float altitude = gps.altitude.meters();
  altDisplay = String(altitude, 2) + " m";  // Kein dtostrf → kein Leerzeichen-Vorspann!
}

if (altDisplay != lastAltitude) {
  M5.Lcd.fillRect(163, 176, 150, 16, BLACK);
  M5.Lcd.setCursor(164, 176);
  M5.Lcd.setTextColor(altDisplay == "---" ? RED : CYAN, BLACK);
  M5.Lcd.print(altDisplay);
  lastAltitude = altDisplay;
}

  // 🏃 SPEED
M5.Lcd.setTextColor(DARKCYAN, BLACK);
M5.Lcd.setCursor(104, 196);
M5.Lcd.print("SPED:");

static String lastSpeed = "";
String speedDisplay = "---";

if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 1) {
  float speed = gps.speed.kmph();
  speedDisplay = String(speed, 1) + " km/h";  // Kein Leerzeichen vorne
}

if (speedDisplay != lastSpeed) {
  M5.Lcd.fillRect(163, 196, 150, 16, BLACK);
  M5.Lcd.setCursor(164, 196);
  M5.Lcd.setTextColor(speedDisplay == "---" ? RED : CYAN, BLACK);
  M5.Lcd.print(speedDisplay);
  lastSpeed = speedDisplay;
}
static String lastTimeStr = "";

// 🎯 DESTINATION
M5.Lcd.setTextColor(DARKCYAN, BLACK);
M5.Lcd.setCursor(104, 215);
M5.Lcd.print("DEST:");

static String lastDest = "";
String distanceStr = "---";

if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 1) {
  double distance = TinyGPSPlus::distanceBetween(
    gps.location.lat(), gps.location.lng(), homeLat, homeLon);

  if (distance < 1000) {
    distanceStr = String((int)distance) + " m";
  } else {
    distanceStr = String(distance / 1000.0, 2) + " km";
  }
}
if (distanceStr != lastDest) {
  M5.Lcd.fillRect(163, 215, 150, 16, BLACK);
  M5.Lcd.setCursor(164, 215);
  M5.Lcd.setTextColor(distanceStr == "---" ? RED : CYAN, BLACK);
  M5.Lcd.print(distanceStr);
  lastDest = distanceStr;
}

// --- ZEIT-BERECHNUNG ---
M5.Lcd.setTextSize(1);
M5.Lcd.setCursor(16, 222);

float distance_km = TinyGPSPlus::distanceBetween(
  gps.location.lat(), gps.location.lng(), homeLat, homeLon) / 1000;
float speed_kmh = 3;
float hours_per_day = 8;
float speed_per_day = speed_kmh * hours_per_day;

float travel_time_hours = distance_km / speed_kmh;
float travel_time_days = distance_km / speed_per_day;
float travel_time_minutes = travel_time_hours * 60;

String timeStr;

if (travel_time_days > 1.5) {
  timeStr = String(travel_time_days, 1) + " DAYS";
} else if (travel_time_days >= 1) {
  timeStr = String(travel_time_days, 1) + " DAY";
} else if (travel_time_hours >= 1.5) {
  timeStr = String(travel_time_hours, 1) + " HOURS";
} else if (travel_time_hours >= 1) {
  timeStr = String(travel_time_hours, 1) + " HOUR";
} else if (travel_time_minutes > 1.5) {
  timeStr = String((int)travel_time_minutes) + " MINUTES";
} else if (travel_time_minutes >= 0.5) {
  timeStr = String((int)travel_time_minutes) + " MINUTE";
  hasArrived = false;
} else {
  if (!hasArrived) {
    playSound("destination.wav");
    Serial.println("🎯 DESTINATION erreicht!");
    hasArrived = true;
  }
  timeStr = " DESTINATION";
}

// Nur aktualisieren, wenn sich die Ausgabe ändert
if (timeStr != lastTimeStr) {
  M5.Lcd.fillRect(12, 222, 80, 9, BLACK);
  M5.Lcd.setCursor(16, 222);
  M5.Lcd.setTextColor((timeStr == " DESTINATION") ? GREEN : YELLOW, BLACK);
  M5.Lcd.print(timeStr);
  lastTimeStr = timeStr;
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
  if (gps.location.isValid() && gps.charsProcessed() > 100 && activeSatellites > 1) {
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

void drawSilentIcon() {
  if (!uiReady) return;

  static bool lastDrawnSilentMode = -1;  // Sicher initial unterschiedlich

  int iconX = 161 - 11;
  int iconY = 76 - 11;

  if (silentMode != lastDrawnSilentMode) {
    // Nur den Bereich des Icons löschen
    M5.Lcd.fillRect(iconX, iconY, 22, 22, BLACK);
    const char *iconPath = silentMode ? "/graphics/silent_on.png" : "/graphics/silent_off.png";
    M5.Lcd.drawPngFile(SD, iconPath, iconX, iconY);

    // Sound abspielen beim Umschalten
    if (silentMode) {
      playSound("warningoff.wav");
    } else {
      playSound("warningon.wav");
    }

    lastDrawnSilentMode = silentMode;
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
  playSound("beep10.wav");
  playSound("saveddestinations.wav");

  M5.Lcd.fillRoundRect(100, 136, 212, 96, 4, BLACK);
  M5.Lcd.setTextSize(1);

  int topY = 140;
  int spacingY = 13;
  int boxX = 105;
  M5.Lcd.setTextWrap(false);  // Optional vor Schleife setzen*************************************

  for (int i = 0; i < targetList.size() && i < maxTargets; i++) {
    int y = topY + i * spacingY;
    M5.Lcd.setCursor(boxX, y);

    if (i == 0) {
      M5.Lcd.setTextColor(YELLOW, BLACK);
    } else {
      M5.Lcd.setTextColor(GREEN, BLACK);
    }

    if (targetList[i].city.length() > 0) {
      double distToHome = distanceBetween(latHomeFixed, lonHomeFixed, targetList[i].lat, targetList[i].lon);
      String countryShort = targetList[i].country.substring(0, 2);

      // Entfernung formatieren
      String distStr;
      if (distToHome < 1000.0) {
        distStr = String((int)distToHome) + "m";
      } else {
        distStr = String(distToHome / 1000.0, 2) + "km";
      }

      // Ausgabe
      M5.Lcd.printf("%s, %s  %s",
                    targetList[i].city.c_str(),
                    countryShort.c_str(),
                    distStr.c_str());
    } else {
      M5.Lcd.print("...");
    }
  }
  M5.Lcd.setTextWrap(true);  // Optional danach wieder anschalten***********************************
  unsigned long startTime = millis();
  bool selected = false;

  while ((millis() - startTime < 10000) && !selected) {
    M5.update();
    M5.Touch.update();

    if (M5.Touch.ispressed()) {
      delay(50);  // 50 ms Entprellen
      TouchPoint_t p = M5.Touch.getPressPoint();

      for (int i = 0; i < targetList.size() && i < maxTargets; i++) {
        int y = topY + i * spacingY;
        if (p.y > y && p.y < (y + spacingY)) {
          playSound("beep20.wav");

          M5.Lcd.fillRect(boxX - 3, y - 3, 210, spacingY, BLUE);
          M5.Lcd.setTextColor(WHITE, BLUE);
          M5.Lcd.setCursor(boxX, y);

          // Entfernung berechnen
          double distToHome = distanceBetween(latHomeFixed, lonHomeFixed, targetList[i].lat, targetList[i].lon);
          String countryShort = targetList[i].country.substring(0, 2);
          String distStr;

          if (distToHome < 1000.0) {
            distStr = String((int)distToHome) + "m";
          } else {
            distStr = String(distToHome / 1000.0, 2) + "km";
          }

          M5.Lcd.printf("%s, %s  %s", targetList[i].city.c_str(), countryShort.c_str(), distStr.c_str());

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
        M5.Lcd.print(">  SELECT  ");
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
