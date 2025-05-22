// -------ALL PROGRAM ESP32-S3--------//

#include <WiFi.h>
#include <HTTPClient.h>
#include <PZEM004Tv30.h>
#include <max6675.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PCF8574.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// PZEM-004T Calibration (scale factors, default 1.0 = no correction)
float PZEM_VOLTAGE_SCALE = 1.0; // Adjust between 0.9 to 1.1
float PZEM_CURRENT_SCALE = 1.0; // Adjust between 0.9 to 1.1
float PZEM_POWER_SCALE = 1.0;   // Adjust between 0.9 to 1.1

// Temperature Calibration (offset, default 0.0 = no correction)
float TEMP_OFFSET = -3.5; // Adjust between -5.0 to +5.0 °C

// EEPROM Configuration
#define EEPROM_SIZE 16
#define EEPROM_ADDR_SETPOINT 0
#define EEPROM_ADDR_DIMMER_MODE 4

// I2C Addresses
#define I2C_ADDR_KEYPAD 0x20
#define I2C_ADDR_OLED 0x3C
#define I2C_ADDR_LCD 0x27

// Pin Definitions - MAX6675 (Thermocouple)
#define SO_PIN 21
#define CS_PIN_MAX6675 47
#define SCK_PIN_MAX6675 48

// Pin Definitions - RPM Sensor
#define SENSOR_PIN 20

// Pin Definitions - SD Card (SPI)
#define SD_CS_PIN 10
#define SD_SCK_PIN 12
#define SD_MISO_PIN 13
#define SD_MOSI_PIN 11

// Pin Definitions - AC Dimmer
#define ZERO_CROSS_PIN 1
#define DIMMER_PWM_PIN 2

// Pin Definitions - Relay (LED 12V Indicators)
#define RELAY1 41 // Database status
#define RELAY2 40 // WiFi connection
#define RELAY3 39 // Error indicator

// Pin Definitions - I2C
#define I2C_SDA_LCD_KEYPAD 17
#define I2C_SCL_LCD_KEYPAD 18
#define I2C_SDA_OLED 6
#define I2C_SCL_OLED 7

// Pin Definitions - PZEM-004T
#define PZEM_RX 15
#define PZEM_TX 16

// AC Dimmer Configuration
const int MAX_DIMMER_VALUE = 100; // 100% power
const int MIN_DIMMER_VALUE = 0;   // 0% power
const float PROPORTIONAL_START = 0.9; // 90% of setpoint
const float FULL_OFF_OFFSET = 5.0; // Full off at setpoint - 5°C
const float HYSTERESIS = 1.0;     // ±1°C for stability
bool dimmerActive = false;
int dimmerValue = MIN_DIMMER_VALUE;

// Manual Dimmer Variables
hw_timer_t* dimmerTimer = NULL;
volatile bool zeroCrossDetected = false;
volatile uint32_t dimmerDelay = 0;
volatile uint32_t lastZeroCrossTime = 0;
volatile bool dimmerDebugFlag = false;
const uint32_t HALF_CYCLE_US = 10000; // 10ms untuk 50Hz
const uint32_t PULSE_WIDTH_US = 100;  // Lebar pulsa 100us untuk TRIAC
const bool INVERT_TRIAC_LOGIC = false; // Ubah ke true jika modul PSM memerlukan logika terbalik

void IRAM_ATTR zeroCrossISR() {
  zeroCrossDetected = true;
  dimmerDelay = 0;
  lastZeroCrossTime = micros();
  dimmerDebugFlag = true; // Untuk debugging
}

void IRAM_ATTR dimmerTimerISR() {
  if (zeroCrossDetected && dimmerValue > 0) {
    dimmerDelay++;
    // Map dimmerValue (0-100) ke penundaan (0-90% dari setengah siklus)
    uint32_t delayThreshold = map(dimmerValue, 0, 100, HALF_CYCLE_US - 500, 100); // 100us hingga 9500us
    if (dimmerDelay * 20 >= delayThreshold / 20) { // Timer 50kHz, 20us per tick
      digitalWrite(DIMMER_PWM_PIN, INVERT_TRIAC_LOGIC ? LOW : HIGH);
      delayMicroseconds(PULSE_WIDTH_US);
      digitalWrite(DIMMER_PWM_PIN, INVERT_TRIAC_LOGIC ? HIGH : LOW);
      zeroCrossDetected = false;
    }
  }
}

void setupDimmer() {
  pinMode(ZERO_CROSS_PIN, INPUT_PULLUP);
  pinMode(DIMMER_PWM_PIN, OUTPUT);
  digitalWrite(DIMMER_PWM_PIN, INVERT_TRIAC_LOGIC ? HIGH : LOW); // Status awal TRIAC mati
  attachInterrupt(digitalPinToInterrupt(ZERO_CROSS_PIN), zeroCrossISR, RISING);
  dimmerTimer = timerBegin(50000); // 50kHz
  timerAttachInterrupt(dimmerTimer, &dimmerTimerISR);
  timerWrite(dimmerTimer, 0); // Reset timer
  timerStart(dimmerTimer);
  Serial.println("Dimmer timer initialized at 50kHz, 20us interval");
}

void setDimmerPower(int power) {
  if (power < MIN_DIMMER_VALUE) power = MIN_DIMMER_VALUE;
  if (power > MAX_DIMMER_VALUE) power = MAX_DIMMER_VALUE;
  dimmerValue = power;
  Serial.printf("Dimmer power set: %d%%\n", power);
}

// Relay Configuration
const unsigned long RELAY1_PULSE = 100;

// RPM Configuration
volatile unsigned int pulseCount = 0;
const int lubangPerPutaran = 50;

// Keypad Configuration
const char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
const byte rowPins[4] = {0, 1, 2, 3};
const byte colPins[4] = {4, 5, 6, 7};

// PZEM Configuration
PZEM004Tv30 pzem(Serial2, PZEM_RX, PZEM_TX);

// MAX6675
MAX6675 thermocouple(SCK_PIN_MAX6675, CS_PIN_MAX6675, SO_PIN);

// I2C for OLED
TwoWire WireOLED = TwoWire(1);

// LCD I2C
LiquidCrystal_I2C lcd(I2C_ADDR_LCD, 16, 2);

// PCF8574 for Keypad
PCF8574 pcf(I2C_ADDR_KEYPAD);

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &WireOLED, OLED_RESET);
bool oledInitialized = false;

// SPI for SD Card
SPIClass spiSD(HSPI);

// WiFi Configuration
const char* ssid = "speedjos";
const char* password = "zidanspeed19";

// Server URLs
const String PZEM_SERVER_URL = "http://192.168.165.151:9056/api/pzem/";
const String TEMP_SERVER_URL = "http://192.168.165.151:9056/api/suhu/";
const String RPM_SERVER_URL = "http://192.168.165.151:9056/api/rpm/";

// Data Structures
struct PzemData {
  float voltage, current, power, energy, frequency, pf, va, var;
  bool sensorDetected;
};

struct TempData {
  float temperature;
  bool sensorDetected;
};

struct RpmData {
  float rpm;
  bool sensorDetected;
};

// Parameter Thresholds untuk mendeteksi error
const float VOLT_MIN = 210.0, VOLT_MAX = 240.0;
const float CURR_MIN = 0.1, CURR_MAX = 7.5;
const float POWER_MIN = 0.1, POWER_MAX = 1800.0;
const float TEMP_MIN = 0.1, TEMP_MAX = 260;
const float RPM_MIN = 0.1, RPM_MAX = 30.0;

// Global Variables
PzemData pzemData;
TempData tempData;
RpmData rpmData;
float tempSetpoint = 0.0;
String setpointInput = "";
String dataNumberInput = "";
String dateInput = "";
bool isSettingSetpoint = false;
bool isSettingDataNumber = false;
bool isSettingDate = false;
bool isLogging = false;
bool sdInitialized = false;
unsigned long startTime;
unsigned long lastPzemPostTime = 0, lastTempPostTime = 0, lastRpmPostTime = 0;
unsigned long lastDisplayUpdateTime = 0, lastKeyDisplayTime = 0, lastRelay1Pulse = 0;
unsigned long lastOledPageTime = 0, lastLogTime = 0;
const unsigned long PZEM_POST_INTERVAL = 500;
const unsigned long TEMP_POST_INTERVAL = 500;
const unsigned long RPM_POST_INTERVAL = 200;
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000;
const unsigned long KEY_DISPLAY_DURATION = 500;
const unsigned long OLED_PAGE_INTERVAL = 3000;
const unsigned long LOG_INTERVAL = 1000;
bool relay1Active = false;
int oledPage = 0;
const unsigned long SKRIPSI_DISPLAY_DURATION = 3000;
unsigned long lastSkripsiDisplayTime = 0;
bool isDisplayingSkripsi = false;
String currentCsvFile = "";
unsigned long sequenceNumber = 1;
unsigned long lastSentSequence = 0;

// Function Declarations
void connectWiFi();
void readPzemData();
void readTempData();
void readRpmData();
void printPzemData();
void printTempData();
void printRpmData();
void sendPzemToServer();
void sendTempToServer();
void sendRpmToServer();
void logToSD();
void syncPendingData();
bool checkErrors();
void controlDimmer();
void controlRelays();
void handleKeypad();
void updateLcdDisplay();
void updateOledDisplay();
void resetSystem();
char readKeypad();
float zeroIfNan(float value);
void IRAM_ATTR countPulse();
void loadEEPROM();
void saveEEPROM();
void configureNewCsvFile();

void setup() {
  Serial.begin(115200);

  // Initialize AC Dimmer
  setupDimmer();
  setDimmerPower(MIN_DIMMER_VALUE);
  Serial.println("AC Dimmer initialized");

  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadEEPROM();

  // Initialize pins
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);

  // Initialize RPM sensor
  pinMode(SENSOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), countPulse, FALLING);

  startTime = millis();

  // Connect to WiFi
  connectWiFi();

  // Initialize Serial2 for PZEM
  Serial2.begin(9600, SERIAL_8N1, PZEM_RX, PZEM_TX);

  // Initialize I2C for LCD & Keypad (PCF)
  Wire.begin(I2C_SDA_LCD_KEYPAD, I2C_SCL_LCD_KEYPAD);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Loading System...");

  // Initialize PCF8574 for Keypad
  pcf.begin();
  pcf.write8(0xFF);

  // Initialize I2C for OLED
  Serial.println("Initializing I2C for OLED...");
  WireOLED.begin(I2C_SDA_OLED, I2C_SCL_OLED);

  if (!display.begin(I2C_ADDR_OLED, true)) {
    Serial.println("OLED failed at 0x3C! Trying 0x3D...");
    if (!display.begin(0x3D, true)) {
      Serial.println("OLED failed at both addresses!");
      oledInitialized = false;
    } else {
      Serial.println("OLED initialized at 0x3D");
      oledInitialized = true;
    }
  } else {
    Serial.println("OLED initialized at 0x3C");
    oledInitialized = true;
  }

  if (oledInitialized) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("Loading Starting");
    display.display();
  }

  // Initialize SPI for SD Card
  Serial.println("Initializing SD Card...");
  spiSD.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spiSD)) {
    Serial.println("SD Card initialization failed!");
    sdInitialized = false;
  } else {
    Serial.println("SD Card initialized!");
    sdInitialized = true;
    File statusFile = SD.open("/status.txt", FILE_READ);
    if (statusFile) {
      String line = statusFile.readStringUntil('\n');
      if (line.startsWith("lastSentSequence=")) {
        lastSentSequence = line.substring(17).toInt();
        Serial.println("Loaded lastSentSequence: " + String(lastSentSequence));
      }
      statusFile.close();
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  delay(2000);
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    digitalWrite(RELAY2, HIGH); // WiFi connected, LED on
  } else {
    digitalWrite(RELAY2, LOW);
    if (isLogging) {
      syncPendingData();
    }
  }

  // Handle PZEM reading and sending
  if (millis() - lastPzemPostTime >= PZEM_POST_INTERVAL) {
    lastPzemPostTime = millis();
    readPzemData();
    printPzemData();
    if (WiFi.status() == WL_CONNECTED) {
      sendPzemToServer();
    }
  }

  // Handle temperature reading and sending
  if (millis() - lastTempPostTime >= TEMP_POST_INTERVAL) {
    lastTempPostTime = millis();
    readTempData();
    printTempData();
    if (tempData.sensorDetected && WiFi.status() == WL_CONNECTED) {
      sendTempToServer();
    }
  }

  // Handle RPM reading and sending
  if (millis() - lastRpmPostTime >= RPM_POST_INTERVAL) {
    lastRpmPostTime = millis();
    readRpmData();
    printRpmData();
    if (rpmData.sensorDetected && WiFi.status() == WL_CONNECTED) {
      sendRpmToServer();
    }
  }

  // Log data to SD card
  if (isLogging && sdInitialized && millis() - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = millis();
    logToSD();
  }

  // Update displays
  if (millis() - lastDisplayUpdateTime >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdateTime = millis();
    updateLcdDisplay();
  }

  // Update OLED with running pages
  if (oledInitialized && millis() - lastOledPageTime >= OLED_PAGE_INTERVAL) {
    lastOledPageTime = millis();
    oledPage = (oledPage + 1) % 3;
    updateOledDisplay();
  }

  // Turn off Relay 1 after pulse
  if (relay1Active && millis() - lastRelay1Pulse >= RELAY1_PULSE) {
    digitalWrite(RELAY1, LOW);
    relay1Active = false;
  }

  // Kontrol dimmer untuk heater
  controlDimmer();

  // Kontrol relay untuk indikator error
  controlRelays();

  // Handle keypad input
  handleKeypad();

  // Debug dimmer dan system resources
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime >= 2000) { // Debug setiap 2 detik
    lastDebugTime = millis();
    Serial.println("isLogging: " + String(isLogging));
    Serial.println("Free heap: " + String(ESP.getFreeHeap()));
    Serial.println("Stack high watermark: " + String(uxTaskGetStackHighWaterMark(NULL)));
    Serial.println("Zero-cross detected: " + String(dimmerDebugFlag ? "Yes" : "No"));
    Serial.println("Dimmer value: " + String(dimmerValue) + "%");
    Serial.println("Last zero-cross time: " + String(lastZeroCrossTime) + "us");
    dimmerDebugFlag = false; // Reset flag setelah debug
  }
}

void IRAM_ATTR countPulse() {
  pulseCount++;
}

void connectWiFi() {
  WiFi.mode(WIFI_OFF);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("Connecting to WiFi...");
  unsigned long wifiTimeout = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi. Will retry later.");
  }
}

void readPzemData() {
  if (isnan(pzem.voltage())) {
    pzemData = {0, 0, 0, 0, 0, 0, 0, 0, false};
    return;
  }

  pzemData.sensorDetected = true;
  pzemData.voltage = zeroIfNan(pzem.voltage()) * PZEM_VOLTAGE_SCALE;
  pzemData.current = zeroIfNan(pzem.current()) * PZEM_CURRENT_SCALE;
  pzemData.power = zeroIfNan(pzem.power()) * PZEM_POWER_SCALE;
  pzemData.energy = zeroIfNan(pzem.energy() * 1000);
  pzemData.frequency = zeroIfNan(pzem.frequency());
  pzemData.pf = zeroIfNan(pzem.pf());
  pzemData.va = (pzemData.pf == 0) ? 0 : pzemData.power / pzemData.pf;
  pzemData.var = (pzemData.pf == 0) ? 0 : pzemData.power / pzemData.pf * sqrt(1 - sq(pzemData.pf));
}

void readTempData() {
  float rawTemp = thermocouple.readCelsius();

  if (isnan(rawTemp) || rawTemp < 20 || rawTemp > 1024) {
    Serial.println("Error: MAX6675 not detected!");
    tempData = {0, false};
    return;
  }

  tempData.temperature = rawTemp + TEMP_OFFSET;
  tempData.sensorDetected = true;
}

void readRpmData() {
  noInterrupts();
  unsigned int jumlahPulsa = pulseCount;
  pulseCount = 0;
  interrupts();

  rpmData.rpm = (jumlahPulsa / (float)lubangPerPutaran) * 60.0;
  rpmData.sensorDetected = (rpmData.rpm > 0);
}

void printPzemData() {
  unsigned long elapsedSeconds = (millis() - startTime) / 1000;
  unsigned int hours = elapsedSeconds / 3600;
  unsigned int minutes = (elapsedSeconds % 3600) / 60;
  unsigned int seconds = elapsedSeconds % 60;

  Serial.printf("[PZEM] Timer: %02u:%02u:%02u\t", hours, minutes, seconds);

  if (!pzemData.sensorDetected) {
    Serial.println("SENSOR NOT DETECTED");
    return;
  }

  Serial.print("Volt: "); Serial.print(pzemData.voltage, 2); Serial.print(" V\t");
  Serial.print("Curr: "); Serial.print(pzemData.current, 3); Serial.print(" A\t");
  Serial.print("Pow: "); Serial.print(pzemData.power, 2); Serial.print(" W\t");
  Serial.print("Freq: "); Serial.print(pzemData.frequency, 1); Serial.print(" Hz\t");
  Serial.print("PF: "); Serial.print(pzemData.pf, 2); Serial.print("\t");
  Serial.print("Energy: "); Serial.print(pzemData.energy, 2); Serial.print(" Wh\t");
  Serial.print("VA: "); Serial.print(pzemData.va, 2); Serial.print(" VA\t");
  Serial.print("VAR: "); Serial.print(pzemData.var, 2); Serial.println(" VAR");
}

void printTempData() {
  unsigned long elapsedSeconds = (millis() - startTime) / 1000;
  unsigned int hours = elapsedSeconds / 3600;
  unsigned int minutes = (elapsedSeconds % 3600) / 60;
  unsigned int seconds = elapsedSeconds % 60;

  Serial.printf("[TEMP] Timer: %02u:%02u:%02u\t", hours, minutes, seconds);

  if (!tempData.sensorDetected) {
    Serial.println("SENSOR NOT DETECTED");
    return;
  }

  Serial.print("Temp: "); Serial.print(tempData.temperature, 1); Serial.println(" °C");
}

void printRpmData() {
  unsigned long elapsedSeconds = (millis() - startTime) / 1000;
  unsigned int hours = elapsedSeconds / 3600;
  unsigned int minutes = (elapsedSeconds % 3600) / 60;
  unsigned int seconds = elapsedSeconds % 60;

  Serial.printf("[RPM] Timer: %02u:%02u:%02u\t", hours, minutes, seconds);

  if (!rpmData.sensorDetected) {
    Serial.println("SENSOR NOT DETECTED");
    return;
  }

  Serial.print("RPM: "); Serial.println(rpmData.rpm);
}

void sendPzemToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot send PZEM data: WiFi disconnected");
    return;
  }
  if (!pzemData.sensorDetected) {
    Serial.println("Cannot send PZEM data: Sensor not detected");
    return;
  }

  String postData = 
    "{\"voltage\":" + String(pzemData.voltage, 2) +
    ",\"current\":" + String(pzemData.current, 3) +
    ",\"power\":" + String(pzemData.power, 2) +
    ",\"frequency\":" + String(pzemData.frequency, 1) +
    ",\"power_factor\":" + String(pzemData.pf, 2) +
    ",\"energy\":" + String(pzemData.energy, 2) +
    ",\"va\":" + String(pzemData.va, 2) +
    ",\"var\":" + String(pzemData.var, 2) + "}";

  HTTPClient http;
  if (!http.begin(PZEM_SERVER_URL)) {
    Serial.println("Failed to initialize HTTP client for PZEM");
    return;
  }
  http.addHeader("Content-Type", "application/json");

  Serial.println("Sending PZEM data: " + postData);
  int httpCode = http.POST(postData);
  String payload = http.getString();

  Serial.println("---------- PZEM HTTP POST RESULT ----------");
  Serial.println("URL: " + PZEM_SERVER_URL);
  Serial.println("Data Sent: " + postData);
  Serial.printf("HTTP Code: %d\n", httpCode);
  Serial.println("Server Response: " + payload);
  Serial.println("-------------------------------------------");

  if (httpCode == 200) {
    digitalWrite(RELAY1, HIGH);
    lastRelay1Pulse = millis();
    relay1Active = true;
    if (sdInitialized && isLogging) {
      lastSentSequence = sequenceNumber - 1;
      File statusFile = SD.open("/status.txt", FILE_WRITE);
      if (statusFile) {
        statusFile.println("lastSentSequence=" + String(lastSentSequence));
        statusFile.close();
        Serial.println("Updated lastSentSequence: " + String(lastSentSequence));
      }
    }
  }
  http.end();
}

void sendTempToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot send temperature data: WiFi disconnected");
    return;
  }
  if (!tempData.sensorDetected) {
    Serial.println("Cannot send temperature data: Sensor not detected");
    return;
  }

  String postData = "{\"temperature\":" + String(tempData.temperature, 1) + "}";

  HTTPClient http;
  if (!http.begin(TEMP_SERVER_URL)) {
    Serial.println("Failed to initialize HTTP client for TEMP");
    return;
  }
  http.addHeader("Content-Type", "application/json");

  Serial.println("Sending TEMP data: " + postData);
  int httpCode = http.POST(postData);
  String payload = http.getString();

  Serial.println("---------- TEMP HTTP POST RESULT ----------");
  Serial.println("URL: " + TEMP_SERVER_URL);
  Serial.println("Data Sent: " + postData);
  Serial.printf("HTTP Code: %d\n", httpCode);
  Serial.println("Server Response: " + payload);
  Serial.println("------------------------------------------");

  if (httpCode == 200) {
    digitalWrite(RELAY1, HIGH);
    lastRelay1Pulse = millis();
    relay1Active = true;
  }
  http.end();
}

void sendRpmToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Cannot send RPM data: WiFi disconnected");
    return;
  }
  if (!rpmData.sensorDetected) {
    Serial.println("Cannot send RPM data: Sensor not detected");
    return;
  }

  String postData = "{\"rpm\":" + String(rpmData.rpm, 1) + "}";

  HTTPClient http;
  if (!http.begin(RPM_SERVER_URL)) {
    Serial.println("Failed to initialize HTTP client for RPM");
    return;
  }
  http.addHeader("Content-Type", "application/json");

  Serial.println("Sending RPM data: " + postData);
  int httpCode = http.POST(postData);
  String payload = http.getString();

  Serial.println("---------- RPM HTTP POST RESULT ----------");
  Serial.println("URL: " + RPM_SERVER_URL);
  Serial.println("Data Sent: " + postData);
  Serial.printf("HTTP Code: %d\n", httpCode);
  Serial.println("Server Response: " + payload);
  Serial.println("-----------------------------------------");

  if (httpCode == 200) {
    digitalWrite(RELAY1, HIGH);
    lastRelay1Pulse = millis();
    relay1Active = true;
  }
  http.end();
}

void logToSD() {
  File dataFile = SD.open(currentCsvFile, FILE_APPEND);
  if (dataFile) {
    String logEntry = String(sequenceNumber) + "," +
                      String(millis()) + "," +
                      String(pzemData.voltage) + "," +
                      String(pzemData.current, 1) + "," +
                      String(pzemData.power) + "," +
                      String(pzemData.frequency) + "," +
                      String(pzemData.pf) + "," +
                      String(pzemData.energy) + "," +
                      String(pzemData.va) + "," +
                      String(pzemData.var) + "," +
                      String(tempData.temperature, 1) + "," +
                      String(rpmData.rpm) + "," +
                      String(checkErrors() ? "ERROR" : "OK");

    dataFile.println(logEntry);
    dataFile.close();
    Serial.println("Data logged to SD card: " + currentCsvFile);
    sequenceNumber++;
  } else {
    Serial.println("Failed to open " + currentCsvFile);
  }
}

void syncPendingData() {
  if (!sdInitialized || WiFi.status() != WL_CONNECTED) {
    return;
  }

  File dataFile = SD.open(currentCsvFile, FILE_READ);
  if (!dataFile) {
    Serial.println("Failed to open " + currentCsvFile + " for syncing");
    return;
  }

  unsigned long currentSequence = lastSentSequence + 1;
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    int firstComma = line.indexOf(',');
    if (firstComma == -1) continue;

    String seqStr = line.substring(0, firstComma);
    unsigned long seq = seqStr.toInt();
    if (seq < currentSequence) continue;

    String data = line.substring(firstComma + 1);
    int commaCount = 0;
    for (char c : data) {
      if (c == ',') commaCount++;
    }
    if (commaCount < 11) continue;

    String postData = "{\"voltage\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"current\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"power\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"frequency\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"power_factor\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"energy\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"va\":" + data.substring(0, data.indexOf(',')) + ",";
    data = data.substring(data.indexOf(',') + 1);
    postData += "\"var\":" + data.substring(0, data.indexOf(',')) + "}";

    String tempData = "{\"temperature\":" + data.substring(0, data.indexOf(',')) + "}";
    data = data.substring(data.indexOf(',') + 1);

    String rpmData = "{\"rpm\":" + data.substring(0, data.indexOf(',')) + "}";

    HTTPClient http;
    int httpCode;

    http.begin(PZEM_SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST(postData);
    String payload = http.getString();

    Serial.println("---------- SYNC PZEM HTTP POST RESULT ----------");
    Serial.println("Sequence: " + String(seq));
    Serial.println("Data Sent: " + postData);
    Serial.printf("HTTP Code: %d\n", httpCode);
    Serial.println("Server Response: " + payload);
    Serial.println("-----------------------------------------------");

    bool syncSuccess = (httpCode == 200);
    if (syncSuccess) {
      lastSentSequence = seq;
      File statusFile = SD.open("/status.txt", FILE_WRITE);
      if (statusFile) {
        statusFile.println("lastSentSequence=" + String(lastSentSequence));
        statusFile.close();
        Serial.println("Updated lastSentSequence: " + String(lastSentSequence));
      }
    }
    http.end();

    http.begin(TEMP_SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST(tempData);
    payload = http.getString();

    Serial.println("---------- SYNC TEMP HTTP POST RESULT ----------");
    Serial.println("Sequence: " + String(seq));
    Serial.println("Data Sent: " + tempData);
    Serial.printf("HTTP Code: %d\n", httpCode);
    Serial.println("Server Response: " + payload);
    Serial.println("-----------------------------------------------");
    http.end();

    http.begin(RPM_SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    httpCode = http.POST(rpmData);
    payload = http.getString();

    Serial.println("---------- SYNC RPM HTTP POST RESULT ----------");
    Serial.println("Sequence: " + String(seq));
    Serial.println("Data Sent: " + rpmData);
    Serial.printf("HTTP Code: %d\n", httpCode);
    Serial.println("Server Response: " + payload);
    Serial.println("-----------------------------------------------");
    http.end();
  }
  dataFile.close();
}

bool checkErrors() {
  return (!pzemData.sensorDetected ||
          !tempData.sensorDetected ||
          !rpmData.sensorDetected ||
          pzemData.voltage < VOLT_MIN || pzemData.voltage > VOLT_MAX ||
          pzemData.current < CURR_MIN || pzemData.current > CURR_MAX ||
          pzemData.power < POWER_MIN || pzemData.power > POWER_MAX ||
          tempData.temperature < TEMP_MIN || tempData.temperature > TEMP_MAX ||
          rpmData.rpm < RPM_MIN || rpmData.rpm > RPM_MAX);
}

void controlDimmer() {
  static unsigned long lastDimmerUpdate = 0;
  const unsigned long DIMMER_UPDATE_INTERVAL = 500; // Update setiap 500ms

  if (millis() - lastDimmerUpdate < DIMMER_UPDATE_INTERVAL) {
    return; // Batasi pembaruan untuk mengurangi beban
  }
  lastDimmerUpdate = millis();

  if (!dimmerActive || !tempData.sensorDetected || tempSetpoint <= 0.0) {
    setDimmerPower(MIN_DIMMER_VALUE);
    dimmerValue = MIN_DIMMER_VALUE;
    Serial.println("Dimmer OFF: " + String(dimmerValue));
    return;
  }

  float proportionalStartTemp = tempSetpoint * PROPORTIONAL_START; // 90% setpoint
  float fullOffTemp = tempSetpoint - FULL_OFF_OFFSET; // Setpoint - 5°C

  if (tempData.temperature >= tempSetpoint + HYSTERESIS) {
    dimmerValue = MIN_DIMMER_VALUE; // Suhu ≥ setpoint: mati
  } else if (tempData.temperature <= proportionalStartTemp) {
    dimmerValue = MAX_DIMMER_VALUE; // Suhu ≤ 90% setpoint: daya penuh
  } else if (tempData.temperature >= fullOffTemp) {
    dimmerValue = MIN_DIMMER_VALUE; // Suhu ≥ setpoint - 5°C: mati
  } else {
    float proportion = (fullOffTemp - tempData.temperature) / (fullOffTemp - proportionalStartTemp);
    dimmerValue = (int)(proportion * MAX_DIMMER_VALUE);
    dimmerValue = constrain(dimmerValue, MIN_DIMMER_VALUE, MAX_DIMMER_VALUE);
  }

  setDimmerPower(dimmerValue);
  Serial.println("Dimmer: " + String(dimmerValue) + "%, Temp: " + String(tempData.temperature, 1) + " C, Setpoint: " + String(tempSetpoint, 1) + " C");
}

void controlRelays() {
  digitalWrite(RELAY3, checkErrors() ? HIGH : LOW);
}

void handleKeypad() {
  char key = readKeypad();
  if (!key) return;
  Serial.println("Key: " + String(key));

  if (key == 'B') {
    resetSystem();
    return;
  }

  if (isSettingSetpoint) {
    if (key >= '0' && key <= '9' || key == '.') {
      setpointInput += key;
    } else if (key == '*') {
      setpointInput = "";
    } else if (key == '#') {
      if (setpointInput.length() > 0) {
        tempSetpoint = setpointInput.toFloat();
        dimmerActive = true;
        isSettingSetpoint = false;
        setpointInput = "";
        isSettingDataNumber = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter Data No:");
        lcd.setCursor(0, 1);
        lcd.print(">");
        saveEEPROM();
      }
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter Setpoint:");
    lcd.setCursor(0, 1);
    lcd.print(setpointInput);
    lastKeyDisplayTime = millis();
  } else if (isSettingDataNumber) {
    if (key >= '0' && key <= '9') {
      if (dataNumberInput.length() < 3) {
        dataNumberInput += key;
      }
    } else if (key == '*') {
      dataNumberInput = "";
    } else if (key == '#') {
      if (dataNumberInput.length() == 3) {
        isSettingDataNumber = false;
        isSettingDate = true;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter Date:");
        lcd.setCursor(0, 1);
        lcd.print("DDMMYYYY >");
      }
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter Data No:");
    lcd.setCursor(0, 1);
    lcd.print(dataNumberInput);
    lastKeyDisplayTime = millis();
  } else if (isSettingDate) {
    if (key >= '0' && key <= '9') {
      dateInput += key;
    } else if (key == '*') {
      dateInput = "";
    } else if (key == '#') {
      if (dateInput.length() == 8) {
        isSettingDate = false;
        configureNewCsvFile();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("System Running");
        isLogging = true;
      }
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter Date:");
    lcd.setCursor(0, 1);
    lcd.print(dateInput);
    lastKeyDisplayTime = millis();
  } else if (key == '*') {
    isSettingSetpoint = true;
    setpointInput = "";
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Enter Setpoint:");
    lcd.setCursor(0, 1);
    lcd.print(">");
    lastKeyDisplayTime = millis();
  } else if (key == 'A') {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SKRIPSI ROASTER");
    lcd.setCursor(0, 1);
    lcd.print("SQUAD 2025");
    isDisplayingSkripsi = true;
    lastSkripsiDisplayTime = millis();
  }
}

void configureNewCsvFile() {
  if (!sdInitialized) return;

  currentCsvFile = "/data_" + dataNumberInput + "_" + dateInput + ".csv";
  File dataFile = SD.open(currentCsvFile, FILE_WRITE);
  if (dataFile) {
    dataFile.println("sequenceNumber,millis,voltage,current,power,frequency,pf,energy,va,var,temperature,rpm,status");
    dataFile.close();
    Serial.println("Created new CSV file: " + currentCsvFile);
    sequenceNumber = 1;
    lastSentSequence = 0;
    File statusFile = SD.open("/status.txt", FILE_WRITE);
    if (statusFile) {
      statusFile.println("lastSentSequence=0");
      statusFile.close();
    }
  } else {
    Serial.println("Failed to create " + currentCsvFile);
  }
}

void updateLcdDisplay() {
  if (isDisplayingSkripsi && millis() - lastSkripsiDisplayTime < SKRIPSI_DISPLAY_DURATION) {
    return;
  } else if (isDisplayingSkripsi) {
    isDisplayingSkripsi = false;
  }

  if (millis() - lastKeyDisplayTime < KEY_DISPLAY_DURATION) return;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(tempData.sensorDetected ? "Temp: " + String(tempData.temperature, 1) + " C" : "Temp Error");
  lcd.setCursor(0, 1);
  lcd.print("Set: " + String(tempSetpoint, 1) + " C");
}

void updateOledDisplay() {
  if (!oledInitialized) return;

  if (isDisplayingSkripsi && millis() - lastSkripsiDisplayTime < SKRIPSI_DISPLAY_DURATION) {
    return;
  } else if (isDisplayingSkripsi) {
    isDisplayingSkripsi = false;
  }

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);

  if (oledPage == 0) {
    display.setTextSize(1);
    display.print("Voltage  : "); display.print(pzemData.voltage, 2); display.println(" V");
    display.print("Current  : "); display.print(pzemData.current, 2); display.println(" A");
    display.print("Power    : "); display.print(pzemData.power, 2); display.println(" W");
    display.print("Cos Phi  : "); display.print(pzemData.pf, 2); display.println(" PF");
    display.print("Apprn Pwr: "); display.print(pzemData.va, 2); display.println(" VA");
    display.print("React Pwr: "); display.print(pzemData.var, 2); display.println(" VAR");
    display.print("Frequency: "); display.print(pzemData.frequency, 1); display.println(" Hz");
    display.print("Energy   : "); display.print(pzemData.energy, 0); display.println(" Wh");
  } else if (oledPage == 1) {
    if (tempData.sensorDetected) {
      display.setTextSize(1);
      display.println("Temperature");
      display.println(String(tempData.temperature, 1) + " C");
    } else {
      display.println("TEMP SENSOR");
      display.println("NOT DETECTED");
    }
    display.setTextSize(1);
    display.println();
    display.print("Setpoint: "); display.print(tempSetpoint, 1); display.println(" C");
  } else {
    if (rpmData.sensorDetected) {
      display.setTextSize(2);
      display.println("RPM");
      display.println(String(rpmData.rpm, 0) + " RPM");
    } else {
      display.println("RPM SENSOR");
      display.println("NOT DETECTED");
    }
    display.setTextSize(1);
    display.println();
    display.print("Dimmer: "); display.print(dimmerValue); display.print("% ");
    display.println(dimmerActive ? "(Auto)" : "(Manual)");
  }
  display.display();
}

void resetSystem() {
  delay(1000);
  pzem.resetEnergy();
  startTime = millis();
  noInterrupts();
  pulseCount = 0;
  interrupts();
  tempSetpoint = 0.0;
  setpointInput = "";
  dataNumberInput = "";
  dateInput = "";
  isSettingSetpoint = false;
  isSettingDataNumber = false;
  isSettingDate = false;
  isLogging = false;
  dimmerValue = MIN_DIMMER_VALUE;
  dimmerActive = false;
  setDimmerPower(MIN_DIMMER_VALUE);
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  sequenceNumber = 1;
  lastSentSequence = 0;
  currentCsvFile = "";
  saveEEPROM();
  lcd.clear();
  lcd.print("System Reset");
  if (oledInitialized) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("System Reset");
    display.display();
  }
}

char readKeypad() {
  pcf.write8(0xFF);
  delayMicroseconds(500);
  for (byte r = 0; r < 4; r++) {
    pcf.write8(~(1 << rowPins[r]));
    delayMicroseconds(1000);
    uint8_t colData = pcf.read8() ^ 0xFF;
    for (byte c = 0; c < 4; c++) {
      if (colData & (1 << colPins[c])) {
        while (pcf.read8() & (1 << colPins[c]));
        delay(100);
        pcf.write8(0xFF);
        return keys[r][c];
      }
    }
  }
  pcf.write8(0xFF);
  return 0;
}

float zeroIfNan(float value) {
  return isnan(value) ? 0 : value;
}

void loadEEPROM() {
  float temp;
  EEPROM.get(EEPROM_ADDR_SETPOINT, temp);
  if (!isnan(temp) && temp >= TEMP_MIN && temp <= TEMP_MAX) {
    tempSetpoint = temp;
    Serial.println("Loaded tempSetpoint from EEPROM: " + String(tempSetpoint, 1));
  } else {
    tempSetpoint = 0.0;
    Serial.println("Invalid tempSetpoint in EEPROM, using default: 0.0");
  }

  bool mode;
  EEPROM.get(EEPROM_ADDR_DIMMER_MODE, mode);
  dimmerActive = mode;
  Serial.println("Loaded dimmerActive from EEPROM: " + String(dimmerActive ? "Auto" : "Manual"));
}

void saveEEPROM() {
  EEPROM.put(EEPROM_ADDR_SETPOINT, tempSetpoint);
  EEPROM.put(EEPROM_ADDR_DIMMER_MODE, dimmerActive);
  if (EEPROM.commit()) {
    Serial.println("Saved to EEPROM: tempSetpoint=" + String(tempSetpoint, 1) + ", dimmerActive=" + String(dimmerActive ? "Auto" : "Manual"));
  } else {
    Serial.println("Failed to save to EEPROM");
  }
}