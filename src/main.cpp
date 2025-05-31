#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <MAX30105.h>
#include <heartRate.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

// Constants
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define ECG_BUFFER_SIZE 200
#define RATE_SIZE 4
#define LO_PLUS_PIN 26  // Lead-off detection positive
#define LO_MINUS_PIN 27 // Lead-off detection negative

// Function declarations
void displayWelcomeMessage();
void updateDisplay();
void checkThresholds();
void readMAX30102();
void readECG();
void handleRoot(AsyncWebServerRequest *request);
void handleData(AsyncWebServerRequest *request);
void handleECGData(AsyncWebServerRequest *request);
void handleSetThreshold(AsyncWebServerRequest *request);
void checkLeadStatus();
void checkFingerPresence();
void setupMAX30102();
void handleIRData(AsyncWebServerRequest *request);

unsigned long lastTempRead = 0;
const int tempInterval = 30000; // every 30 sec

// WiFi Credentials
const char *ssid = "OPTIMUS";
const char *password = "qqwweeaaaa";

// Global variables
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MAX30105 particleSensor;
AsyncWebServer server(80);
bool leadsOff = false;
unsigned long lastLeadCheck = 0;
const unsigned long leadCheckInterval = 1000; // Check every 1 second
const int IR_BUFFER_SIZE = 200;
int irBuffer[IR_BUFFER_SIZE];
int irIndex = 0;
// Thresholds
int bpmThresholdHigh = 120;
int bpmThresholdLow = 50;
int spo2Threshold = 90;
int ecgThreshold = 1000;

// Sensor data
float bpm = 0;
float spo2 = 0;
float temperature = 0;
int ecgValue = 0;
bool heartAttackAlert = false;
bool fingerDetected = false;
unsigned long lastDetectionTime = 0;
const unsigned long detectionTimeout = 3000; // 3 seconds timeout
float irValueSmooth = 0;
const float smoothingFactor = 0.1; // Smoothing factor for IR values

// For BPM calculation
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;

// ECG buffer
int ecgBuffer[ECG_BUFFER_SIZE];
int ecgIndex = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(LO_PLUS_PIN, INPUT);
  pinMode(LO_MINUS_PIN, INPUT);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.display();
  delay(2000);
  display.clearDisplay();

  setupMAX30102();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize LittleFS
  if (!LittleFS.begin())
  {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  // List files to verify
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while (file)
  {
    Serial.print("File: ");
    Serial.println(file.name());
    file = root.openNextFile();
  }

  // Web server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/ecgdata", HTTP_GET, handleECGData);
  server.on("/setthreshold", HTTP_POST, handleSetThreshold);
  server.on("/irdata", HTTP_GET, handleIRData);
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  temperature = particleSensor.readTemperature();
  displayWelcomeMessage();
}

void loop()
{
  checkFingerPresence();
  if (fingerDetected)
  {
    readMAX30102();
    checkThresholds();
  }
  else
  {
    // Reset values when no finger is present
    bpm = 0;
    spo2 = 0;
    heartAttackAlert = false;
  }

  readECG();
  checkLeadStatus();
  updateDisplay();
}

void setupMAX30102()
{
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 not found");
    while (1)
      ;
  }

  // Configure sensor with optimal settings
  byte ledBrightness = 0x1F; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;    // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400;      // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;      // Options: 69, 118, 215, 411
  int adcRange = 4096;       // Options: 2048, 4096, 8192, 16384

  // particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.setup();
  // Configure pulse amplitude
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // The LEDs are very low power and won't affect the finger detection
  particleSensor.setPulseAmplitudeProximity(0x0F); // Helps with finger detection
}

void checkLeadStatus()
{
  if (millis() - lastLeadCheck >= leadCheckInterval)
  {
    lastLeadCheck = millis();

    // Read lead-off detection pins
    bool loPlus = digitalRead(LO_PLUS_PIN);
    bool loMinus = digitalRead(LO_MINUS_PIN);

    // Determine lead status (active low: 0 = lead connected, 1 = lead off)
    leadsOff = loPlus || loMinus;

    if (leadsOff)
    {
      Serial.println("Warning: Electrode leads disconnected!");
    }
  }
}

// Function implementations
void updateDisplay()
{
  display.clearDisplay();

  // Header (always visible)
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Heart Monitor");

  // Last update time (right aligned)
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  if (millis() - lastBlink > 500)
  { // Blink every 500ms
    blinkState = !blinkState;
    lastBlink = millis();
  }

  // Finger detection status (blinking when no finger)
  display.setCursor(90, 0);
  if (fingerDetected)
  {
    display.setTextColor(WHITE);
    display.print("FINGER OK");
  }
  else
  {
    display.setTextColor(blinkState ? WHITE : BLACK); // Blink effect
    display.print("NO FINGER");
  }

  // Main data section (only show when finger detected)
  if (fingerDetected)
  {
    // BPM display
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 15);
    display.print("BPM:");

    display.setTextSize(2);
    display.setCursor(40, 12);
    display.print(bpm);

    // SpO2 display
    display.setTextSize(1);
    display.setCursor(0, 35);
    display.print("SpO2:");

    display.setTextSize(2);
    display.setCursor(40, 32);
    display.print(spo2);
    display.print("%");

    // Temperature display
    display.setTextSize(1);
    display.setCursor(0, 55);
    display.print("Temp:");

    display.setTextSize(1);
    display.setCursor(40, 55);
    display.print(temperature);
    display.print("C");

    // Status indicators (right aligned)
    if (leadsOff)
    {
      display.setTextSize(1);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(80, 20);
      display.print("LEAD OFF");
    }

    if (heartAttackAlert)
    {
      display.setTextSize(1);
      display.setTextColor(BLACK, WHITE);
      display.setCursor(80, 45);
      display.print("ALERT!");
    }
  }
  else
  {
    // Show instructions when no finger detected
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.print("Place finger on");
    display.setCursor(0, 30);
    display.print("sensor to start");
    display.setCursor(0, 40);
    display.print("measurement...");
  }

  display.display();
}
void checkThresholds()
{
  heartAttackAlert = (bpm > bpmThresholdHigh || bpm < bpmThresholdLow || spo2 < spo2Threshold);
}

void readMAX30102()
{
  long irValue = particleSensor.getIR();

  // Store in circular buffer
  irBuffer[irIndex] = irValue;
  irIndex = (irIndex + 1) % IR_BUFFER_SIZE;

  if (checkForBeat(irValue))
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);
    Serial.println(beatsPerMinute);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  bpm = beatAvg;
  float redValue = particleSensor.getRed();
  spo2 = 100 - (redValue / 10000); // Placeholder calculation

  if (millis() - lastTempRead > tempInterval)
  {
    temperature = particleSensor.readTemperature();
    lastTempRead = millis();
  }
}

void readECG()
{
  if (!leadsOff)
  {
    ecgValue = analogRead(34);
    ecgBuffer[ecgIndex] = ecgValue;
    ecgIndex = (ecgIndex + 1) % ECG_BUFFER_SIZE;
  }
  else
  {
    ecgValue = 0;
  }
}

void handleRoot(AsyncWebServerRequest *request)
{
  request->send(LittleFS, "/index.html", "text/html");
}

void handleData(AsyncWebServerRequest *request)
{
  String json = "{\"bpm\":" + String(bpm) +
                ",\"spo2\":" + String(spo2) +
                ",\"temp\":" + String(temperature) +
                ",\"ecg\":" + String(ecgValue) +
                ",\"alert\":" + String(heartAttackAlert) +
                ",\"leadsOff\":" + String(leadsOff) + "}";
  request->send(200, "application/json", json);
}

void handleECGData(AsyncWebServerRequest *request)
{
  String ecgJson = "[";
  for (int i = 0; i < ECG_BUFFER_SIZE; i++)
  {
    ecgJson += String(ecgBuffer[i]);
    if (i < ECG_BUFFER_SIZE - 1)
      ecgJson += ",";
  }
  ecgJson += "]";
  request->send(200, "application/json", ecgJson);
}

void handleSetThreshold(AsyncWebServerRequest *request)
{
  if (request->hasParam("bpmHigh", true))
  {
    bpmThresholdHigh = request->getParam("bpmHigh", true)->value().toInt();
  }
  if (request->hasParam("bpmLow", true))
  {
    bpmThresholdLow = request->getParam("bpmLow", true)->value().toInt();
  }
  if (request->hasParam("spo2Threshold", true))
  {
    spo2Threshold = request->getParam("spo2Threshold", true)->value().toInt();
  }
  if (request->hasParam("ecgThreshold", true))
  {
    ecgThreshold = request->getParam("ecgThreshold", true)->value().toInt();
  }
  request->send(200, "text/plain", "Thresholds updated");
}

void checkFingerPresence()
{ 
  static float irBuffer[10] = {0};
  static byte bufIndex = 0;

  // Get current IR value
  float currentIR = particleSensor.getIR();

  // Apply simple smoothing
  irValueSmooth = (1.0 - smoothingFactor) * irValueSmooth + smoothingFactor * currentIR;

  // Store in circular buffer
  irBuffer[bufIndex] = irValueSmooth;
  bufIndex = (bufIndex + 1) % 10;

  // Calculate variance
  float mean = 0, variance = 0;
  for (int i = 0; i < 10; i++)
    mean += irBuffer[i];
  mean /= 10;
  for (int i = 0; i < 10; i++)
    variance += pow(irBuffer[i] - mean, 2);
  variance /= 10;

  // Finger detection logic
  bool currentDetection = (irValueSmooth > 50000) && (variance < 10000);

  if (currentDetection)
  {
    lastDetectionTime = millis();
    if (!fingerDetected)
    {
      fingerDetected = true;
      Serial.println("Finger detected");
    }
  }
  else if (millis() - lastDetectionTime > detectionTimeout)
  {
    if (fingerDetected)
    {
      fingerDetected = false;
      Serial.println("Finger removed");
      // Reset BPM calculation when finger is removed
      beatAvg = 0;
      rateSpot = 0;
      lastBeat = 0;
      memset(rates, 0, sizeof(rates));
    }
  }
}

void handleIRData(AsyncWebServerRequest *request)
{
  String irJson = "[";
  for (int i = 0; i < IR_BUFFER_SIZE; i++)
  {
    irJson += String(irBuffer[i]);
    if (i < IR_BUFFER_SIZE - 1)
      irJson += ",";
  }
  irJson += "]";
  request->send(200, "application/json", irJson);
}

void displayWelcomeMessage()
{
  display.clearDisplay();

  // Draw a heart at the top center
  display.drawBitmap(52, 0,
                     new uint8_t[8]{
                         0b00000000,
                         0b01100110,
                         0b11111111,
                         0b11111111,
                         0b01111110,
                         0b00111100,
                         0b00011000,
                         0b00000000},
                     8, 8, WHITE);

  // Display title with heart symbol
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println(F(" Heart Monitor System"));
  display.drawFastHLine(0, 20, display.width(), WHITE);

  // Display IP address in a box
  display.setCursor(0, 25);
  display.println(F("IP Address:"));
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.print(F(" "));
  display.print(WiFi.localIP());
  display.print(F(" "));

  // Draw a box around the IP
  display.drawRect(0, 34, display.width(), 12, WHITE);

  // Hardware initialization status
  display.setCursor(0, 50);
  display.print(F("Initializing..."));

  display.display();

  // Animate the initialization process
  for (int i = 0; i < 3; i++)
  {
    display.fillRect(80, 50, 40, 8, BLACK);
    display.setCursor(80, 50);
    display.print(F("."));
    display.display();
    delay(300);
    display.setCursor(90, 50);
    display.print(F("."));
    display.display();
    delay(300);
    display.setCursor(100, 50);
    display.print(F("."));
    display.display();
    delay(300);
  }

  // Display hardware status
  display.fillRect(0, 50, display.width(), 8, BLACK);
  display.setCursor(0, 50);

  if (particleSensor.begin() &&
      !digitalRead(LO_PLUS_PIN) &&
      !digitalRead(LO_MINUS_PIN))
  {
    display.println(F("Hardware OK!"));
  }
  else
  {
    display.println(F("HW Error! Check:"));
    display.setCursor(0, 60);
    if (!particleSensor.begin())
      display.print(F("MAX30102 "));
    if (digitalRead(LO_PLUS_PIN) || digitalRead(LO_MINUS_PIN))
      display.print(F("AD8232 "));
  }

  display.display();
  delay(2000);

  // Clear and show ready message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("System Ready"));
  display.setCursor(0, 20);
  display.println(F("Monitoring..."));
  display.display();
  delay(1000);
}