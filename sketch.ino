
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================
// LIBRARY OCCUPANCY PREDICTION - ESP32 / WOKWI
// ============================================

// Entry sensor
#define ENTRY_TRIG 5
#define ENTRY_ECHO 18

// Exit sensor
#define EXIT_TRIG 17
#define EXIT_ECHO 16

// LEDs and buzzer
#define GREEN_LED 25
#define YELLOW_LED 26
#define RED_LED 27
#define BUZZER 14

// Library configuration
const int MAX_CAPACITY = 100;
const int WARNING_PERCENT = 80;
const int INITIAL_OCCUPANCY = 25;

// Sensor configuration
const float DETECT_DISTANCE_CM = 80.0;
const unsigned long SENSOR_COOLDOWN_MS = 1200;

// Timing configuration
const unsigned long SAMPLE_INTERVAL_MS = 10000;
const unsigned long DISPLAY_INTERVAL_MS = 500;
const unsigned long SERIAL_INTERVAL_MS = 2000;

const int HISTORY_SIZE = 6;

// LCD I2C address
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Occupancy state
int occupancy = INITIAL_OCCUPANCY;
int occupancyHistory[HISTORY_SIZE];
int historyCount = 0;
int historyIndex = 0;

// Sensor state
bool previousEntryDetected = false;
bool previousExitDetected = false;

unsigned long lastEntryEvent = 0;
unsigned long lastExitEvent = 0;
unsigned long lastSampleTime = 0;
unsigned long lastDisplayTime = 0;
unsigned long lastSerialTime = 0;

// Read distance from HC-SR04
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration =
      pulseIn(echoPin, HIGH, 25000UL);

  if (duration == 0) {
    return 999.0;
  }

  return duration * 0.0343 / 2.0;
}

// Save a new occupancy sample
void saveOccupancySample() {
  occupancyHistory[historyIndex] = occupancy;

  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  if (historyCount < HISTORY_SIZE) {
    historyCount++;
  }
}

// Predict occupancy five minutes ahead
// Uses a simple linear trend from recent samples
float predictOccupancy() {
  if (historyCount < 2) {
    return occupancy;
  }

  float sumX = 0.0;
  float sumY = 0.0;
  float sumXY = 0.0;
  float sumXX = 0.0;

  for (int i = 0; i < historyCount; i++) {
    int index =
        (historyIndex - historyCount + i
         + HISTORY_SIZE) % HISTORY_SIZE;

    float x = i;
    float y = occupancyHistory[index];

    sumX += x;
    sumY += y;
    sumXY += x * y;
    sumXX += x * x;
  }

  float n = historyCount;
  float denominator = n * sumXX - sumX * sumX;

  if (denominator == 0.0) {
    return occupancy;
  }

  float slope =
      (n * sumXY - sumX * sumY) / denominator;

  // Samples are 10 seconds apart.
  // Five minutes corresponds to 30 intervals.
  float prediction = occupancy + slope * 30.0;

  if (prediction < 0.0) {
    prediction = 0.0;
  }

  if (prediction > MAX_CAPACITY) {
    prediction = MAX_CAPACITY;
  }

  return prediction;
}

// Control LEDs and buzzer
void updateIndicators(float prediction) {
  int warningLevel =
      MAX_CAPACITY * WARNING_PERCENT / 100;

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);
  noTone(BUZZER);

  if (occupancy >= MAX_CAPACITY ||
      prediction >= MAX_CAPACITY) {

    digitalWrite(RED_LED, HIGH);
    tone(BUZZER, 1800, 300);

  } else if (occupancy >= warningLevel ||
             prediction >= warningLevel) {

    digitalWrite(YELLOW_LED, HIGH);

  } else {
    digitalWrite(GREEN_LED, HIGH);
  }
}

// Display occupancy on LCD
void updateLCD(float prediction) {
  lcd.setCursor(0, 0);
  lcd.print("Inside:");
  lcd.print(occupancy);
  lcd.print("/");
  lcd.print(MAX_CAPACITY);
  lcd.print("    ");

  lcd.setCursor(0, 1);
  lcd.print("In 5 min:");
  lcd.print((int)(prediction + 0.5));
  lcd.print("      ");
}

// Print project data to Serial Monitor
void printTelemetry(float prediction) {
  Serial.print("Occupancy=");
  Serial.print(occupancy);

  Serial.print(", Predicted_5min=");
  Serial.print(prediction, 1);

  Serial.print(", Capacity=");
  Serial.print(MAX_CAPACITY);

  Serial.print(", Status=");

  if (occupancy >= MAX_CAPACITY ||
      prediction >= MAX_CAPACITY) {

    Serial.println("FULL");

  } else if (
      occupancy >= MAX_CAPACITY * WARNING_PERCENT / 100 ||
      prediction >= MAX_CAPACITY * WARNING_PERCENT / 100) {

    Serial.println("WARNING");

  } else {
    Serial.println("NORMAL");
  }
}

// Initialize ESP32 and components
void setup() {
  Serial.begin(115200);

  pinMode(ENTRY_TRIG, OUTPUT);
  pinMode(ENTRY_ECHO, INPUT);

  pinMode(EXIT_TRIG, OUTPUT);
  pinMode(EXIT_ECHO, INPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  digitalWrite(ENTRY_TRIG, LOW);
  digitalWrite(EXIT_TRIG, LOW);

  // ESP32 I2C pins
  Wire.begin(21, 22);

  lcd.init();
  lcd.backlight();

  // Initialize occupancy history
  for (int i = 0; i < HISTORY_SIZE; i++) {
    occupancyHistory[i] = INITIAL_OCCUPANCY;
  }

  saveOccupancySample();

  lcd.setCursor(0, 0);
  lcd.print("Library IoT");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1500);
  lcd.clear();

  Serial.println();
  Serial.println("Library Occupancy Prediction");
  Serial.println("ESP32 Simulation Started");
  Serial.println("Initial occupancy: 25");
  Serial.println("Maximum capacity: 100");
  Serial.println("Prediction interval: 5 minutes");
  Serial.println("--------------------------------");
}

// Main program loop
void loop() {
  unsigned long now = millis();

  // Read entry sensor
  float entryDistance =
      readDistance(ENTRY_TRIG, ENTRY_ECHO);

  delay(40);

  // Read exit sensor
  float exitDistance =
      readDistance(EXIT_TRIG, EXIT_ECHO);

  bool entryDetected =
      entryDistance < DETECT_DISTANCE_CM;

  bool exitDetected =
      exitDistance < DETECT_DISTANCE_CM;

  // Count a person entering
  if (entryDetected &&
      !previousEntryDetected &&
      (lastEntryEvent == 0 ||
       now - lastEntryEvent >= SENSOR_COOLDOWN_MS)) {

    if (occupancy < MAX_CAPACITY) {
      occupancy++;
      Serial.println("EVENT: PERSON ENTERED");
    } else {
      Serial.println("EVENT: LIBRARY FULL - ENTRY BLOCKED");
    }

    lastEntryEvent = now;
  }

  // Count a person leaving
  if (exitDetected &&
      !previousExitDetected &&
      (lastExitEvent == 0 ||
       now - lastExitEvent >= SENSOR_COOLDOWN_MS)) {

    if (occupancy > 0) {
      occupancy--;
      Serial.println("EVENT: PERSON EXITED");
    } else {
      Serial.println("EVENT: LIBRARY EMPTY");
    }

    lastExitEvent = now;
  }

  previousEntryDetected = entryDetected;
  previousExitDetected = exitDetected;

  // Record occupancy periodically
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    saveOccupancySample();
    lastSampleTime = now;
  }

  // Update forecast and alerts
  float prediction = predictOccupancy();

  updateIndicators(prediction);

  // Update LCD
  if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
    updateLCD(prediction);
    lastDisplayTime = now;
  }

  // Print telemetry
  if (now - lastSerialTime >= SERIAL_INTERVAL_MS) {
    printTelemetry(prediction);
    lastSerialTime = now;
  }

  delay(50);
}
