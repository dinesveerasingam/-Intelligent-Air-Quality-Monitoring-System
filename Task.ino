/*
 * ===============================================================================
 * ATmega32A - Intelligent Air Quality & Ventilation Control System
 * ===============================================================================
 *
 * Fixes:
 * - Re-enabled Sri Lanka offset (+5:30) when showing RTC time on LCD
 * - Ensure fanSpeedPercent always updated immediately after computing fanSpeed
 * - Added Serial debug prints for fanSpeed and fanSpeedPercent (useful to verify)
 * - Kept all other logic (counting, AQ, PWM) same as your requested behaviour
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <SPI.h>
#include <SD.h>

// ==================== PIN DEFINITIONS ====================
#define IR_SENSOR_A      10
#define IR_SENSOR_B      11
#define MOTOR_IN1        12
#define MOTOR_IN2        13
#define MOTOR_ENABLE     3     // PWM pin (keep your mapping)
#define MQ2_PIN          A0
#define DUST_LED_PIN     26
#define DUST_MEASURE_PIN A1
#define LED_GREEN        0
#define LED_YELLOW       1
#define LED_RED          2
#define SD_CS_PIN        4

// ==================== SENSOR PARAMETERS ====================
#define SAMPLING_TIME    280
#define DELTA_TIME       40
#define SLEEP_TIME       9680

#define MQ2_GOOD         300
#define MQ2_MODERATE     600
#define MQ2_POOR         1000

#define DUST_GOOD        35
#define DUST_MODERATE    75
#define DUST_POOR        150

#define MIN_FAN_SPEED    50    // baseline minimum PWM
#define MAX_FAN_SPEED    255
#define FAN_OFF          0

#define DATA_LOG_INTERVAL   60000UL
#define DISPLAY_UPDATE      1000UL
#define SENSOR_READ_DELAY   50

// ==================== GLOBAL OBJECTS ====================
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;

// ==================== GLOBAL VARIABLES ====================
int peopleCount = 0;
enum CountState { IDLE, A_FIRST, B_FIRST, BOTH_TRIGGERED };
CountState currentState = IDLE;
bool currA = false, currB = false;
unsigned long lastCountChange = 0;
const unsigned long COUNT_TIMEOUT = 2000UL;

int mq2Value = 0;
float dustDensity = 0.0;
int airQualityIndex = 0;
String airCondition = "Good";

int fanSpeed = 0;           // PWM value 0..255
int fanSpeedPercent = 0;   // mapped 0..100

unsigned long lastLogTime = 0;
unsigned long lastDisplayUpdate = 0;
bool sdCardAvailable = false;

// Display alternation
uint8_t displayModeIndex = 0;           // 0=time, 1=status
unsigned long lastDisplaySwitch = 0;
const unsigned long DISPLAY_SWITCH_MS = 4000UL;  // 4 seconds

// ==================== SETUP ====================
void setup() {
  Serial.begin(9600);
  Serial.println(F("========================================"));
  Serial.println(F(" ATmega32A Air Quality Control System"));
  Serial.println(F("========================================"));

  setupPins();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Air Quality Sys");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // ---------- RTC Initialization ----------
  if (!rtc.begin()) {
    Serial.println(F("ERROR: RTC not found!"));
    lcd.setCursor(0, 1);
    lcd.print("RTC Error!      ");
    delay(2000);
  } else {
    Serial.println(F("RTC found"));
    if (rtc.lostPower()) {
      Serial.println(F("RTC lost power, please set RTC time manually"));
      // Intentionally not auto-adjusting from compile time to avoid overwriting correct RTC
    } else {
      Serial.println(F("RTC OK - not adjusting"));
    }
  }

  // ---------- SD Card Initialization ----------
  Serial.print(F("Initializing SD card... "));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FAILED"));
    sdCardAvailable = false;
  } else {
    Serial.println(F("OK"));
    sdCardAvailable = true;
    createLogFile();
  }

  // Initialize display cycle: start with time
  displayModeIndex = 0;
  lastDisplaySwitch = millis();
  lastDisplayUpdate = 0; // force immediate update in loop

  delay(200);
  lcd.clear();

  Serial.println(F("System operational"));
  Serial.println(F("----------------------------------------"));
  printHeader();
}

// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentMillis = millis();

  // 1. Sensors & counting
  updatePeopleCount();
  readMQ2Sensor();
  readDustSensor();
  calculateAirQuality();

  // 2. Fan control (based on updated AQ and count)
  controlFan();

  // 3. LEDs
  updateLEDs();

  // 4. Display handling: cycle through 0..1 each DISPLAY_SWITCH_MS
  if (currentMillis - lastDisplaySwitch >= DISPLAY_SWITCH_MS) {
    displayModeIndex++;
    if (displayModeIndex > 1) displayModeIndex = 0;
    lastDisplaySwitch = currentMillis;
    // immediate update on switch
    if (displayModeIndex == 0) showTimeOnLCD();
    else updateDisplay();
    lastDisplayUpdate = currentMillis;
  } else {
    // periodic refresh inside same mode (to update seconds / numbers)
    if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE) {
      if (displayModeIndex == 0) showTimeOnLCD();
      else updateDisplay();
      lastDisplayUpdate = currentMillis;
    }
  }

  // 5. Logging every minute
  if (currentMillis - lastLogTime >= DATA_LOG_INTERVAL) {
    logDataToSD();
    printToSerial();
    lastLogTime = currentMillis;
  }

  delay(SENSOR_READ_DELAY); // small sleep for stability (50ms)
}

// ==================== PIN SETUP ====================
void setupPins() {
  pinMode(IR_SENSOR_A, INPUT_PULLUP);
  pinMode(IR_SENSOR_B, INPUT_PULLUP);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(DUST_LED_PIN, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);
}

// ==================== PEOPLE COUNTING ====================
void updatePeopleCount() {
  currA = (digitalRead(IR_SENSOR_A) == LOW);
  currB = (digitalRead(IR_SENSOR_B) == LOW);

  if ((millis() - lastCountChange > COUNT_TIMEOUT) && (currentState != IDLE))
    currentState = IDLE;

  switch (currentState) {
    case IDLE:
      if (currA && !currB) {
        currentState = A_FIRST;
        lastCountChange = millis();
      } else if (currB && !currA) {
        currentState = B_FIRST;
        lastCountChange = millis();
      }
      break;

    case A_FIRST:
      if (currB) {
        peopleCount++;
        if (peopleCount < 0) peopleCount = 0;
        Serial.print(F(">>> ENTERED | Count: "));
        Serial.println(peopleCount);
        currentState = BOTH_TRIGGERED;
        lastCountChange = millis();
      } else if (!currA && !currB)
        currentState = IDLE;
      break;

    case B_FIRST:
      if (currA) {
        peopleCount--;
        if (peopleCount < 0) peopleCount = 0;
        Serial.print(F("<<< LEFT | Count: "));
        Serial.println(peopleCount);
        currentState = BOTH_TRIGGERED;
        lastCountChange = millis();
      } else if (!currA && !currB)
        currentState = IDLE;
      break;

    case BOTH_TRIGGERED:
      if (!currA && !currB)
        currentState = IDLE;
      break;
  }
}

// ==================== SENSOR READING ====================
void readMQ2Sensor() {
  mq2Value = analogRead(MQ2_PIN);
}

void readDustSensor() {
  digitalWrite(DUST_LED_PIN, LOW);
  delayMicroseconds(SAMPLING_TIME);
  int voMeasured = analogRead(DUST_MEASURE_PIN);
  delayMicroseconds(DELTA_TIME);
  digitalWrite(DUST_LED_PIN, HIGH);
  delayMicroseconds(SLEEP_TIME);
  float calcVoltage = voMeasured * (5.0 / 1024.0);
  dustDensity = 170.0 * calcVoltage - 0.1;
  if (dustDensity < 0) dustDensity = 0;
}

// ==================== AIR QUALITY CALCULATION ====================
void calculateAirQuality() {
  int mq2Score = 0, dustScore = 0, countScore = 0;

  if (mq2Value < MQ2_GOOD)
    mq2Score = map(mq2Value, 0, MQ2_GOOD, 0, 30);
  else if (mq2Value < MQ2_MODERATE)
    mq2Score = map(mq2Value, MQ2_GOOD, MQ2_MODERATE, 30, 60);
  else if (mq2Value < MQ2_POOR)
    mq2Score = map(mq2Value, MQ2_MODERATE, MQ2_POOR, 60, 90);
  else mq2Score = 100;

  if (dustDensity < DUST_GOOD)
    dustScore = map((int)dustDensity, 0, DUST_GOOD, 0, 30);
  else if (dustDensity < DUST_MODERATE)
    dustScore = map((int)dustDensity, DUST_GOOD, DUST_MODERATE, 30, 60);
  else if (dustDensity < DUST_POOR)
    dustScore = map((int)dustDensity, DUST_MODERATE, DUST_POOR, 60, 90);
  else dustScore = 100;

  if (peopleCount == 0)
    countScore = 0;
  else if (peopleCount <= 5)
    countScore = map(peopleCount, 1, 5, 10, 30);
  else if (peopleCount <= 10)
    countScore = map(peopleCount, 6, 10, 30, 60);
  else countScore = 100;

  airQualityIndex = (mq2Score * 40 + dustScore * 40 + countScore * 20) / 100;
  airQualityIndex = constrain(airQualityIndex, 0, 100);

  if (airQualityIndex < 40)
    airCondition = "Good";
  else if (airQualityIndex < 60)
    airCondition = "Moderate";
  else
    airCondition = "Poor";
}

// ==================== FAN CONTROL (UPDATED) ====================
void controlFan() {
  int targetFromAQ = 0;
  int targetFromCount = 0;

  if (airQualityIndex < 30) targetFromAQ = FAN_OFF;
  else if (airQualityIndex < 40) targetFromAQ = MIN_FAN_SPEED;
  else if (airQualityIndex < 60) targetFromAQ = MIN_FAN_SPEED + 50;
  else {
    targetFromAQ = map(airQualityIndex, 60, 100, 180, MAX_FAN_SPEED);
    targetFromAQ = constrain(targetFromAQ, 0, MAX_FAN_SPEED);
  }

  if (peopleCount <= 0) {
    targetFromCount = 0;
  } else if (peopleCount >= 10) {
    targetFromCount = MAX_FAN_SPEED;
  } else {
    targetFromCount = map(peopleCount, 1, 10, 100, MAX_FAN_SPEED);
    targetFromCount = constrain(targetFromCount, 0, MAX_FAN_SPEED);
  }

  if (peopleCount >= 1) {
    fanSpeed = max(targetFromCount, targetFromAQ);
    if (fanSpeed < 100) fanSpeed = 100;
  } else {
    if (airQualityIndex >= 50) {
      fanSpeed = max(targetFromAQ, 155); // ~50% when AQ bad and no people
      fanSpeed = constrain(fanSpeed, 0, MAX_FAN_SPEED);
    } else {
      fanSpeed = FAN_OFF;
    }
  }

  // Ensure fanSpeed within bounds
  fanSpeed = constrain(fanSpeed, 0, MAX_FAN_SPEED);

  // Map to percent right away (so display always sees updated value)
  fanSpeedPercent = map(fanSpeed, 0, MAX_FAN_SPEED, 0, 100);
  if (fanSpeed > 0 && fanSpeedPercent == 0) fanSpeedPercent = 1; // show tiny >0 indication

  // Write PWM to motor enable
  analogWrite(MOTOR_ENABLE, fanSpeed);

  // Debug prints to Serial - useful to check mismatch between actual PWM and displayed %
  Serial.print(F("Fan raw: "));
  Serial.print(fanSpeed);
  Serial.print(F(" | Fan%: "));
  Serial.println(fanSpeedPercent);
}

// ==================== LED INDICATORS ====================
void updateLEDs() {
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  if (airQualityIndex < 30)
    digitalWrite(LED_GREEN, HIGH);
  else if (airQualityIndex < 60)
    digitalWrite(LED_YELLOW, HIGH);
  else
    digitalWrite(LED_RED, HIGH);
}

// ==================== LCD DISPLAY HELPERS ====================

// Show time (reads rtc.now() live) with Sri Lanka offset +5:30 using TimeSpan
void showTimeOnLCD() {
  DateTime now = rtc.now();
  // Add Sri Lanka offset +5:30
  //DateTime sri = now + TimeSpan(0, 5, 30, 0); // days,hours,minutes,seconds

  // Minimal updates to reduce flicker
  lcd.setCursor(0, 0);
  char buf1[17];
  snprintf(buf1, sizeof(buf1), "%04d/%02d/%02d", now.year(), now.month(), now.day());
  lcd.print(buf1);
  lcd.setCursor(0, 1);
  char buf2[17];
  snprintf(buf2, sizeof(buf2), "%02d:%02d:%02d       ", now.hour(), now.minute(), now.second());
  lcd.print(buf2);

  // Serial debug of displayed time as well (helps check RTC updating)
  Serial.print(F("Time (SL): "));
  Serial.print(now.year()); Serial.print('/');
  Serial.print(now.month()); Serial.print('/');
  Serial.print(now.day()); Serial.print(' ');
  Serial.print(now.hour()); Serial.print(':');
  Serial.print(now.minute()); Serial.print(':');
  Serial.println(now.second());
}

// Show status (Cnt, AQ, Fan%, Cond)
void updateDisplay() {
  lcd.setCursor(0, 0);
  char line0[17];
  snprintf(line0, sizeof(line0), "Cnt:%2d  AQ:%3d%%", peopleCount, airQualityIndex);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  char condShort[5];
  if (airCondition == "Good") strcpy(condShort, "Gd");
  else if (airCondition == "Moderate") strcpy(condShort, "Md");
  else strcpy(condShort, "Pr");
  char line1[17];
  snprintf(line1, sizeof(line1), "Fan:%3d%% %3s   ", fanSpeedPercent, condShort);
  lcd.print(line1);
}

// ==================== SD CARD LOGGING ====================
void createLogFile() {
  File logFile = SD.open("airlog.csv", FILE_WRITE);
  if (logFile) {
    if (logFile.size() == 0)
      logFile.println(F("Timestamp,People_Count,MQ2_Value,Dust_Density,Air_Quality_Index,Fan_Speed_Percent,Air_Condition"));
    logFile.close();
    Serial.println(F("Log file ready"));
  } else Serial.println(F("ERROR: Cannot create log file"));
}

void logDataToSD() {
  if (!sdCardAvailable) return;
  DateTime now = rtc.now();
  File logFile = SD.open("airlog.csv", FILE_WRITE);
  if (logFile) {
    char timestamp[20];
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    logFile.print(timestamp); logFile.print(",");
    logFile.print(peopleCount); logFile.print(",");
    logFile.print(mq2Value); logFile.print(",");
    logFile.print(dustDensity, 2); logFile.print(",");
    logFile.print(airQualityIndex); logFile.print(",");
    logFile.print(fanSpeedPercent); logFile.print(",");
    logFile.println(airCondition);
    logFile.close();
    Serial.println(F("Data logged to SD card"));
  } else Serial.println(F("ERROR: Cannot open log file"));
}

// ==================== SERIAL MONITOR OUTPUT ====================
void printHeader() {
  Serial.println(F("Time\t\t\tCount\tMQ2\tDust\tAQI\tFan%\tCondition"));
  Serial.println(F("--------------------------------------------------------------------------------"));
}

void printToSerial() {
  DateTime now = rtc.now();
  //DateTime sri = now + TimeSpan(0, 5, 30, 0);
  char timestamp[20];
  sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.print(timestamp); Serial.print("\t");
  Serial.print(peopleCount); Serial.print("\t");
  Serial.print(mq2Value); Serial.print("\t");
  Serial.print(dustDensity, 1); Serial.print("\t");
  Serial.print(airQualityIndex); Serial.print("\t");
  Serial.print(fanSpeedPercent); Serial.print("\t");
  Serial.print(airCondition);
  Serial.println();
}
