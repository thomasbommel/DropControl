#include <Arduino.h>
// ======== LIBRARIES =======
#include<stdlib.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h> 
 
// ======= DEVICE CONFIG ========
#define DEVICE_NAME         "FMP"
#define MIN_TIME_BETWEEN_INTERRUPTS 500

// ========= PIN DEFINE  ========
#define SD_CHIPSELECT_PIN       15
#define SPEED_PIN               A7
#define TELTONIKA_PIN           A6
#define INTERRUPT_PIN           A17
#define INCREASE_DISTANCE_PIN   A9
#define DECREASE_DISTANCE_PIN   A8
#define ADJUSTMENT_PIN          A0
#define LED_PIN                 2

// ========= OBJECTS ============
TinyGPSPlus gps;
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ========= CONSTANTS ===========
#define EARTH_RADIUS_METERS 6372795.0
#define GPS_SERIAL          Serial3
#define GPS_BAUDRATE        9600
#define AREA_COUNT          5000

// ========= VARIABLES ===========
float AREAS_TO_EXCLUDE[AREA_COUNT][4];
long areas_to_excluce_count = 0;
File excludeFile;
unsigned long previousMillis = 0;
const long excludeCheckInterval = 1000;

volatile boolean baitWasDropped = false;
long dropCount = 0;
double distanceBetweenDrops = 50.0;
int currentSpeed = 200; // 0-255
double ms = 10;
char currentDate[20]; //for filename
double lastLat = -1;
double lastLng = -1;

double adjustMentMeters = 0;
boolean first = true;

double lastDistanceBetweenDrops = 50;

boolean shouldCheckExclude = true;
unsigned long lastTeltonikaActivationTime = 0;

// ========= SETUP ==============
void setup() {
  Serial.begin(9600);
  delay(2000);
  initPins();
  stopMotor();
  initGPS();
  initLCD();
  initSD();
  initAreasToExclude();
  lcd.clear();

  if (digitalRead(INCREASE_DISTANCE_PIN) == LOW &&  digitalRead(DECREASE_DISTANCE_PIN) == LOW) {
    shouldCheckExclude = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("SKIP EXCLUDE"));
    delay(3000);
  }
}

void handleInterrupt()
{
  static long last_interrupt_time = 0;
  long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > MIN_TIME_BETWEEN_INTERRUPTS) {
    baitWasDropped = true;
    last_interrupt_time = interrupt_time;
  }
}

// ========= LOOP ===============
void loop() {
  if (gps.speed.isValid()) {
    ms = double( gps.speed.mps());
  }
  smartDelay(30);
  if (gps.sentencesWithFix() == 0 || !gps.speed.isValid() ||  !gps.location.isValid()) {
    //searching gps
    lcd.setCursor(0, 0);
    lcd.print(F("SEARCHING GPS..."));
    delay(40);
    lcd.setCursor(0, 0);
    lcd.print(F("SEARCHING GPS   "));
    lcd.setCursor(0, 1);
    lcd.print("d:");
    lcd.print(distanceBetweenDrops, 0);
    lcd.print("  adj:");
    lcd.print(adjustMentMeters, 1);
    delay(40);
  } else if (baitWasDropped) {
    if (first) {
      first = false;
      lcd.clear();
    }

    detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
    baitWasDropped = false;
    dropCount = dropCount + 1;
    adjustSpeed();
    savePoint();
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);
  } else {
    printStatsToLCD();
  }

  //adjustMentMeters = (map(analogRead(ADJUSTMENT_PIN), 0, 1023, -100, 100) / 10.0);

  unsigned long currentMillis = millis();

  if ( (currentMillis - lastTeltonikaActivationTime) <= 300) {
    digitalWrite(TELTONIKA_PIN, HIGH);
  } else {
    digitalWrite(TELTONIKA_PIN, LOW);
  }

  if (currentMillis - previousMillis >= excludeCheckInterval) {
    previousMillis = currentMillis;
    checkForExclude();
  }

  if (ms > 8 && gps.location.isValid()) {
    analogWrite(SPEED_PIN, currentSpeed);
  } else {
    analogWrite(SPEED_PIN, 0);
  }

  increaseOrDecreaseDistanceIfButtonPressed();
}

void checkForExclude() {
  if (shouldCheckExclude) {
    float point[] = {(float)gps.location.lat(), (float)gps.location.lng()};
    boolean shouldExcludeThisPoint = shouldExcludePoint(point);

    while (shouldExcludeThisPoint) {
      analogWrite(SPEED_PIN, 0);
      point[0] = (float)gps.location.lat();
      point[1] = (float)gps.location.lng();
      shouldExcludeThisPoint = shouldExcludePoint(point);
      smartDelay(50);
      digitalWrite(LED_PIN, HIGH);
    }
    digitalWrite(LED_PIN, LOW);
  }
}

void initAreasToExclude() {
  excludeFile = SD.open("exclude.txt");
  if (excludeFile) {
    String line = "";
    while (excludeFile.available()) {
      char c = excludeFile.read();
      if (c != '\n') {
        line += c;
      } else {
        computeArea( areas_to_excluce_count, line);
        areas_to_excluce_count++;
        line = "";
      }
    }
    excludeFile.close();
  } else {
    //    Serial.println("error opening exclude.txt");
    lcd.setCursor(0, 0);
    lcd.print("no exclude file");
    lcd.setCursor(0, 1);
    lcd.print("no areas excl.");
    delay(3000);
    lcd.clear();
    delay(100);
  }
}

void computeArea(long index, String line) {
  String nwLat = line.substring(0, 20);
  // Serial.print(nwLat.toFloat(), 10);
  // Serial.print("x");

  String nwLng = line.substring(20, 41);
  // Serial.print(nwLng.toFloat(), 10);
  // Serial.print("x");

  String seLat = line.substring(41, 62);
  //  Serial.print(seLat.toFloat(), 10);
  //  Serial.print("x");

  String seLng = line.substring(62, 83);
  // Serial.print(seLng.toFloat(), 10);
  // Serial.println("x");

  AREAS_TO_EXCLUDE[index][0] = nwLat.toFloat();
  AREAS_TO_EXCLUDE[index][1] = nwLng.toFloat();
  AREAS_TO_EXCLUDE[index][2] = seLat.toFloat();
  AREAS_TO_EXCLUDE[index][3] = seLng.toFloat();
}

boolean shouldExcludePoint(float point[2]) {
  for (long i = 0; i < areas_to_excluce_count; i++) {
    //    printArea(excludeAreas[i]);
    //    Serial.print(point[0], 6);
    //    Serial.print(" ");
    //    Serial.print(point[1], 6);
    //    Serial.print(" ");
    //    Serial.println(isContainedInArea(point, excludeAreas[i]));

    if (isContainedInArea(point, AREAS_TO_EXCLUDE[i])) {
      return true;
    }
  }
  return false;
}

/**
   point is lat / lng , area is nwlat nwlng selat selng
*/
boolean isContainedInArea(float point[2], float area[4]) {
  float pointLat = point[0];
  float pointLng = point[1];
  float areaNwLat = area[0];
  float areaNwLng = area[1];
  float areaSeLat = area[2];
  float areaSeLng = area[3];
  return (pointLat < areaNwLat && pointLat > areaSeLat  && pointLng > areaNwLng && pointLng < areaSeLng);
}

void printStatsToLCD() {
  Serial.print(F("dropCount: "));
  Serial.println(dropCount);
  Serial.print(F(", currentSpeed: "));
  Serial.print(currentSpeed);
  Serial.print(F(", distanceBetweenDrops: "));
  Serial.print(distanceBetweenDrops);
  Serial.print(F(", ms: "));
  Serial.println(ms);

  lcd.setCursor(0, 0);
  lcd.print("# ");
  lcd.print(dropCount);

  lcd.setCursor(8, 0);
  lcd.print("kmh:");
  lcd.print(ms * 3.6, 0);
  lcd.print(" ");

  lcd.setCursor(0, 1);
  lcd.print(distanceBetweenDrops, 0);
  lcd.print(" ");

  lcd.setCursor(5, 1);
  lcd.print(lastDistanceBetweenDrops, 0);
  lcd.print(" ");

  lcd.setCursor(9, 1);
  lcd.print(map(currentSpeed, 0, 255, 0, 100));
  lcd.print(" ");
}

void adjustSpeed() {
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  double distance = TinyGPSPlus::distanceBetween(lastLat, lastLng, latitude, longitude);

  handleDifference(distanceBetweenDrops, distance);
  lastLat = latitude;
  lastLng = longitude;
}

void handleDifference(double wanted, double actual) {
  wanted = wanted + adjustMentMeters;

  lastDistanceBetweenDrops = actual;

  double difference = wanted - actual;
  double factor;
  if (difference < 1) {
    factor = 2.0; // get faster
  } else {
    factor = -2.0; // slow down
  }
  int changeAmount = 1;
  changeAmount = abs(round(( (currentSpeed + 40) / 150.0) * difference));

  changeAmount = constrain(changeAmount, 1, 30);
  changeAmount *= factor;

  Serial.print(F("handleDifference - old speed: "));
  Serial.print(currentSpeed);
  Serial.print(F(", changeAmount: "));
  Serial.println(changeAmount);
  currentSpeed += changeAmount;

  currentSpeed = constrain(currentSpeed, 70, 255);
}

void increaseOrDecreaseDistanceIfButtonPressed() {
  if (digitalRead(INCREASE_DISTANCE_PIN) == LOW && distanceBetweenDrops <= 500) {
    // increase the distance between the drops by 1m
    distanceBetweenDrops += 1.0;

    Serial.print("increased dist to ");
    Serial.println(distanceBetweenDrops);
    delay(20);
  }
  if (digitalRead(DECREASE_DISTANCE_PIN) == LOW && distanceBetweenDrops >= 1) {
    // decrease the distance between the drops by 1m
    distanceBetweenDrops -= 1.0;

    Serial.print("decreased dist to ");
    Serial.println(distanceBetweenDrops);
    delay(20);
  }
}

void initPins() {
  pinMode(INCREASE_DISTANCE_PIN, INPUT_PULLUP);
  pinMode(DECREASE_DISTANCE_PIN, INPUT_PULLUP);
  pinMode(SPEED_PIN, OUTPUT);
  digitalWrite(SPEED_PIN, LOW);
  pinMode(TELTONIKA_PIN, OUTPUT);
  digitalWrite(TELTONIKA_PIN, HIGH);
  delay(500);
  digitalWrite(TELTONIKA_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  blinkLed();
  digitalWrite(LED_PIN, LOW);


  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);
  pinMode(ADJUSTMENT_PIN, INPUT);
}

void blinkLed() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

void stopMotor() {
  analogWrite(SPEED_PIN, 0);
}

void initGPS() {
  GPS_SERIAL.begin(GPS_BAUDRATE);
  delay(100);
  GPS_SERIAL.println("$PMTK251,57600*2C"); // change baud rate
  delay(100);
  GPS_SERIAL.end();
  delay(1000);
  GPS_SERIAL.begin(57600);
  delay(500);
  //GPS_SERIAL.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"); //only get some sentences
  GPS_SERIAL.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"); //only get some sentences
  delay(1000);
  GPS_SERIAL.println("$PMTK220,100*2F"); // set update Rate to 10Hz
  delay(1000);
}

void initLCD() {
  lcd.begin();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(DEVICE_NAME);
  lcd.print("-006");
  delay(2000);
}

void initSD() {
  Serial.print(F("Init. SD card..."));
  while (!SD.begin(SD_CHIPSELECT_PIN)) {
    Serial.println(F("SD init failed!"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD ERROR");
    delay(2000);
  }
  Serial.println(F("SD init done."));
}

void getFileName() {
  if (currentDate[0] == NULL) {//DO NOT CHANGE THIS LINE!!!!
    //Serial.println("getFileName");
    sprintf(currentDate, "%s_%02d%02d.txt", DEVICE_NAME,  gps.date.month(),  gps.date.day());
  }
}

void setNewFileName() {
  static int tries;
  tries = tries + 1;
  if (tries < 10) {
    tries = 10;
  }
  if (tries > 99) {
    tries = 99;
  }
  sprintf(currentDate, "%d_%02d%02d.txt", tries,  gps.date.month(),  gps.date.day());
}

static void smartDelay(unsigned long milliseconds) {               // This custom version of delay() ensures that the gps object is being "fed".
  unsigned long start = millis();
  do {
    yield();
    while (!GPS_SERIAL.available()) {}
    while (GPS_SERIAL.available()) {
      gps.encode(GPS_SERIAL.read());
    }
  } while (millis() - start < milliseconds);
}

void savePoint() {
  delay(10);//needed because garbage lines were written to the sd (caused by motor)
  getFileName();
  File dataFile = SD.open(currentDate, FILE_WRITE);
  if (dataFile) { // valid file
    if (gps.time.hour() < 10) {
      dataFile.print("0");
    }
    dataFile.print(gps.time.hour());
    dataFile.print(":");

    if (gps.time.minute() < 10) {
      dataFile.print("0");
    }
    dataFile.print(gps.time.minute());
    dataFile.print(":");

    if (gps.time.second() < 10) {
      dataFile.print("0");
    }
    dataFile.print(gps.time.second());
    dataFile.print(",");

    //date
    if (gps.date.day() < 10) {
      dataFile.print("0");
    }
    dataFile.print(gps.date.day());
    dataFile.print(".");

    if (gps.date.month() < 10) {
      dataFile.print("0");
    }
    dataFile.print(gps.date.month());
    dataFile.print(".");

    dataFile.print(gps.date.year() - 2000);
    dataFile.print(",");
    dataFile.print("  ");
    dataFile.print(gps.location.lat(), 10);
    dataFile.print(", ");
    dataFile.print(gps.location.lng(), 10);
    dataFile.print("  ");
    dataFile.println("");

    lastTeltonikaActivationTime = millis();

    Serial.println(F(" drop was finished"));
  } else {
    Serial.print(F("-- ERROR was not able to open file: "));
    Serial.println(currentDate);
    setNewFileName();
    delay(50);
  }
  dataFile.close();
}
