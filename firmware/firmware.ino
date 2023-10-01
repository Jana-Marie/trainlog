#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include "SD.h"
#include "FS.h"

// GPS
static const int RXPin = 16, TXPin = 17;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXPin, TXPin);

// LED
uint8_t led[8] = {25, 26, 27, 14, 12};

// BNO
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 500;
sensors_event_t orientationData , linearAccelData, angVelData, magData, gravData;
uint8_t bnoTemp;
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  // LEDs
  for (uint8_t i = 0; i <= 5; i++) {
    pinMode(led[i], OUTPUT);
    digitalWrite(led[i], LOW);
  }

  // Serial
  Serial.begin(115200);

  // SD
  pinMode(5, OUTPUT);
  SD.begin(5);

  // GPS
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); // GPS enable
  gpsSerial.begin(9600); // connect gps sensor

  // BNO
  bno.begin();
  bno.setExtCrystalUse(true);
}

void loop() {

  smartDelay(500);

  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&gravData, Adafruit_BNO055::VECTOR_GRAVITY);
  bnoTemp = bno.getTemp();

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  digitalWrite(led[3], gps.satellites.value() > 0 ? 1 : 0);
  digitalWrite(led[1], write_to_SD());
  delay(2);
  digitalWrite(led[3], LOW);
  digitalWrite(led[1], LOW);
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available()) {
      digitalWrite(led[4], HIGH);
      gps.encode(gpsSerial.read());
    }
    digitalWrite(led[4], LOW);
  } while (millis() - start < ms);
  delay(2);
  digitalWrite(led[4], LOW);
}


bool write_to_SD() {
  String dataString = String(gps.satellites.value()) +
                      "," + String(gps.hdop.hdop()) +
                      "," + String(gps.location.lat()) +
                      "," + String(gps.location.lng()) +
                      "," + String(gps.location.age()) +
                      "," + String(gps.altitude.meters()) +
                      "," + String(gps.course.deg()) +
                      "," + String(gps.speed.kmph()) +
                      "," + gps.date.year() + "-" + gps.date.month() + "-" + gps.date.day() +
                      "," + gps.time.hour() + ":" + gps.time.minute() + ":" + gps.time.second() +
                      "," + String(xPos) +
                      "," + String(yPos) +
                      "," + String(headingVel) +
                      "," + String(bnoTemp) +
                      "," + String(orientationData.orientation.x) +
                      "," + String(orientationData.orientation.y) +
                      "," + String(orientationData.orientation.z) +
                      "," + String(angVelData.gyro.x) +
                      "," + String(angVelData.gyro.y) +
                      "," + String(angVelData.gyro.z) +
                      "," + String(linearAccelData.acceleration.x) +
                      "," + String(linearAccelData.acceleration.y) +
                      "," + String(linearAccelData.acceleration.x) +
                      "," + String(magData.magnetic.x) +
                      "," + String(magData.magnetic.y) +
                      "," + String(magData.magnetic.x) +
                      "," + String(gravData.acceleration.x) +
                      "," + String(gravData.acceleration.y) +
                      "," + String(gravData.acceleration.x) + 
                      "," + String(analogRead(36)) + "\r\n";

  Serial.print(dataString);

  File logFile = SD.open("/log.csv", FILE_APPEND);
  if (logFile) {
    logFile.print(dataString);
    logFile.close();
    return 1;
  }
  return 0;
}
