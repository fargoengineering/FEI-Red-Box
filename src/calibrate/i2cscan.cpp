#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#define NUMBER_SAMPLES 500

Adafruit_Sensor *gyro;
sensors_event_t event;

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

Adafruit_ISM330DHCX ism;

void setup(void)
{

  sensors_event_t accel;
  sensors_event_t gyroEvent;
  sensors_event_t temp;

  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println(F("Gyroscope Calibration!"));

  Serial.println("Looking for the gyro");
  // gyro = lab.getGyroscope();
  // if (! gyro) {
  //   Serial.println(F("Could not find a gyro, check wiring!"));
  //   while(1) delay(10);
  // }
  // gyro->printSensorDetails();
  // delay(100);
  if (!ism.begin_SPI(5))
  {
    Serial.println("ISM330DHCX connection error");
  }

  ism.getEvent(&accel, &gyroEvent, &temp);
  // gyro.printSensorDetails();
  //  gyro->printSensorDetails();

  min_x = max_x = event.gyro.x;
  min_y = max_y = event.gyro.y;
  // min_z = max_z = event.gyro.z;
  delay(10);

  Serial.println(F("Place gyro on flat, stable surface!"));

  Serial.print(F("Fetching samples in 3..."));
  delay(1000);
  Serial.print("2...");
  delay(1000);
  Serial.print("1...");
  delay(1000);
  Serial.println("NOW!");

  float x, y, z;
  for (uint16_t sample = 0; sample < NUMBER_SAMPLES; sample++)
  {
    // gyro->getEvent(&gyroEvent);
    ism.getEvent(&accel, &gyroEvent, &temp);
    x = gyroEvent.gyro.x;
    y = gyroEvent.gyro.y;
    z = gyroEvent.gyro.z;
    Serial.print(F("Gyro: ("));
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.print(")");

    min_x = min(min_x, x);
    min_y = min(min_y, y);
    min_z = min(min_z, z);

    max_x = max(max_x, x);
    max_y = max(max_y, y);
    max_z = max(max_z, z);

    mid_x = (max_x + min_x) / 2;
    mid_y = (max_y + min_y) / 2;
    mid_z = (max_z + min_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(mid_x, 4);
    Serial.print(", ");
    Serial.print(mid_y, 4);
    Serial.print(", ");
    Serial.print(mid_z, 4);
    Serial.print(")");

    Serial.print(F(" rad/s noise: ("));
    Serial.print(max_x - min_x, 3);
    Serial.print(", ");
    Serial.print(max_y - min_y, 3);
    Serial.print(", ");
    Serial.print(max_z - min_z, 3);
    Serial.println(")");
    delay(10);
  }
  Serial.println(F("\n\nFinal zero rate offset in radians/s: "));
  Serial.print(mid_x, 4);
  Serial.print(", ");
  Serial.print(mid_y, 4);
  Serial.print(", ");
  Serial.println(mid_z, 4);
}

void loop()
{
  delay(10);
}

// Final zero rate offset in radians/s:
// 0.0012 , -0.0024,  -0.0031