#include <Adafruit_ISM330DHCX.h>
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

#define NUMBER_SAMPLES 500

Adafruit_Sensor *gyro;
Adafruit_ISM330DHCX ism;
sensors_event_t event;

float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

bool rateIsHigh = false;
unsigned long startTime;
unsigned long checkDurationMillis = 2000;   // time that degree/s should be active before triggering
int accelThresh = 2;                        // Degrees/second speed that we need to trigger 

void setup()
{
    Serial.begin(115200);
    if (!ism.begin_SPI(5))
    {
        Serial.println("ISM330DHCX connection error");
    }
}

void loop()
{
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    ism.getEvent(&accel, &gyro, &temp);
    
    // Converts radian to degree
    gyro.gyro.x = (gyro.gyro.x * 57.2957795);// - 0.14;
    gyro.gyro.y = (gyro.gyro.y * 57.2957795);// - 1.0;
    gyro.gyro.z = (gyro.gyro.z * 57.2957795);// - 0.14;

    // Remove Zero Offset
    // gyro.gyro.x = gyro.gyro.x - 0.7;    //0.14;
    // gyro.gyro.y = gyro.gyro.y - 0.49;    //0.55;
    // gyro.gyro.z = gyro.gyro.z - 0.7;     //0.14;
    Serial.print("X: ");
    Serial.print(gyro.gyro.x);
    Serial.print("  Y: ");
    Serial.print(gyro.gyro.y);
    Serial.print("  Z: ");
    Serial.print(gyro.gyro.z);
    Serial.println();

    // check if movement is continuous for specified amount of time
    if((abs(gyro.gyro.x) > 2) || (abs(gyro.gyro.y) > 2) || (abs(gyro.gyro.z) > 2)){
        // Serial.println("TRIGGERED\n\n\n");
        rateIsHigh = true;
        if(!startTime){
            startTime = millis();
        } else if (millis() - startTime >= checkDurationMillis){
            Serial.println("Over 2 deg/s for at 1000 ms!!!!!!!!!!!\n\n\n\n\n\n");
            startTime = 0;
            rateIsHigh = false;
        }
    }
}