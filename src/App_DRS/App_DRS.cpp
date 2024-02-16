#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "esp_sleep.h"
#include <Adafruit_ISM330DHCX.h>
#include <WiFi.h>
#include <FastLED.h>
#include <TimedBlink.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#define EN_3V3_SW 32 // The 3.3V_SW regulator Enable pin is connected to D32
#define RGB_LED 26   // OpenLog ESP32 RGB LED is connected to D26
#define MAG_CS 27
#define CHARGE_PIN 33

TimedBlink statusled(26);
Adafruit_ISM330DHCX ISM;
SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048
uint8_t IMU_CS = 5;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

// Constant values
const char VERSION[] = "2024.01.25"; // This version: Jan 2024 - Dylan
const char BLE_Name[] = "FEI_IMU_BOX";
const int NUM_SERVICES = 1;
const int NUM_CHAR_PER_SERVICE = 2;
const int NUM_CHARS = NUM_SERVICES * NUM_CHAR_PER_SERVICE;
const long interval = 500; // interval to update BLE
int input_vals[3];

// Non-Constant values
int accel_thresh = 2;                
int battery = 100;
unsigned long time_thresh = 0;
bool rateIsHigh = false;
bool movement = false;
int movement_char = 0;
unsigned long checkDurationMillis = 2000;   // time that degree/s should be active before triggering
unsigned long previousMillis = 0;
int trigger_in = 0;
char notify_char[100];
float analog_vals[2];
bool is_triggered = false;
bool is_triggered_state = false;
unsigned long actTime;
unsigned long remTime;
unsigned long startTime = 0; // Store the start time
unsigned long switchStartTime = 0;
static int switchCaseValue = 0;
float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;

// BLE Variables
BLEServer *pServer;
BLEService *pServices[NUM_SERVICES];
BLECharacteristic *characteristics[NUM_CHARS];
const char *const service_ids[NUM_SERVICES] =
    {
        "00000000-0000-0000-0000-000000000001",
};
const char *characteristic_ids[NUM_CHARS] =
    {
        "00000000-0000-0000-0000-000000000001", // Read AI 1,2
        "00000000-0000-0000-0000-000000000002", // WRITE DO
};

void parseStringToIntArray(const char* inputString) {
  // Check if the input string starts with '<' and ends with '>'
  if (inputString[0] != '<' || inputString[strlen(inputString) - 1] != '>') {
    // Invalid format
    return;
  }

  // Remove the '<' and '>'
  char cleanString[strlen(inputString)];
  strncpy(cleanString, inputString + 1, strlen(inputString) - 2);
  cleanString[strlen(inputString) - 2] = '\0';

  // Tokenize the string using ","
  char* token = strtok(cleanString, ",");
  int i = 0;

  // Iterate through tokens and convert to integers
  while (token != NULL && i < 3) {
    input_vals[i] = atoi(token);
    i++;
    token = strtok(NULL, ",");
  }
}

void splitString(String inputString, int outputIntegers[], float outputFloats[], int& numIntegers, int& numFloats) {
  inputString.trim(); // Remove leading/trailing whitespaces
  
  // Check if the string starts with '<' and ends with '>'
  if (inputString.charAt(0) != '<' || inputString.charAt(inputString.length() - 1) != '>') {
    // Invalid format
    numIntegers = 0;
    numFloats = 0;
    return;
  }

  inputString = inputString.substring(1, inputString.length() - 1); // Remove '<' and '>'
  
  // Split the string by commas
  int commaIndex;
  int lastIndex = 0;
  numIntegers = 0;
  numFloats = 0;

  while ((commaIndex = inputString.indexOf(',', lastIndex)) != -1) {
    String valueString = inputString.substring(lastIndex, commaIndex);
    if (valueString.indexOf('.') != -1) {
      // Float value
      outputFloats[numFloats++] = valueString.toFloat();
    } else {
      // Integer value
      outputIntegers[numIntegers++] = valueString.toInt();
    }
    lastIndex = commaIndex + 1;
  }

  // Process the last value
  String lastValueString = inputString.substring(lastIndex);
  if (lastValueString.indexOf('.') != -1) {
    // Float value
    outputFloats[numFloats++] = lastValueString.toFloat();
  } else {
    // Integer value
    outputIntegers[numIntegers++] = lastValueString.toInt();
  }
}



class CharacteristicCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();
        std::string UID = (pCharacteristic->getUUID()).toString();
        Serial.printf("\nReceived Data from BLE: %s", value.c_str());
        Serial.printf("\nCharacteristic: %s", UID.c_str());

        parseStringToIntArray(value.c_str());

        Serial.printf("Trigger: %d\n",input_vals[0]);
        Serial.printf("d/s thresh: %d\n",input_vals[1]);
        Serial.printf("ms thresh: %d\n",input_vals[2]);

        trigger_in = input_vals[0];
        accel_thresh = input_vals[1];
        time_thresh = input_vals[2];

        // Reset loop
        is_triggered = false;
        movement_char = 0;
        movement = false;
    }
};

void setBLE()
{
    // To save values in string
    char analog1Str[10] = "";
    char analog2Str[10] = "";

    // Converting to strings
    dtostrf(static_cast<float>(movement_char),1,2,analog1Str);
    dtostrf(battery,1,2,analog2Str);

    // Combining to single string
    // snprintf(notify_char, sizeof(notify_char), "<%s,%s>", analog1Str, analog2Str);   // show flag and battery status
    snprintf(notify_char, sizeof(notify_char), "<%s>", analog1Str);                     // Only show flag

    // Notify string
    characteristics[0]->setValue(notify_char);
    characteristics[0]->notify();
}

void imusetup()
{
    SPI.begin();

    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    if (!ISM.begin_SPI(5))
    {
        Serial.println("ISM330DHCX connection error");
    }
}

void setupfuel()
{
    // Set up the MAX17048 LiPo fuel gauge:
    if (lipo.begin() == false) // Connect to the MAX17048 using the default wire port
    {
        Serial.println(F("MAX17048 not detected. Please check wiring. Freezing."));
        while (1)
            ;
    }

    // Just because we can, let's reset the MAX17048
    Serial.println(F("Resetting the MAX17048..."));
    delay(1000); // Give it time to get its act back together

    // Read and print the reset indicator
    Serial.print(F("Reset Indicator was: "));
    bool RI = lipo.isReset(true); // Read the RI flag and clear it automatically if it is set
    Serial.println(RI);           // Print the RI
    // If RI was set, check it is now clear
    if (RI)
    {
        Serial.print(F("Reset Indicator is now: "));
        RI = lipo.isReset(); // Read the RI flag
        Serial.println(RI);  // Print the RI
    }

    // Read and print the device ID
    Serial.print(F("Device ID: 0x"));
    uint8_t id = lipo.getID(); // Read the device ID
    if (id < 0x10)
        Serial.print(F("0")); // Print the leading zero if required
    Serial.println(id, HEX);  // Print the ID as hexadecimal

    // Read and print the device version
    Serial.print(F("Device version: 0x"));
    uint8_t ver = lipo.getVersion(); // Read the device version
    if (ver < 0x10)
        Serial.print(F("0")); // Print the leading zero if required
    Serial.println(ver, HEX); // Print the version as hexadecimal

    // Read and print the battery threshold
    Serial.print(F("Battery empty threshold is currently: "));
    Serial.print(lipo.getThreshold());
    Serial.println(F("%"));

    // Read and print the high voltage threshold
    Serial.print(F("High voltage threshold is currently: "));
    float highVoltage = ((float)lipo.getVALRTMax()) * 0.02; // 1 LSb is 20mV. Convert to Volts.
    Serial.print(highVoltage, 2);
    Serial.println(F("V"));

    // Read and print the low voltage threshold
    Serial.print(F("Low voltage threshold is currently: "));
    float lowVoltage = ((float)lipo.getVALRTMin()) * 0.02; // 1 LSb is 20mV. Convert to Volts.
    Serial.print(lowVoltage, 2);
    Serial.println(F("V"));
}

void setup()
{
    Serial.begin(115200);

    Serial.println("ESP32 Starting!");

    BLEDevice::init(BLE_Name);

    // Create services and characteristics
    pServer = BLEDevice::createServer();
    BLEService *services[NUM_SERVICES];

    // Read/Write Service
    services[0] = pServer->createService(service_ids[0]);

    // Read characteristics
    int char_index = 0;
    characteristics[char_index] = services[0]->createCharacteristic(characteristic_ids[char_index], BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    characteristics[char_index]->addDescriptor(new BLE2902());

    // Write characteristics
    char_index = 1;
    characteristics[char_index] = services[0]->createCharacteristic(characteristic_ids[char_index], BLECharacteristic::PROPERTY_WRITE);
    characteristics[char_index]->addDescriptor(new BLE2902());
    characteristics[char_index]->setCallbacks(new CharacteristicCallback());

    // Start services
    Serial.println("Starting service: " + String(service_ids[0]));
    services[0]->start();

    // advertising services
    Serial.println("Advertising service: " + String(service_ids[0]));
    pServer->getAdvertising()->addServiceUUID(services[0]->getUUID());

    delay(1000);
    pServer->getAdvertising()->start();

    Wire.begin();
    imusetup();
    if (lipo.begin() == false){
        lipo.enableDebugging();
        Serial.println("Unable to read battery.");
    }else{
        Serial.print(F("Battery empty threshold is currently: "));
        battery = lipo.getThreshold();
        Serial.print(lipo.getThreshold());
        Serial.println(F("%"));
    }

    pinMode(EN_3V3_SW, OUTPUT);
    pinMode(33, INPUT);
    digitalWrite(EN_3V3_SW, HIGH);

    delay(30); // sanity delay

    statusled.blink(1000, 500, CRGB::Red);

    Serial.println(F("Setup Done. Starting LOOP."));
}

void loop()
{
    unsigned long currentMillis = millis();

    // Updated BLE Every 1000 milliseconds
    if (currentMillis - previousMillis >= interval)
    {
        setBLE(); // configure and set all 4 characteristics
        pServer->getAdvertising()->start();
        previousMillis = currentMillis;
    }

    if (trigger_in > 0)
    {
       switchCaseValue = 1;
    }
    else if (trigger_in <= 0){
        switchCaseValue = 0;
        is_triggered = false;
        movement_char = 0;
        movement = false;
    }

    if (switchCaseValue > 0)
    {
        ISM.getEvent(&accel,&gyro,&temp);

        // Converts radian to degree
        gyro.gyro.x = (gyro.gyro.x * 57.2957795);// - 0.14;
        gyro.gyro.y = (gyro.gyro.y * 57.2957795);// - 1.0;
        gyro.gyro.z = (gyro.gyro.z * 57.2957795);// - 0.14;

        // Serial.print("X: ");
        // Serial.print(gyro.gyro.x);
        // Serial.print("  Y: ");
        // Serial.print(gyro.gyro.y);
        // Serial.print("  Z: ");
        // Serial.print(gyro.gyro.z);
        // Serial.println();
    }

    statusled.blink();

    if (movement == true){
        switchCaseValue = 2;
    }

    switch (switchCaseValue)
    {
    case 0:
        statusled.blink(500, 500, CRGB::Red);
        break;
    case 1:
        statusled.blink(500, 500, CRGB::Green);
        // check if movement is continuous for specified amount of time
        if((abs(gyro.gyro.x) > accel_thresh) || (abs(gyro.gyro.y) > accel_thresh) || (abs(gyro.gyro.z) > accel_thresh)){
            // rateIsHigh = true;
            if(startTime == 0){
                startTime = millis();
            } else if (millis() - startTime >= time_thresh){
                // switchCaseValue = 2;
                Serial.printf("Over %d deg/s for at %d ms!!!!!!!!!!!\n\n\n\n\n\n\n",accel_thresh,time_thresh);
                startTime = 0;
                // rateIsHigh = false;
                movement = true;
            }
        }else{
            startTime = 0;
        }
        break;
    case 2:
        // Send Positive bit to UTS indicating movement detected 
        statusled.blink(500, 500, CRGB::Blue);
        actTime = millis();
        movement_char = 1;
        break;
    default:
        break;
    }
}