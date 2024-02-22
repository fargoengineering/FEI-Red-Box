#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Arduino.h>
#include <string>
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
const char VERSION[] = "2024.02.15"; // This version: Jan 2024 - Dylan
const char BLE_Name[] = "FEI_IMU_BOX_01";
const int NUM_SERVICES = 2;
const int NUM_CHAR_PER_SERVICE = 1;
const int NUM_CHARS = NUM_SERVICES * NUM_CHAR_PER_SERVICE;
const long interval = 500; // interval to update BLE
int input_vals[3];

// Non-Constant values
int accel_thresh = 2;                
int battery = 100;
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
int start_bit = 0;
float movement_low_end = 0;
float movement_high_end = 0;
int ms_target = 0;
int axis_key = 0;

BLEServer *pServer;
BLEService *pServices[NUM_SERVICES];
BLECharacteristic *characteristics[NUM_CHARS];
const char *const service_ids[NUM_SERVICES] =
    {
        "00000000-0000-0000-0000-000000000001",
        "00000000-0000-0000-0000-000000000002",
};
const char *characteristic_ids[NUM_CHARS] =
    {
        "00000000-0000-0000-0000-000000000001", // Read AI 1,2
        "00000000-0000-0000-0000-000000000005", // WRITE DO
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

int outputIntegers[10];
float outputFloats[10];

void parseString(String inputString) {
  // Remove leading/trailing whitespaces
  String trimmedString = inputString.substring(1, inputString.length() - 1);

  // Split the string by commas
  int commaIndex;
  int lastIndex = 0;
  int index = 0;
  String values[5];

  while ((commaIndex = trimmedString.indexOf(',', lastIndex)) != -1 && index < 5) {
    values[index++] = trimmedString.substring(lastIndex, commaIndex);
    lastIndex = commaIndex + 1;
  }

  // Process the last value
  if (index < 5) {
    values[index] = trimmedString.substring(lastIndex);
  }

    // <1,0.5,1.5,500,0>
  // Convert and store values
  start_bit = values[0].toInt();
  movement_low_end = values[1].toFloat();
  movement_high_end = values[2].toFloat();
  ms_target = values[3].toInt();
  axis_key = values[4].toInt();
}


class CharacteristicCallback : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string value = pCharacteristic->getValue();
        std::string UID = (pCharacteristic->getUUID()).toString();
        Serial.printf("\nReceived Data from BLE: %s", value.c_str());
        Serial.printf("\nCharacteristic: %s", UID.c_str());

        // February 13, 2024:
        String valueStr;
        for(char c : value){
            valueStr += c;
        }
        parseString(valueStr);

        Serial.printf("Start bit: %d\n",start_bit);
        Serial.printf("Movement Low end: %f\n",movement_low_end);
        Serial.printf("Movement High End: %f\n",movement_high_end);
        Serial.printf("ms_target : %d\n",ms_target);
        Serial.printf("axis_key: %d\n",axis_key);

        trigger_in = start_bit;

        // Reset loop
        is_triggered = false;
        movement_char = 0;
    }
};

void setBLE()
{
    // To save values in string
    char analog1Str[10] = "";
    char analog2Str[10] = "";
    char analog3Str[10] = "";

    // Converting to strings
    dtostrf(static_cast<float>(movement_char),1,2,analog1Str);
    dtostrf(int(movement),1,2,analog2Str);
    dtostrf(lipo.getSOC(),1,2,analog3Str);

    // Combining to single string
    snprintf(notify_char, sizeof(notify_char), "<%s,%s,%s>", analog1Str, analog2Str, analog3Str);   // show flag and battery status
    // snprintf(notify_char, sizeof(notify_char), "<%s>", analog1Str);                              // Only show flag

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
    services[1] = pServer->createService(service_ids[1]);

    // Read characteristics
    int char_index = 0;
    characteristics[char_index] = services[0]->createCharacteristic(characteristic_ids[char_index], BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    characteristics[char_index]->addDescriptor(new BLE2902());

    // Write characteristics
    char_index = 1;
    characteristics[char_index] = services[1]->createCharacteristic(characteristic_ids[char_index], BLECharacteristic::PROPERTY_WRITE);
    characteristics[char_index]->addDescriptor(new BLE2902());
    characteristics[char_index]->setCallbacks(new CharacteristicCallback());

    // Start services
    Serial.println("Starting service: " + String(service_ids[0]));
    services[0]->start();
    services[1]->start();

    // advertising services
    Serial.println("Advertising service: " + String(service_ids[0]));
    pServer->getAdvertising()->addServiceUUID(services[0]->getUUID());
    pServer->getAdvertising()->addServiceUUID(services[1]->getUUID());

    delay(1000);
    pServer->getAdvertising()->start();

    Wire.begin();
    imusetup();
    if (lipo.begin() == false){
        lipo.enableDebugging();
        Serial.println("Unable to read battery.");
    }else{
        Serial.print("Battery Charge is at: %");
        Serial.println(lipo.getSOC());
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
        movement_char = 0;
        break;
    case 1:
        statusled.blink(500, 500, CRGB::Green);
        // check if movement is continuous for specified amount of time
        if(axis_key == 0)   // measure pitch - x axis
        {
        //    Serial.println(abs(gyro.gyro.x));
           if((movement_high_end)>(abs(gyro.gyro.x))>(movement_low_end)){
                if(startTime == 0){
                    startTime = millis();
                } else if (millis() - startTime >= ms_target){
                    Serial.printf("Over %d deg/s for at %d ms!\n\n\n\n\n\n\n",movement_low_end,ms_target);
                    startTime = 0;
                    movement = true;
                }
                movement_char = 1;
            }else{
                startTime = 0;
                if((abs(gyro.gyro.x)>(movement_high_end))){
                    movement_char = 3;
                }
                else if((abs(gyro.gyro.x)<(movement_low_end))){
                    movement_char = 2;
                }
                movement = false;
            }
        }
        else if (axis_key == 1){ // measure YAW - z axis
            // Serial.println(abs(gyro.gyro.z));
            if((movement_high_end)>(abs(gyro.gyro.z))>(movement_low_end)){
                if(startTime == 0){
                    startTime = millis();
                } else if (millis() - startTime >= ms_target){
                    Serial.printf("Over %d deg/s for at %d ms!\n\n\n\n\n\n\n",movement_low_end,ms_target);
                    startTime = 0;
                    movement = true;
                }
                movement_char = 1;
            }else{
                startTime = 0;
                if(abs(gyro.gyro.z)>(movement_high_end)){
                    movement_char = 3;
                }
                else if (abs(gyro.gyro.z)<(movement_low_end)){
                    movement_char = 2;
                }
                movement = false;
            }
        }
        break;
    case 2:
        // Send Positive bit to UTS indicating movement detected 
        statusled.blink(500, 500, CRGB::Blue);
        actTime = millis();
        // movement_char = 1;
        break;
    default:
        break;
    }
}