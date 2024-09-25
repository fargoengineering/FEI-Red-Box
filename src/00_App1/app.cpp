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
#include "SparkFun_ISM330DHCX.h"
#include <WiFi.h>
#include <FastLED.h>
#include <TimedBlink.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BusIO_Register.h>

#define EN_3V3_SW 32 // The 3.3V_SW regulator Enable pin is connected to D32
#define RGB_LED 26   // OpenLog ESP32 RGB LED is connected to D26

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

TimedBlink statusled(26);

// Note: With FastLED v3.5.0: Arduino\libraries\FastLED\src\platforms\esp\32\clockless_rmt_esp32.cpp generates two unused variable errors:
// "Arduino\libraries\FastLED\src\platforms\esp\32\clockless_rmt_esp32.cpp:111:15: error: variable 'espErr' set but not used [-Werror=unused-but-set-variable]"
// You can fix this by adding (void)espErr; at line 112 and again at line 241

uint8_t gHue = 0; // rotating "base color" used by many of the patterns
// SPI instance class call
SparkFun_ISM330DHCX_SPI myISM;
SFE_MAX1704X lipo;

uint8_t IMU_CS = 5;
#define MAG_CS 27
#define CHARGE_PIN 33;

// Structs for X,Y,Z data
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData_triggered;
sfe_ism_data_t gyroData;
const char VERSION[] = "2023.11.001"; // Initial version - Nov 2023
const char BLE_Name[] = "FEI_IMU_BOX";

const int NUM_SERVICES = 1;
const int NUM_CHAR_PER_SERVICE = 2;                        // Restrict to 4 all the time.
const int NUM_CHARS = NUM_SERVICES * NUM_CHAR_PER_SERVICE; // Total Characteristics

unsigned long previousMillis = 0;
const long interval = 500; // interval at which to update BLE

float analog_vals[2]; // Stores 2 values. AI0 and AI1
int trigger_in = 0;   // Stores DO status value 0 or 1.
char notify_char[100];

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

int parseBLE(char *input)
{
  return input[1] - '0';
}
//<0>
class CharacteristicCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    std::string UID = (pCharacteristic->getUUID()).toString();
    Serial.printf("\nReceived Data from BLE: %s", value.c_str());
    Serial.printf("\nCharacteristic: %s", UID.c_str());
    int inp_val = value.c_str()[1];
    Serial.printf("\nInput Int value: %d", inp_val);

    // Process val
    // Ex
    analog_vals[0] = inp_val * 1.0f;
    analog_vals[1] = inp_val * 1.0f;
    trigger_in = inp_val;
  }
};

void showData()
{
  Serial.printf("All values: AI0: %10.3f, AI1: %10.3f, DO0: %d \n", analog_vals[0], analog_vals[1], trigger_in);

  Serial.print("Notified Characteristic value: ");
  Serial.println(notify_char);
}

void setBLE()
{
  // To save values in string
  char analog1Str[10] = "";
  char analog2Str[10] = "";
  char analog3Str[10] = "";
  char doutStr[2] = "0";

  // Converting to strings
  // dtostrf(lipo.getSOC(), 1, 2, analog1Str);  // Battery State
  dtostrf(gyroData_triggered.xData, 1, 2, analog1Str);
  dtostrf(gyroData_triggered.yData, 1, 2, analog2Str);
  dtostrf(gyroData_triggered.zData, 1, 2, analog3Str);

  if (trigger_in > 0)
  {
    // doutStr = "1";
    itoa(trigger_in, doutStr, 10);
  }

  // Combining to single string
  snprintf(notify_char, sizeof(notify_char), "<%s,%s,%s>", analog1Str, analog2Str, analog3Str);    //doutStr);
  // Notify string
  characteristics[0]->setValue(notify_char);
  characteristics[0]->notify();
}

void imusetup()
{

  SPI.begin();

  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);

  if (!myISM.begin(IMU_CS))
  {
    Serial.println(F("IMU did not begin. Freezing..."));
    while (1)
      ;
  }

  // Reset the device to default settings
  // This if helpful is you're doing multiple uploads testing different settings.
  myISM.deviceReset();

  // Wait for it to finish reseting
  while (!myISM.getDeviceReset())
  {
    delay(1);
  }

  delay(100);

  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();

  // Set the output data rate and precision of the accelerometer
  myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
  myISM.setAccelFullScale(ISM_4g);

  // Set the output data rate and precision of the gyroscope
  myISM.setGyroDataRate(ISM_GY_ODR_104Hz);
  myISM.setGyroFullScale(ISM_500dps);

  // Turn on the accelerometer's filter and apply settings.
  myISM.setAccelFilterLP2();
  myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

  // Turn on the gyroscope's filter and apply settings.
  myISM.setGyroFilterLP1();
  myISM.setGyroLP1Bandwidth(ISM_MEDIUM);
}

void setupfuel()
{

  // lipo.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

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

  // To quick-start or not to quick-start? That is the question!
  // Read the following and then decide if you do want to quick-start the fuel gauge.
  // "Most systems should not use quick-start because the ICs handle most startup problems transparently,
  //  such as intermittent battery-terminal connection during insertion. If battery voltage stabilizes
  //  faster than 17ms then do not use quick-start. The quick-start command restarts fuel-gauge calculations
  //  in the same manner as initial power-up of the IC. If the system power-up sequence is so noisy that the
  //  initial estimate of SOC has unacceptable error, the system microcontroller might be able to reduce the
  //  error by using quick-start."
  // If you still want to try a quick-start then uncomment the next line:
  // lipo.quickStart();

  // Read and print the device ID
  Serial.print(F("Device ID: 0x"));
  uint8_t id = lipo.getID(); // Read the device ID
  if (id < 0x10)
    Serial.print(F("0"));  // Print the leading zero if required
  Serial.println(id, HEX); // Print the ID as hexadecimal

  // Read and print the device version
  Serial.print(F("Device version: 0x"));
  uint8_t ver = lipo.getVersion(); // Read the device version
  if (ver < 0x10)
    Serial.print(F("0"));   // Print the leading zero if required
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

  // Setting defaults
  analog_vals[0] = 0;
  analog_vals[1] = 0;
  trigger_in = 0;

  Wire.begin();

  imusetup();
  setupfuel();

  pinMode(EN_3V3_SW, OUTPUT);
  pinMode(33, INPUT);
  digitalWrite(EN_3V3_SW, HIGH);

  delay(30); // sanity delay

  statusled.blink(1000, 500, CRGB::Red);

  Serial.println(F("Setup Done. Starting LOOP."));
}

bool is_triggered = false;
bool is_triggered_state = false;
unsigned long actTime;
unsigned long remTime;
unsigned long period = 5000;

static unsigned long startTime = 0; // Store the start time
static unsigned long switchStartTime = 0;
static int switchCaseValue = 0;

void loop()
{
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  //  Serial.println("Loop Restart");
  unsigned long currentMillis = millis();

  // Updated BLE Every 1000 milliseconds
  if (currentMillis - previousMillis >= interval)
  {
    setBLE(); // configure and set all 4 characteristics
    pServer->getAdvertising()->start();
    previousMillis = currentMillis;

    // showData();
  }

  // Serial.println();
  if (trigger_in > 0 && !is_triggered)
  {
    is_triggered = true;
    switchCaseValue = 1;
  }

  // if (switchCaseValue > 0)
  // {
    if (myISM.checkStatus())
    {
      myISM.getAccel(&accelData);
      myISM.getGyro(&gyroData);
      gyro.gyro.x = gyro.gyro.x - 0.14;
      gyro.gyro.y = gyro.gyro.y - 0.55;
      gyro.gyro.z = gyro.gyro.z - 0.14;

      Serial.print(F("\nActual Accelerometer:\n "));
      Serial.print(F("X: "));
      Serial.print(accelData.xData);
      Serial.println(F(" "));
      Serial.print(F("Y: "));
      Serial.print(accelData.yData);
      Serial.println(F(" "));
      Serial.print(F("Z: "));
      Serial.print(accelData.zData);
      Serial.println(F(" "));
      Serial.print(F("\nGyroscope:\n "));
      Serial.print("X: ");
      Serial.print(gyroData.xData);
      Serial.println(F(" "));
      Serial.print(F("Y: "));
      Serial.print(gyroData.yData);
      Serial.println(F(" "));
      Serial.print(F("Z: "));
      Serial.print(gyroData.zData);
      Serial.println(F(" "));
    }
  // }

  delay(500);

  statusled.blink();

  switch (switchCaseValue)
  {
  case 0:
    statusled.blink(100, 100, CRGB::Red);
    break;
  case 1:
    // statusled.blink(100, 100, CRGB::Green);
    gyroData_triggered = gyroData;
    switchCaseValue = 2;
    break;
  case 2:
    if (abs(gyroData.xData - gyroData_triggered.xData) > 2 || abs(gyroData.yData - gyroData_triggered.yData) > 2 || abs(gyroData.zData - gyroData_triggered.zData) > 2)
    {
      switchCaseValue = 3;
    }
    statusled.blink(500, 500, CRGB::Yellow);
    statusled.blinkColor2(CRGB::Black);

    Serial.print(F("Trigger X: "));
    Serial.print(trigger_in);
    Serial.print(F(" "));
    Serial.print(F("############# gyroData: "));
    Serial.print(F("X: "));
    Serial.print(gyroData.xData);
    Serial.print(F(" "));
    Serial.print(gyroData_triggered.xData);
    Serial.print(F("-: "));
    Serial.print(abs(gyroData.xData - gyroData_triggered.xData));
    Serial.println(F(" "));
    break;
    // <0> <1> <0,1>
    // <12.45,12.55,0>
  case 3:
    statusled.blink(500, 500, CRGB::Green);
    actTime = millis();
    if (actTime - remTime >= period)
    {
      remTime = actTime;
      is_triggered = false;
      switchCaseValue = 0;
    }
    break;
  case 4:
    statusled.blinkOff(); // Call blinkOff after 10 seconds
    statusled.blink(100, 50, CRGB::Green);
    switchCaseValue = 5;
    break;
  case 5:
    statusled.blinkColor2(CRGB::Green);
    switchCaseValue = 6;
    break;
  case 6:
    statusled.blink(25, 0, CRGB::Green);
    switchCaseValue = 7;
    break;
  case 7:

    switchCaseValue = 1;
    break;
  }

  // switchCaseValue++;
  if (switchCaseValue > 5)
  {
    switchCaseValue = 1;
  }

  delay(1);
}
