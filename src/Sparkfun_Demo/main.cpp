#include <Wire.h> // Needed for I2C

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEUtils.h>

// Replace with your unique UUIDs
#define SERVICE_UUID "Service-1"
#define CHARACTERISTIC_1_UUID "characteristic_1"
#define CHARACTERISTIC_2_UUID "characteristic_2"
#define CHARACTERISTIC_3_UUID "characteristic_3"

SFE_MAX1704X lipo; // Defaults to the MAX17043

const char BLE_Name[] = "FEI_IMU_BOX_01";

const int NUM_SERVICES = 1;
const int NUM_CHAR_PER_SERVICE = 3;
const int NUM_CHARS = NUM_SERVICES * NUM_CHAR_PER_SERVICE;

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
        "00000000-0000-0000-0000-000000000002",
        "00000000-0000-0000-0000-000000000003",
};

double voltage = 0; // Variable to keep track of LiPo voltage
double soc = 0;     // Variable to keep track of LiPo state-of-charge (SOC)
bool alert;         // Variable to keep track of whether alert has been triggered
float change_rate = 0.0;
bool charging = 0;

double current_voltage = 0;
double prev_voltage = 0;


void setBLE()
{
    // To save values in string
    char analog1Str[10] = "";
    char analog2Str[10] = "";
    char analog3Str[10] = "";
    char notify_char[100];
    // Converting to strings
    dtostrf(voltage, 1, 2, analog1Str);
    dtostrf(static_cast<float>(soc), 1, 2, analog2Str);
    dtostrf(charging, 1, 2, analog3Str);

    // Combining to single string
    snprintf(notify_char, sizeof(notify_char), "<%s,%s,%s>", analog1Str, analog2Str, analog3Str); // show flag and battery status
    // snprintf(notify_char, sizeof(notify_char), "<%s>", analog1Str);                              // Only show flag

    // Notify string
    characteristics[0]->setValue(notify_char);
    characteristics[0]->notify();
}


void setup()
{
  Serial.begin(115200); // Start serial, to output debug data
  while (!Serial)
    ; // Wait for user to open terminal
  Serial.println(F("MAX17043 Example"));

  BLEDevice::init("DRS_IMU");
  pServer = BLEDevice::createServer();
  BLEService *services[NUM_SERVICES];

  services[0] = pServer->createService(service_ids[0]);

  characteristics[0] = services[0]->createCharacteristic(characteristic_ids[0],  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristics[0]->addDescriptor(new BLE2902());
  characteristics[1] = services[0]->createCharacteristic(characteristic_ids[1],  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristics[1]->addDescriptor(new BLE2902());
  characteristics[2] = services[0]->createCharacteristic(characteristic_ids[2],  BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  characteristics[2]->addDescriptor(new BLE2902());

  services[0]->start();
  pServer->getAdvertising()->addServiceUUID(services[0]->getUUID());
  pServer->getAdvertising()->start();

  Wire.begin();

  lipo.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // Set up the MAX17043 LiPo fuel gauge:
  if (lipo.begin() == false) // Connect to the MAX17043 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Quick start restarts the MAX17043 in hopes of getting a more accurate
  // guess for the SOC.
  lipo.quickStart();

  // We can set an interrupt to alert when the battery SoC gets too low.
  // We can alert at anywhere between 1% - 32%:
  lipo.setThreshold(20); // Set alert threshold to 20%.
}

void loop()
{
  voltage = lipo.getVoltage();
  current_voltage = voltage;

  if(current_voltage > 4.20){
    charging = 1;
  }else{
    charging = 0;
  }
  prev_voltage = current_voltage;

  soc = lipo.getSOC();
  alert = lipo.getAlert();

  // Print the variables:
  Serial.print("Voltage: ");
  Serial.print(voltage); // Print the battery voltage
  Serial.println(" V");

  Serial.print("Percentage: ");
  Serial.print(soc); // Print the battery state of charge
  Serial.println(" %");

  
  setBLE();

  delay(500);
}