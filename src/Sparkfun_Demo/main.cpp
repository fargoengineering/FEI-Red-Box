#include <Arduino.h>
#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

// Structs for X,Y,Z data
SparkFun_ISM330DHCX myISM; 
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData;


// Interrupt pin
byte interrupt_pin = 1;//D1; 

void i2c_scan(){
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

void imusetup()
{
    SPI.begin();

    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH);

    if (!myISM.begin_SPI(5))
    {
        Serial.println("ISM330DHCX connection error");
    }
}

Adafruit_ISM330DHCX myISM;

void setup(){
	// Set the interrupt to INPUT
	pinMode(interrupt_pin, INPUT);
	Serial.begin(115200);
	Serial.println("Begin");

	// Wire.begin();

	// // i2c_scan();

	// if( !myISM.begin(0x36) ){
	// 	Serial.println("Did not begin.");
	// 	while(1);
	// }

	// Reset the device to default settings. This is helpful if you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 

	Serial.println("Reset.");
	Serial.println("Applying settings.");
	delay(100);
	
	myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
	myISM.setAccelFullScale(ISM_4g); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Set the accelerometer's status i.e. the data ready to interrupt one. 
	// Commented out just below is the function to send the data ready
	// to interrupt two. 

	myISM.setAccelStatustoInt1();
	//myISM.setAccelStatustoInt2();


	// We can just as easily set the gyroscope's data read signal to either interrupt

	//myISM.setGyroStatustoInt1();
	//myISM.setGyroStatustoInt2();


	// Uncommenting the function call below will change the interrupt to 
	// active LOW instead of HIGH.

	//myISM.setPinMode();

	// This function call will modify which "events" trigger an interrupt. No 
	// argument has been given, please refer to the datasheet for more 
	// information.

	// myISM.setIntNotification(uint8_t val);
	Serial.println("End setup");
}

void loop(){


	// if( digitalRead(interrupt_pin) == HIGH ){
	myISM.getAccel(&accelData);
	Serial.print("Accelerometer: ");
	Serial.print("X: ");
	Serial.print(accelData.xData);
	Serial.print(" ");
	Serial.print("Y: ");
	Serial.print(accelData.yData);
	Serial.print(" ");
	Serial.print("Z: ");
	Serial.print(accelData.zData);
	Serial.println(" ");

	myISM.getGyro(&gyroData);
	Serial.print("Gyrometer: ");
	Serial.print("X: ");
	Serial.print(gyroData.xData);
	Serial.print(" ");
	Serial.print("Y: ");
	Serial.print(gyroData.yData);
	Serial.print(" ");
	Serial.print("Z: ");
	Serial.print(gyroData.zData);
	Serial.println(" ");
	// }

	delay(500);
}
