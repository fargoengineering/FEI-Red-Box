// SparkFun DataLogger IoT – 9DoF Test Example
// Tested with Espressif ESP32 v2.0.5 and the "ESP32 Dev Module" board definition

/**********************************************************************************************
 *
 * WARNING!
 * 
 * This is a sketch we wrote to test the DataLogger IoT – 9DoF hardware.
 * Please think very carefully before uploading it to your DataLogger.
 * 
 * You will overwrite the DataLogger firmware, leaving it unable to update or restore itself. 
 * 
 * The DataLogger IoT – 9DoF comes pre-programmed with amazing firmware which can do _so_ much.
 * It is designed to be able to update itself and restore itself if necessary.
 * But it can not do that if you overwrite the firmware with this test sketch.
 * It is just like erasing the restore partition on your computer hard drive.
 * Do not do it - unless you really know what you are doing.
 * 
 * Really. We mean it.
 * 
 * Your friends at SparkFun.
 * 
 * License: MIT. Please see LICENSE.MD for more details
 * 
 **********************************************************************************************/


#include <SPI.h>
#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h" //Click here to get the library: http://librarymanager/All#SparkFun_6DoF_ISM330DHCX
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#define IMU_CS 5 // The ISM330 chip select is connected to D5
#define MAG_CS 27

// SPI instance class call
SparkFun_ISM330DHCX_SPI myISM; 
SFE_MMC5983MA myMag;

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

void setup(){

	SPI.begin();

	Serial.begin(115200);
	pinMode(IMU_CS, OUTPUT);
	digitalWrite(IMU_CS, HIGH);

  	Serial.println(F("SparkFun DataLogger IoT – 9DoF Test Example"));

	if( !myISM.begin(IMU_CS) ){
		Serial.println(F("IMU did not begin. Freezing..."));
	  while(1);
	}

	if(myMag.begin(MAG_CS) == false){
		Serial.println("Magnetometer setup failure");
		while(true)
			;
	}

	Serial.println("Magnetometer connected");

	myMag.softReset();
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 

	Serial.println(F("IMU has been reset."));
	Serial.println(F("Applying settings..."));
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

void loop(){

	// Check if both gyroscope and accelerometer data is available.
	if( myISM.checkStatus() ){
		myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);
		
		// Accelerometer
		// Serial.print(F("Accelerometer: "));
		// Serial.print(F("X: "));
		// Serial.print(accelData.xData);
		// Serial.print(F(" "));
		// Serial.print(F("Y: "));
		// Serial.print(accelData.yData);
		// Serial.print(F(" "));
		// Serial.print(F("Z: "));
		// Serial.print(accelData.zData);
		// Serial.println(F(" "));

		// Gyroscope
		Serial.print(F("Gyroscope: "));
		Serial.print("X: ");
		Serial.print(gyroData.xData);
		Serial.print(F(" "));
		Serial.print(F("Y: "));
		Serial.print(gyroData.yData);
		Serial.print(F(" "));
		Serial.print(F("Z: "));
		Serial.print(gyroData.zData);
		Serial.println(F(" "));
	}

	delay(100);

	// Magnetometer 
    // uint32_t currentX = 0;
    // uint32_t currentY = 0;
    // uint32_t currentZ = 0;
    // double normalizedX = 0;
    // double normalizedY = 0;
    // double normalizedZ = 0;

    // myMag.getMeasurementXYZ(&currentX, &currentY, &currentZ);

    // Serial.print("X raw: ");
    // Serial.print(currentX);
    // Serial.print("\nY raw: ");
    // Serial.print(currentY);
    // Serial.print("\nZ raw: ");
    // Serial.println(currentZ);

    // normalizedX = (double)currentX - 131072.0;
    // normalizedX /= 131072.0;
    // normalizedY = (double)currentY - 131072.0;
    // normalizedY /= 131072.0;
    // normalizedZ = (double)currentZ - 131072.0;
    // normalizedZ /= 131072.0;

    // Serial.print("X Gauss: ");
    // Serial.print(normalizedX * 8, 5);

    // Serial.print("\nY Gauss: ");
    // Serial.print(normalizedY * 8, 5);

    // Serial.print("\nZ Gauss: ");
    // Serial.println(normalizedZ * 8, 5);

    // Serial.println();
	delay(100);
}