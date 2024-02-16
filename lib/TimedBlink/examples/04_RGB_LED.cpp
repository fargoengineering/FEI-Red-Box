// Green Blink messages in, Green and yellow for send/Receiving going on, Red blink CAN errors, Blue Bluetooth software update, etc? Might be nice to have a pc link concept if possible.
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <TimedBlink.h>

#define EN_3V3_SW 32 // The 3.3V_SW regulator Enable pin is connected to D32
#define RGB_LED 26   // OpenLog ESP32 RGB LED is connected to D2

bool is_red = false;
unsigned long actTime;
unsigned long remTime;
unsigned long period = 250;

TimedBlink statusled(26);

void setup()
{

  pinMode(EN_3V3_SW, OUTPUT);
  digitalWrite(EN_3V3_SW, HIGH);

  delay(30); // sanity delay

  statusled.blink(1000, 500, CRGB::Red);
}

void loop()
{

  static unsigned long startTime = 0; // Store the start time
  static unsigned long switchStartTime = 0;
  static int switchCaseValue = 1;
  statusled.blink();

  if (millis() - switchStartTime >= 5000)
  { // 5 seconds
    //   switchCaseValue = 4;
    switch (switchCaseValue)
    {
    case 1:
      statusled.blinkColor2(CRGB::Blue);
      switchCaseValue = 2;
      break;
    case 2:
      statusled.blink(100, 300, CRGB::Blue);
      statusled.blinkColor2(CRGB::Black);
      switchCaseValue = 3;
      break;
    case 3:
      statusled.blink(500, 500, CRGB::Yellow);
      statusled.blinkColor2(CRGB::Green);
      switchCaseValue = 4;
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

    switchStartTime = millis(); // Reset the timer for the next cycle
  }
  // Other non-blocking code can be executed here

  // To start the cycle again, you can reset isWaiting to true under certain conditions
}