/*
 * TimedBlink blink delay sample
 *
 * Makes the built-in LED blink. 200ms ON and 100ms OFF.
 *
 * created January 14, 2017
 * by Lorenzo Pasqualis
 *
 * This example code is in the public domain.
 */
 
#include <TimedBlink.h>

TimedBlink statusled(LED_BUILTIN);

void setup() {
   Serial.begin(9600);
   pinMode(LED_BUILTIN, OUTPUT);
   statusled.blink(100,300);
}

void loop() {
  Serial.println("Waiting 3 seconds and blinking the LED at the same time...");
  statusled.blinkDelay(3000);
}

