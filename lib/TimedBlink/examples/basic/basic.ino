/*
 * TimedBlink basic example of usage
 *
 * Makes the built-in LED blink. 200ms ON and 100ms OFF.
 *
 * created January 14, 2017
 * by Lorenzo Pasqualis
 *
 * This example code is in the public domain.
 */

#include <TimedBlink.h>

TimedBlink statusled(LED_BUILTIN); // Use built-in LED

void setup() {
   Serial.begin(9600);
   pinMode(LED_BUILTIN, OUTPUT);
   statusled.blink(150,50, CRGB::Blue);  // On for 150ms, off for 50ms
}

void loop() {
  statusled.blink();

}
