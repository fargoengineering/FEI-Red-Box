
#ifndef __TimedBlink__H__
#define __TimedBlink__H__

#include <Arduino.h>
#include <FastLED.h>

enum blink_t
{
  BLINK_ON,
  BLINK_OFF
};

class TimedBlink
{
private:
  unsigned long m_blinkTime;
  int m_onForTime;
  int m_offForTime;
  blink_t m_blinkState;
  short m_pin;
  int m_resolution;
  CRGB m_color;
  CRGB m_color2;

  void reset();

public:
  TimedBlink(int pin);
  void blink(int on_for, int off_for, CRGB color);
  void blinkColor2(CRGB color);
  void blink();
  void setOnTime(int ms);
  void setOffTime(int ms);
  void setBlinkState(blink_t state);
  void blinkDelay(int d);
  void blinkOff();
};

#endif // __TimedBlink__H__
