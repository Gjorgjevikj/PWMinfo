# PWMinfo
Arduino PWM measuring library - determines the frequency and the duty of signals at any pin

Simple interrupt driven library for measuring parameters (duty cycle & frequency) of a PWM signal on any Arduino pin 
Uses EnableInterrupt library from https://github.com/GreyGnome/EnableInterrupt.

Supports AVR and ESP8266 also.

Single header file defining template fully static class and some global functions.

## PWMinfo 

PWMinfo object (templetized by pin) `PWMinfo<pin> obj` installs ISR that records the timestamps of the last 2 signal level changes that are used to determine the frequency and the duty cycle of the signal. 
  
  ```C
  obj::begin(); // installs the ISR and starts monitoring the signal
  obj::end(); // uninstalls the ISR (stops monitoring)
  obj::reset(); // resets timestamps (starts monitoring from scratch)
  bool obj::valid(); // returns true if the signal has changed sufficient times in order to determine the frequency and the duty 
  float obj::duty(); // returns the duty of the signal in % (0-100) - estimate using the timings of the last cycle only
  float obj::frequency(); // returns the frequency of the signal in Hz  - estimate using the timings of the last cycle only
  ```
  
  Global functions:

  `OnePulseFreqEstimate<pin>(timeout)` - deretmines the frequency of the signal on the given pin and returns as soon as one full cycle or at most timeout milliseconds have passed
  
  `OnePulseDutyEstimate<pin>(timeout)` - deretmine the duty of the signal on the given pin and returns as soon as one full cycle or at most timeout milliseconds have passed
   
