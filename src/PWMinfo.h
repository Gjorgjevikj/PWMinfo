/*
Library for simple PWM signal measuring v 1.0.1

Estimates the frequency and the duty of a PWM signal
based on timing of the rising and falling edges of the signal
using interrupts using only the last full cycle

This library is distributed in the hope that it will be useful, but
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE either express or implied.
Released into the public domain.
Distributed under: MIT License

https://github.com/Gjorgjevikj/PWMsense

(c) Dejan Gjorgjevikj, 2017
revised: 23.08.2018
revised: 10.09.2018 - major revision
 - changed to use only last 3 transitions (not 4) in monitored signal
 - changed names of functions
*/

#ifndef PWMINFO_H
#define PWMINFO_H

#define PWMINFO_VER 1.0.1

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

// To enable pinchange intterupt on any pin on Arduino EnableIntterupt library from GrayGnome 
// https://github.com/GreyGnome/EnableInterrupt.git is used 

//#define LIBCALL_ENABLEINTERRUPT
#if defined(ARDUINO_ARCH_AVR)
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>
#define ICACHE_RAM_ATTR
#endif

// the value to be returned from the frequency sense functions when the frequency can not be determined
#define UNDETERMINED_FREQ 0.0

#define PWMinf(obj,pin) typedef PWMinfo<pin> obj;

// The data filled in intterupt handler packed in one single structure
struct Cycle
{
  unsigned long fallTime; // time of last pulse fall event
  unsigned long riseTime; // time of last pulse rise event
  unsigned long prevFallTime; // time of previous pulse fall event
  unsigned long prevRiseTime; // time of previous pulse rise event
};

// The interrupt handler functions per pin and the class handling the sensing (reding the recorded counters by the interrupt handlers and calculating the duty/frequency) are templates 
template <uint8_t PIN> 
class PWMinfo;

// Template for the pinchange interrupt handler that records the time ste signal changed state
template <uint8_t PIN>
void __attribute__ ((optimize("-O4"))) ICACHE_RAM_ATTR ISR_Change(void) 
//void ICACHE_RAM_ATTR ISR_Change(void) 
{
	unsigned long t = micros(); // must stay here!!!
#if defined(ESP8266)      
	//bool PinState = digitalRead(PIN);
	if (digitalRead(PIN)) // rise
#else
	//bool PinState = !!arduinoPinState;
	if (arduinoPinState) // rise
#endif

  //if (PinState) // rise
  {
	  PWMinfo<PIN>::pulses.prevRiseTime = PWMinfo<PIN>::pulses.riseTime;
	  PWMinfo<PIN>::pulses.riseTime = t; 
  }
  else //fall
  {
	  PWMinfo<PIN>::pulses.prevFallTime = PWMinfo<PIN>::pulses.fallTime;
	  PWMinfo<PIN>::pulses.fallTime = t; 
  }
}

/// <summary> PWMinfo Template Class 
/// templetized by pin, so there will be a separate class / object / ISR for every monitored pin
/// </summary>
/// <remarks>
/// Monitors a signal on a given pin recording the times of the changes of last 1 and 1/2 of a cycle 
/// the last 2 rises and the last 2 falls
/// </remarks>
template <uint8_t PIN> 
class PWMinfo
{
  private:
//public:
    PWMinfo() {} // no need for constructor - everything is static, since every class will have a single instance 
	volatile static struct ICACHE_RAM_ATTR Cycle pulses;

public:
	/// <summary> Initializes the monitoring object and starts monitoring </summary>
    static void begin(void) 
    { 
      pinMode(PIN, INPUT_PULLUP); 
      reset();
#if defined(ARDUINO_ARCH_AVR)
      enableInterrupt(PIN, ISR_Change<PIN>, CHANGE);
#elif defined(ESP8266)      
      attachInterrupt(digitalPinToInterrupt(PIN), ISR_Change<PIN>, CHANGE);
#endif
    }
    
	/// <summary> Stops monitoring </summary>
	static void end(void)
    {
#if defined(ARDUINO_ARCH_AVR)
      disableInterrupt(PIN);
#elif defined(ESP8266)      
      detachInterrupt(digitalPinToInterrupt(PIN));
#endif
    }

	/// <summary> Do we have a valid signal - at least one full cycle has been recorded so we can estimeate the duty and the frequency </summary>
	/// <returns> true if the signal is valid for estimation </returns>
	static bool valid(void)
	{
		return pulses.prevRiseTime || pulses.prevFallTime;
	}

	/// <summary> Returns the duty cycle of PWM signal in % </summary>
	/// <returns> the duty in percents 0-100 </returns>
	static float duty()
	{
		float CalculatePWMDuty(Cycle *);
		return CalculatePWMDuty(const_cast<Cycle *>(&pulses));
	}

	/// <summary> Returns the Pulse Repetition Frequency (PRF) in Hz </summary>
	/// <returns> frequency in Hz, UNDETERMINED_FREQ if can not determine </returns>
	static float frequency()
	{
		float CalculatePWMFrequency(Cycle *);
		return CalculatePWMFrequency(const_cast<Cycle *>(&pulses));
	}

	/// <summary> Resets the object </summary>
	static void reset() // resets the timestamps
    {
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
		// produces slihgtly shorter code on Arduino, not accepted by ESP8266 compiler
		pulses= { 0UL, 0UL, 0UL, 0UL };
#else
		pulses.fallTime = pulses.prevFallTime = pulses.prevRiseTime = pulses.riseTime = 0UL;
#endif
    }

	/// <summary> Returns the pin being monitored </summary>
	/// <returns> the pin being monitored </returns> 
	static uint8_t pin()
	{
		return PIN;
	}

    friend void ISR_Change<PIN>(void); // enable acces to the private static variables to the ISR routine 
};

template <uint8_t PIN> volatile Cycle PWMinfo<PIN>::pulses = { 0UL, 0UL, 0UL, 0UL };

// class end
///////////////////////////////////////////////////////////////////

/// <summary> Calculates the duty of the signal </summary>
/// <param name="pulsesp"> Pointer to the Cycle stucture with timings of the signal changes </param>
/// <returns> returns the duty cycle od PWM signal in % (0-100), 0 if can not determine </returns>
float CalculatePWMDuty(Cycle *pulsesp)
{
	float r = -1.0;
	
	if (pulsesp->riseTime && pulsesp->fallTime && (pulsesp->prevRiseTime || pulsesp->prevFallTime)) // at elast both are not zeroes
	{
		noInterrupts();
		Cycle pulses_c = *pulsesp;
		interrupts();
		if (pulses_c.fallTime > pulses_c.riseTime)
//		if (pulses_c.fallTime > pulses_c.riseTime && pulses_c.fallTime > pulses_c.riseTime)
//			if((pulses_c.fallTime - pulses_c.riseTime) <= (pulses_c.fallTime - pulses_c.prevFallTime))
				r = (pulses_c.fallTime - pulses_c.riseTime) * 100.0 / (pulses_c.fallTime - pulses_c.prevFallTime);
		else //if (pulses_c.riseTime > pulses_c.fallTime && pulses_c.riseTime > pulses_c.prevRiseTime)
//			if((pulses_c.riseTime > pulses_c.prevRiseTime) && (pulses_c.fallTime - pulses_c.prevRiseTime) <= (pulses_c.riseTime - pulses_c.prevRiseTime))
				r = (pulses_c.fallTime - pulses_c.prevRiseTime) * 100.0 / (pulses_c.riseTime - pulses_c.prevRiseTime);
	}
	return r;
}

/// <summary> Calculates the frequency of the signal </summary>
/// <param name="pulsesp"> Pointer to the Cycle stucture with timings of the signal changes </param>
/// <returns> returns the frequency in Hz, UNDETERMINED_FREQ if can not determine </returns>
float CalculatePWMFrequency(Cycle *pulsesp)
{
	if (pulsesp->riseTime && pulsesp->fallTime && (pulsesp->prevRiseTime || pulsesp->prevFallTime)) // at elast both are not zeroes
	{
		noInterrupts();
		Cycle pulses_c = *pulsesp;
		interrupts(); 
		unsigned long pulseWidth;
		if (pulses_c.fallTime > pulses_c.riseTime)
			pulseWidth = pulses_c.fallTime - pulses_c.prevFallTime;
		else // if (pulses_c.riseTime > pulses_c.fallTime)
			pulseWidth = pulses_c.riseTime - pulses_c.prevRiseTime;
		if (pulseWidth)
			return 1000000.0 / pulseWidth; // the frequency in Hz, according to the duration of the last whole pulse
	}
	return UNDETERMINED_FREQ;
}

// Some usefull functions

/// <summary> Estimeates the frequency according to the timing of only one pulse </summary>
/// <param name="t"> maximum time in milliseconds to wait for the cycle to end </param>
/// <returns> the frequency in Hz, of 1/t Hz if timedout </returns>
/// <remark> installs the ISR, waits for the cycle to end (or max time waiting to pass)
/// removes the ISR and reports the result </remark>
template <uint8_t PIN>
float OnePulseFreqEstimate(unsigned long t)
{
	typedef PWMinfo<PIN> pwmMonitor;
	pwmMonitor::begin();
	unsigned long start = millis();
	while (!pwmMonitor::valid() && millis() - start < t)
		yield();
	pwmMonitor::end();
	return pwmMonitor::frequency();
}

/// <summary> Estimeates the duty according to the timing of only one pulse </summary>
/// <param name="t"> maximum time in milliseconds to wait for the cycle to end </param>
/// <returns> the duty in % 1-100, ? if timedout </returns>
/// <remark> installs the ISR, waits for the cycle to end (or max time waiting to pass)
/// removes the ISR and reports the result </remark>
template <uint8_t PIN>
float OnePulseDutyEstimate(unsigned long t)
{
	typedef PWMinfo<PIN> pwmMonitor;
	pwmMonitor::begin();
	unsigned long start = millis();
	while (!pwmMonitor::valid() && millis() - start < t)
		yield();
	pwmMonitor::end();
	return pwmMonitor::duty();
}

#endif