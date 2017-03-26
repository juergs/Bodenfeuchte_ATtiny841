/**
 * Narcoleptic - A sleep library for Arduino
 * Copyright (C) 2010 Peter Knight (Cathedrow)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Functionality added by juergs. 
 *
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/common.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "Narcoleptic.h"

#ifndef cbi
	#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
	#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// mod to support attiny85
#if defined(__AVR_ATtiny85__)
	#define WDTCSR WDTCR
#endif

//----------------------------------------------------
SIGNAL(WDT_vect) 
{
  wdt_disable();
  wdt_reset();
  WDTCSR &= ~_BV(WDIE);
  //WDTCR &= ~_BV(WDIE);
}
//----------------------------------------------------
void NarcolepticClass::sleep(uint8_t wdt_period) 
{
  wdt_enable(wdt_period);
  wdt_reset();
  WDTCSR |= _BV(WDIE);           //ATtiny841
  //WDTCR |= _BV(WDIE);         // ATTiny85 only 
  cbi(ADCSRA, ADEN);                    //20160508_juergs added, switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
  wdt_disable();
  sbi(ADCSRA, ADEN);              //20160508_juergs added,  switch Analog to Digitalconverter ON
  WDTCSR &= ~_BV(WDIE);     //ATtiny841
  //WDTCR &= ~_BV(WDIE);    //ATtiny85
}
//----------------------------------------------------
//--- allows to specify a sleep time in minutes.
void NarcolepticClass::delay_minutes(int minutes)
{
		// --- 30 sec (* 1000 ms) * 2 = 1 Minute 
  for (int count = 0; count < (minutes * 2); count++){
    Narcoleptic.delay_millis(30000);
  }
}
//----------------------------------------------------
void NarcolepticClass::delay_millis(unsigned int milliseconds) {
  while (milliseconds >= 8000) { sleep(WDTO_8S); milliseconds -= 8000; }
  /*
  if (milliseconds >= 4000)    { sleep(WDTO_4S); milliseconds -= 4000; }
  if (milliseconds >= 2000)    { sleep(WDTO_2S); milliseconds -= 2000; }
  if (milliseconds >= 1000)    { sleep(WDTO_1S); milliseconds -= 1000; }
  if (milliseconds >= 500)     { sleep(WDTO_500MS); milliseconds -= 500; }
  if (milliseconds >= 250)     { sleep(WDTO_250MS); milliseconds -= 250; }
  if (milliseconds >= 125)     { sleep(WDTO_120MS); milliseconds -= 120; }
  if (milliseconds >= 64)      { sleep(WDTO_60MS); milliseconds -= 60; }
  if (milliseconds >= 32)      { sleep(WDTO_30MS); milliseconds -= 30; }
  if (milliseconds >= 16)      { sleep(WDTO_15MS); milliseconds -= 15; }
  */
}

//----------------------------------------------------
// the instance
NarcolepticClass Narcoleptic;
