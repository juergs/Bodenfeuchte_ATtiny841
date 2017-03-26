
#include <Arduino.h>
#include "LaCrosse.h"

#ifndef PIN_SEND
	#define	PIN_SEND	 7  // ATTiny DIP.2 = PB3 is D3  
#endif 	

#ifndef SENSORID
	#define SENSORID		120
#endif 

#define MODELID		0x0A   //--- LaCrosse-Protocol: Fix defines a TX3 Sensor
#define COM_TEMP	0x00   //--- Command key for Transmitting Temperature
#define COM_HUM		0x0E   //--- Command key for Transmitting Humidity
#define COM_PRESS 0x0F   //--- geraten

#define DELAYTIME	60				//--- 1 Minutes delay between Measaurements
#define REPEATMSG   3				//--- No of repeats of radio message

uint8_t bSensorId = 120;

//--- Message String for // Serial Debug + sprintf buffers
char msg[80];
char tmsg[10];
char hmsg[10];

int tens, ones, tenths;   //--- Nibles for tens, ones and tenths
int checksum;             //--- Nible for Checksum

//--- Dynamic nibles which will be calculated 
int nible1, nible2, nible3, nible4, nible5;
int parity		= 0;       //--- Parity Bit

//--- Duration of pulses and delays
int iLongPulse	= 975;
int iShortPulse = 250;
int iDelay		= 1450;

//--- Temperature and Hunidity
//float t, h;

//---------------------------------
void LaCrosseClass::setTxPinMode(byte value)
{
	pinMode(PIN_SEND, value);
  digitalWrite(PIN_SEND, LOW); // initialize, too!
}

void LaCrosseClass::setSensorId(byte id)
{
	bSensorId = id; 
}
//---------------------------------
void LaCrosseClass::O() 
{ // sends zero
	digitalWrite(PIN_SEND, HIGH);
	delayMicroseconds(iLongPulse);
	digitalWrite(PIN_SEND, LOW);
	delayMicroseconds(iDelay);
}
//---------------------------------
void LaCrosseClass::I() 
{ // sends onde
	digitalWrite(PIN_SEND, HIGH);
	delayMicroseconds(iShortPulse);
	digitalWrite(PIN_SEND, LOW);
	delayMicroseconds(iDelay);
}
//---------------------------------
void LaCrosseClass::CalcNibles()
{
	int idr;
	//-- left  nible of Model
	nible1 = MODELID / 0x10;
	// right  nible of Model
	nible2 = MODELID & 0xF;

	// Nible 4 is the left part of the SensorId
	// 1101 111?
	// 0110 1111
	nible4 = bSensorId / 0x08;
	//// Serial.println(nible4);

	// For Parity we need the last 3 bits of the Sensor ID
	idr = bSensorId & 0x7;

	// Parity bit makes even parity for sum 
	parity = (nible4 + idr + tens + ones + tenths + tens + ones) & 0x1;

	// nible 5 is last 3 digits of Sensor and parity bit
	nible5 = ((idr * 0x2) + parity) & 0xf;

	// Checksum nible of all nibles
	checksum = (nible1 + nible2 + nible3 + nible4 + nible5 + tens + ones + tenths + tens + ones) & 0xF;
}
//-----------------------------------
void LaCrosseClass::SendNibble(int i)
{
	if (i & 8) I(); else O();
	if (i & 4) I(); else O();
	if (i & 2) I(); else O();
	if (i & 1) I(); else O();
	sprintf(msg, "%x", i);
}
//-----------------------------------
void LaCrosseClass::sendNibles()
{
	int i;
	CalcNibles();
	for (i = 0; i < REPEATMSG; i++)
	{
		SendNibble(nible1);
		SendNibble(nible2);
		SendNibble(nible3);
		SendNibble(nible4);
		SendNibble(nible5);
		SendNibble(tens);
		SendNibble(ones);
		SendNibble(tenths);
		SendNibble(tens);
		SendNibble(ones);
		SendNibble(checksum);
		delay(20);
 
 #ifdef VERBOSE_MODE   
    Serial.print("h=");
    Serial.println(h,DEC);
    Serial.print("nible1=");
    Serial.print(nible1,BIN);
    Serial.print("\t");
    Serial.print("nible2=");
    Serial.print(nible2,BIN);
    Serial.print("\t");
    Serial.print("nible3=");
    Serial.print(nible3,BIN);
    Serial.print("\t");
    Serial.print("nible4=");
    Serial.print(nible4,BIN);
    Serial.print("\t");
    Serial.print("nible5=");
    Serial.print(nible5,BIN);
    Serial.println();
    Serial.print("tens     = ");
    Serial.print(tens,DEC);
    Serial.print("\t");
    Serial.println(tens,BIN);
    Serial.print("ONES     = ");
    Serial.println(ones,DEC);
    Serial.print("TENTHS   = ");
    Serial.println(tenths,DEC);
    Serial.print("CheckSum = ");
    Serial.println(checksum,DEC);    
 #endif 
	}
}
//-----------------------------------
void LaCrosseClass::CalcTemp()
{
	// Temperature has an offset of 50 degrees, makes possible to transmit neg. Values
	tens = (int(t) / 10) + 5;
	ones = (int(t) % 10);
	tenths = (int(t*10.0) % 10);

	// We will send a temperature
	nible3 = COM_TEMP;
}
//-----------------------------------
void LaCrosseClass::CalcHum() 
{
	// Humidity values go from 00.0 to 99.99, so no negative adjustment neccessary
	tens = (int(h) / 10);
	ones = (int(h) % 10);
	tenths = (int(h*10.0) % 10);
	nible3 = COM_HUM;
}
//-----------------------------------
void LaCrosseClass::CalcPress() 
{
  //--- AirPressure readings, to be positive subtract 800 hPa
     //--- case 1.) 890 hPa = 890 -800 = 90  case 2.) 1013 hPa = 1013 - 800 = 213   
  tens = (int(h) / 10);
  ones = (int(h) % 10);
  tenths = (int(h*10.0) % 10);
  nible3 = COM_HUM;
  
  //sprintf(msg, "h=%d | tens = %d | ones=%d | tenths=%d",h,tens,ones,tenths );
  //Serial.println(msg);
#ifdef VERBOSE_MODE  
  Serial.print("CalcPress::H=");
  Serial.print(h,DEC);
  
  Serial.print(" CalcPress::TENS=");
  Serial.print(tens,DEC);
  
  Serial.print(" CalcPress::ONES=");
  Serial.print(ones,DEC);

  Serial.print(" CalcPress::TENTHS=");
  Serial.print(tenths,DEC);
  Serial.println();  
#endif 
}
//-----------------------------------
void LaCrosseClass::sendTemperature()
{
	CalcTemp();
	sendNibles();
}
//-----------------------------------
void LaCrosseClass::sendHumidity()
{
	CalcHum();
	sendNibles();
}
//-----------------------------------
void LaCrosseClass::sendPress()
{
  CalcPress();
  sendNibles();
}
//-----------------------------------
void LaCrosseClass::sleep(int sek)
{
	int i;	
	for (i = 0; i < sek; i++)
	{
		delay(1000);
	}
}
/* need space!!! 
//-----------------------------------
int LaCrosseClass::freeRam()
{
	extern int __heap_start, *__brkval;
	int v;
	return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}
*/
//---------------------------------------------------------
//---------------------------------------------------------
//--- set class-instance 
LaCrosseClass LaCrosse;
//---------------------------------------------------------
