/*************************************************************************
  Bodenfeuchte ATTINY841, with deep sleep  
  ************************************************************************
  20170325_juergs: initial version, 
                   based on ATTiny85-version with improvements. 
  *************************************************************************
  Remarks:
    Bodenfeuchte_433MHz_ATiny841_V1.ino.hex: 8.138 / 8.192 Bytes (99,34%)
    Sketch uses 8136 bytes (99%) of program storage space. Maximum is 8192 bytes.
    Flash-Space is full (with Serial) !
**************************************************************************/
//#define SN ("Bodenfeuchte-Sensor-433-ATtiny841")
//#define SV ("V3.0 vom 25.03.2017")

#include "Narcoleptic.h"
#include "LaCrosse.h"
#include "OneWire.h"
//----------------------------------------
//--- conditionals, 
//--- zum aktivieren Kommentierung entfernen            
//----------------------------------------
//#define USE_SEPARATE_BATTERIE_ID 
//----------------------------------------
#define USE_WITH_DALLAS_SENSOR 
#define USE_SERIAL_FOR_DEBUG 
//----------------------------------------
#define SENSORID_BODENFEUCHTE     120
#define SENSORID_BATTERIE         121
//----------------------------------------
#define LED_PB2                   2     //--- needed only for tindie-breakboard: is buildin LED (possible free pin)
#define DUO_LED_GN                3
#define DUO_LED_RT                4
//----------------------------------------
#define TX_433_PIN                7     // Hardware-setup, auch in LaCross.cpp setzen! 
#define DALLAS_SENSOR_PIN         10    // pullup resistor 4K7 - Data to VCC
#define BODENFEUCHTE_POWER_PIN    0     //  war 6 (SCK) stört Prorammierung 
#define BODENFEUCHTE_SENSOR_PIN   1     //--- must be INT0: PB1, unique on 841 
//----------------------------------------
#define DEEP_SLEEP_MINUTES        2     // Deep_Sleep-Timeout, with no power consumption should be below 10 yS
#define MAIN_PERIOD               998   // How long to count pulses, for a fixed frequency this should be 998 ms (for a total runtime of 1 sec with 1 MHz oscillator, but we are on 8MHz, check impulse on it)
#define SETTLE_TIME               1000  // Waiting time in ms between powering up AMV and stable frequency
//----------------------------------------
#define OW_ROMCODE_SIZE           8

#define POWERAMV                  BODENFEUCHTE_POWER_PIN    // Output line to provide power for the AMV
#define INPUTFREQ                 BODENFEUCHTE_SENSOR_PIN   // IRQ input for frequency count. Must be 2 or 3

#define MAIN_PERIOD               998     // How long to count pulses, for a fixed frequency this should be 998 ms (for a total runtime of 1 sec with 1 MHz oscillator)
#define SETTLE_TIME               1000    // Waiting time in ms between powering up AMV and stable frequency


//--- set 18B20-instance
OneWire  dallas(DALLAS_SENSOR_PIN);     // on arduino port (a pullup 4.7K resistor is necessary, between Vcc and DQ-Pin   1=GND 2=DQ 3=Vcc )

//-- Globales 
volatile unsigned int   pulsecount        = 0;           // counts during open sample-window
unsigned int            average           = 0;          // IIR floating average filter
byte                    led_temporary     = 1;        // Are LEDs jumpered to stay on=0, otherwise 1 
float                   bodenfeuchte      = 0.0;
float                   batteriespannung  = 0.0;
float                   controller_VCC    = 0.0; 

//-----------------------------------------------------------
//--- prototypes 
//-----------------------------------------------------------
float DoBodenFeuchteMeasurement();
float ReadSingleOneWireSensor(OneWire ds); 
//-----------------------------------------------------------
/* ATTINY841-Version only!
  measures the supply voltage internally using the ADC.
  returns the ADC-value of the 1.1 band gap input measured 
  with VCC as reference voltage. This value can be converted 
  to the VCC voltage by using this equation:
      VCC=1024*1.1/ADC;
  https://cpldcpu.com/2014/11/16/ws2812_length/
  https://github.com/cpldcpu?tab=repositories
  20170326_juergs: umgebaut. 
*/
//-----------------------------------------------------------
uint16_t MeasureVCC(void)
{
  uint16_t sum=0;
  uint8_t i;

  //--- ATTiny841-Version only!
  PRR    &= ~_BV(PRADC);                     // ADC power on
  ADCSRA = _BV(ADEN)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); 
  ADMUXB = 0;                             // Reference is Vcc
  ADMUXA = 0x0d;                          // Measure internal 1.1v reference  
  delay(2);                         // Wait for Vref to settle
  ADCSRA |=_BV(ADSC);               // Start conversion
  while (!(ADCSRA&_BV(ADIF)));      //~100 us
  ADCSRA |= _BV(ADIF);               // Clear ADIF
  uint8_t  low = ADCL;               // bytewise, must read ADCL first - it then locks ADCH  
  uint8_t  high = ADCH;              // unlocks both
  uint16_t result = (high << 8) | low;
  // ========================================================================================
  // *  Berechnung/Skalierung mit manueller Messung der Betriebsspannung:
  // *        VCC=1024*1.1/ADC;
  // *        internal1.1Ref = 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
  // *                       = 1.1 * 5126      / 5258 => 1.09 ==> 1.09*1023*1000 = 1097049
  // ========================================================================================
  result = 1125300L / result;       // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000  
  return result;                    // liefert Vcc in millivolts  
}
//-----------------------------------------------------------
/* space needed, if not used, compiler will drop too */ 
void blink(byte pin, int delay_ms)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(delay_ms);
  digitalWrite(pin, LOW);
  delay(delay_ms);
}
//-----------------------------------------------------------
//--- the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_PB2, OUTPUT);
  pinMode(DUO_LED_GN, OUTPUT);
  pinMode(DUO_LED_RT, OUTPUT);

  //--- setup , switch power for AMV off. Preset Pin as Output.
  //--- sensor pin is set during measurement  
  pinMode(BODENFEUCHTE_POWER_PIN, OUTPUT);    // PowerPin fpor soilmoisture-circuit 
  digitalWrite(BODENFEUCHTE_POWER_PIN, LOW);  // off
  
  //--- 433MHz-sender-module (STX882): set first SensorId & TX instance 
  LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;  
  LaCrosse.setTxPinMode(TX_433_PIN);              // and set there to LOW, too. 

  #ifdef USE_SERIAL_FOR_DEBUG
    Serial.begin(38400);
    delay (2000); // !Serial doesn't work, wait instead   
    Serial.println(); Serial.println ("Hi!");  
  #endif 
}
//-----------------------------------------------------------
// the loop function runs over and over again forever
void loop() 
{
  #ifdef USE_SERIAL_FOR_DEBUG
    Serial.println("- woke up.");    
  #endif 
    
  delay(750); // ms, needed for settling DS18B20
 
  //--- [1] Bodenfeuchte auslesen 
  digitalWrite(DUO_LED_GN, HIGH);    
  bodenfeuchte = DoBodenFeuchteMeasurement(); 
  digitalWrite(DUO_LED_GN, LOW);    

  //--- [2] read Dallas-Sensor
  digitalWrite(DUO_LED_RT, HIGH);   
  float theta = ReadSingleOneWireSensor(dallas);
  digitalWrite(DUO_LED_RT, LOW);   
  
  uint16_t intVcc = MeasureVCC(); 
  float controllerVCC = intVcc / 1000.0;
  
  #ifdef USE_SERIAL_FOR_DEBUG
    Serial.print("Vcc_1: "); Serial.println(intVcc);     
  #endif 

  //--- transfer measured values to LaCrosse-instance
  // [1] float  temperature
  
  LaCrosse.bSensorId = SENSORID_BODENFEUCHTE;
  LaCrosse.t = theta;                 //--- send temperature;  
  LaCrosse.sendTemperature(); 
         
  digitalWrite(LED_PB2, HIGH);
  LaCrosse.sleep(1);       
  digitalWrite(LED_PB2, LOW); 
  
  // [2] float  humidity
  LaCrosse.h = bodenfeuchte/1000;     //--- send soil-moisture;  
  LaCrosse.sendHumidity();
  
  digitalWrite(LED_PB2, HIGH);
  LaCrosse.sleep(1);        /* 1 second, no power-reduction! */
  digitalWrite(LED_PB2, LOW);
  
  //--- Versorgungsspannung mit eigener ID senden 
  // [3] float VCC as humidity
  digitalWrite(LED_PB2, HIGH);
  LaCrosse.bSensorId = SENSORID_BATTERIE;
  LaCrosse.h = controllerVCC;   //--- internal VCC-reading as float
  LaCrosse.sendHumidity();  
  digitalWrite(LED_PB2, LOW);
 
  //--- need some delay before transmission (timers) will be forced off. 
   #ifdef USE_SERIAL_FOR_DEBUG    
    delay(500);   //before transmission (timers) will be switched off. 
   #else
    delay(5); 
  #endif 
  
  //--- fall to deep powersave-sleep
  Narcoleptic.delay_minutes(DEEP_SLEEP_MINUTES);
}
//---------------------------------------------------------------------
void myinthandler() 
{
    //--- Interrupt_0- handler acts on INT0-falling edges
    pulsecount++;
}
//---------------------------------------------------------------------
float DoBodenFeuchteMeasurement()
{
    //--- [1] setup Multivibrator 555 
    pinMode(BODENFEUCHTE_SENSOR_PIN, INPUT_PULLUP);   // internal Pullups are strong 10K, change to external 47K resistors ? 
    digitalWrite(BODENFEUCHTE_SENSOR_PIN, HIGH);      //--- activate pullups, falling edges are counted  
    
    pinMode(BODENFEUCHTE_POWER_PIN, OUTPUT);      
    digitalWrite(BODENFEUCHTE_POWER_PIN, HIGH);   //--- begin to power up sensor circuit  
        
    //--- [2] wait for settle
    delay(SETTLE_TIME);

    //--- [3] Prepare measurement, init
    pulsecount = 0;
   
    //--- ATtiny841 has INT0 only, starts sampling window  
    attachInterrupt(0, myinthandler, FALLING); 

    //--- [4] wait for measurement to finalize
    delay (MAIN_PERIOD);

    //--- [5] store actual counter value
    //---     register counts per period (frequency) and calculate IIR
    unsigned long _pulses = pulsecount;
    pulsecount = 0; 

    //--- simple floating average
    average += _pulses;
    if(average != _pulses)    //--- during startup both are equal
      average >>= 1;

    //--- [6] stop to measure and AMV
    digitalWrite(BODENFEUCHTE_POWER_PIN, LOW);

    //--- [7] detach Interrupt_0: this ends sampling window 
    detachInterrupt(0);

    //--- [8] result is reading for TX-humidity 
   return (average * 1.0);       
}
//-------------------------------------------------------------------------
float ReadSingleOneWireSensor(OneWire ds)
{
  //--- 18B20 stuff
  byte i;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius = 12.3;

  if (!ds.search(addr))
  {
    ds.reset_search();
    delay(250);
    return celsius;
  }

  if (OneWire::crc8(addr, 7) != addr[7])
  {
    return celsius;
  }

  //-- we know: we are using Dallas-chip  DS18B20
  type_s = 0;
   
  /* 8K Flash, we need space !!! 
  //--- the first ROM byte indicates which chip
  switch (addr[0])
  {
  case 0x10:
    //Serial.println("  Chip = DS18S20");  // or old DS1820
    type_s = 1;
    break;
  case 0x28:
    // Serial.println("  Chip = DS18B20");
    type_s = 0;
    break;
  case 0x22:
    // Serial.println("  Chip = DS1822");
    type_s = 0;
    break;
  default:
    // Serial.println("Device is not a DS18x20 family device.");
    return celsius;
  }
  */ 

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1 );  // start conversion, use alternatively ds.write(0x44,1) with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
                   // we might do a ds.depower() here, but the reset will take care of it.

  ds.reset();    //--- DS18B20 responds with presence pulse
  
  //--- match ROM 0x55, sensor sends ROM-code command ommitted here.
  ds.select(addr);
  ds.write(0xBE);         //--- read scratchpad
  for (i = 0; i < 9; i++)
  {
    //--- we need 9 bytes, 9th byte is CRC, first 8 are data
    data[i] = ds.read();
  }

  //--- Convert the data to actual temperature
  //--- because the result is a 16 bit signed integer, it should
  //--- be stored to an "int16_t" type, which is always 16 bits
  //--- even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s)
  {
    raw = raw << 3;     //--- 9 bit resolution default
    if (data[7] == 0x10)
    {
      //--- "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    };
  }
  else
  {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
                        //// default is 12 bit resolution, 750 ms conversion time
  };

  celsius = (float)raw / 16.0;    //fahrenheit = celsius * 1.8 + 32.0;

  //---- Error: check if any reads failed and exit early (to try again).  
  if (isnan(celsius))
  {
    //--- signalize error condition 
    celsius = -99.9;
  };
  return celsius;
}
//-----------------------------------------------------------
//-----------------------------------------------------------
//-----------------------------------------------------------
//<eot>
