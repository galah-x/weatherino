//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'weatherino temp humidity pressure rainfall wind for moteino Time-stamp: "2019-03-23 16:31:32 john"';

// $ grabserial -b 19200 -d /dev/ttyUSB1 | ts [%y%m%d%H%M%S]



#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <RFM69_OTA.h>     // allow OTA reprogramming
#include <LowPower.h>      //get library from: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/






//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        3   //unique for each node on same network
#define GATEWAYID     1  //node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
// #define USE_ENCRYP
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define SENDLOOPS    80 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send


//*********************************************************************************************
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST    SLEEP_250MS
#define SLEEP_SEC     SLEEP_1S
#define SLEEP_LONG    SLEEP_2S
#define SLEEP_LONGER  SLEEP_4S
#define SLEEP_LONGEST SLEEP_8S

// period_t sleepTime = SLEEP_LONGEST; //period_t is an enum type defined in the LowPower library (LowPower.h)
//#define SLEEP_MULTIPLIER 75
// 10 minutes is 600 seconds / 8s sleep is 75

// speed up for testing
period_t sleepTime = SLEEP_2S;    //period_t is an enum type defined in the LowPower library (LowPower.h)
#define SLEEP_MULTIPLIER 300
// #define SLEEP_MULTIPLIER 3

//*********************************************************************************************
#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23
#else
  #define LED           9 // Moteinos have LEDs on D9
  #define FLASH_SS      8
#endif
//#define TRIG           6  // digital pin wired to TRIG pin of ultrasonic sensor
//#define ECHO           7  // digital pin wired to ECHO pin of ultrasonic sensor
//#define SENSOR_EN        4  // digital pin that drives the pullup on the float sensor. High to read sensor.
//#define FLOATSW          5  // digital input from the floatsw. shorted == 0 == empty. Open == 1 == full. 
//#define BUZZER         4  // digital pin that is connected to onboard buzzer
//#define MAX_DISTANCE 300  // maximum valid distance
//#define MIN_DISTANCE   2  // minimum valid distance
//#define MAX_ADJUST_DISTANCE (MAX_DISTANCE-GRN_LIMIT_UPPER)   //this is the amount by which the RED_LIMIT_UPPER can by increased

//******************************************************************************************
// IOs
// BMP280 is on I2C
// SHT31  is on I2C at address 0x44
// sda = A4
// scl = A5

// rain interrupt  7
// wind interrupt  8
// wind direction  A7
#define WIND_DRN_ADC_CH 7
#define BATT_ADC_CH 6
#define RainInt 6
#define WindInt 7
//
// timers used 


// calibration coefficient for wind drn. 0 .. 1023

// taped windvane.
// compass from in front indicated it was at 200 degrees
// subtracting 180 implies it was pointed at 20 degrees magnetic
// local declination at -31.45 degrees 151.62 elevation 950 is
// http://www.ga.gov.au/oracle/geomag/agrfform.jsp
// Requested: Latitude -31o 27' 00", Longitude 151o 37' 00", Elevation .95 km, Date 2015/01/1 
// Calculated: Latitude -31.4500o, Longitude +151.6167o, Elevation 0.95 km, Epoch 2015.0000
// Magnetic Field Components
// D = 11.807 deg

// from wikipedia Declination Magnetic
// D, the magnetic declination (sometimes called the magnetic variation), is the angle between the horizontal
// component of the magnetic field and true north. It is positive when the compass points east of true north,
// and negative when the compass points west of true north.
//
// so compass 20 degrees magnetic is 20-11.8 = 8 degrees east of north 
// so given the adc range is 1024 and that represents 360 degrees, thats an adc count of 12.2 * 1024/360 = 35  
// the system reported [181120192600] 03 WindDrn=S 180°
// or an adc count of 180/360 * 1024 = 512
// so I need a cal of 35 - 512 = -477  or 1024 - 477 = 547

// #define drn_cal 0
#define drn_cal 547





#ifdef SERIAL_EN
#define SERIAL_BAUD   19200
#define DEBUG(input)   {Serial.print(input);}
#define DEBUGln(input) {Serial.println(input);}
#define SERIALFLUSH() {Serial.flush();}
#else
#define DEBUG(input);
#define DEBUGln(input);
#define SERIALFLUSH();
#endif


byte sendLoops=0;
//byte distReadLoops=0;
//byte battReadLoops=0;
//float distance=0;
//float prevDistance=0;
//float batteryVolts = 5;
#define BUFLEN 50
char buff[BUFLEN]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[10];    // for float conversion


RFM69 radio;
SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

int windloop_times;
int batteryloop_times;

// due to a lightning strike, separating weatherino into 2 parts. This part is physically high
// mounted near the rain and wind sensors. Want short wires due to high field strength.
// Temperature sensors are 1.2m off the ground. If I connect to them too I get long wires.

#define RAIN
#define WIND
#define BATTERY

// about 4 times per day is 6 hrs or 360 minutes 
// with 8 minute inner loop update
#define BATTERYLOOP_TIMES 45

unsigned int  rain_raw;
unsigned long wind_raw;

long unsigned current_msec;
long unsigned last_gust_msec;
unsigned int  gust_period;
unsigned int  used_gust_period;
float         biggest_gust;
float         this_gust;
unsigned int  this_gust_count;
long unsigned this_wind_count;
long unsigned last_wind_count;
  


void setup() {
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif	 
#ifdef USE_ENCRYPT
  radio.encrypt(ENCRYPTKEY);
#else
  radio.encrypt(null);
#endif
  
#ifdef SERIAL_EN
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));
#endif
  
  sprintf(buff, "%02x weatherino 20190317", NODEID );  
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));

  flash.initialize();
  
  windloop_times = 0;
  batteryloop_times = 0;
    
  radio.sleep();

#ifdef RAIN
  pinMode(RainInt, INPUT);
  // enable interrupt for pins
  pciSetup(RainInt);
  rain_raw = 0;
#endif
  
#ifdef WIND
  pinMode(WindInt, INPUT);
  // enable interrupt for pins
  pciSetup(WindInt);

  wind_raw = 0;
  last_gust_msec = millis();
  biggest_gust = 0;;
  last_wind_count = 0;
#endif

}

/************************** MAIN ***************/

  
#define min_gust_period 8400

  
void loop() {
  
  int batt_adc;
  int ref_v;
  float batt_v;

  // gust update
  current_msec = millis();
  gust_period = current_msec - last_gust_msec;
  if (gust_period >= min_gust_period)
    {
      used_gust_period = gust_period;
      this_wind_count = get_wind_count();
      this_gust_count = this_wind_count - last_wind_count;
      last_wind_count = this_wind_count;
      last_gust_msec = current_msec;
      
      // gust count is max revs in 8.388 seconds
      // where v[mph] = 2.25 * count / T
      // or v[kph]    = 2.25 * 1.609 * count / T
      // or v[kph]    = 3.62 * count / T
      this_gust = 3620.0 * this_gust_count / gust_period; 
      if (this_gust > biggest_gust)
	{
	  biggest_gust = this_gust;
	}
    }
  
  if (windloop_times == 0) {

    
    // *************** rainfall  **************
    // rain count
    float r = get_rain_count() / 5.0;  // 0.2mm per blip on int pin
    dtostrf(r, 5, 1, buff2);
    
    sprintf(buff, "%02x rainfall=%smm", NODEID, buff2 );  
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    
    
    // *************** wind direction **************

    int drn;
    unsigned int drn_temp;
    drn = analogRead(WIND_DRN_ADC_CH);
    drn = drn + drn_cal
;
    if (drn > 1023)
      { 
	drn = drn - 1023;
      }

    dirn2str(drn, buff2);  // update drnstr
    
    drn_temp = 36 * drn ;
    
    sprintf(buff, "%02x WindDrn=%s %u°", NODEID, buff2, drn_temp / 103);
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    delay(50);
    
    // **************** wind count *****************
    dtostrf(biggest_gust, 5, 1, buff2);
    sprintf(buff, "%02x Wind gust=%s Kph dist=%lu", NODEID, buff2, get_wind_count());
    radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
    biggest_gust=0;

    windloop_times = SLEEP_MULTIPLIER;

    if (batteryloop_times == 0)
      {
	delay(50);
	batt_adc =  analogRead(BATT_ADC_CH);
	ref_v =  readVref();

	// shouldn't happen, but whatever.  Clearly gives an incorrect value.
	if (ref_v == 0)
	    ref_v = 1;

	// restore the Vdd adc reference after futzing with it as part of readVref
	analogReference(DEFAULT);
      
	// reading the reference as a fraction of Vdd allows me to calculate vdd
	// (1) Vdd = 1023 * v_ref / ref_v;
	// check:  vdd = 1023 * 1.1 / 337 = 3.34
	
	// then reading the scaled external voltage as a fraction of Vdd allows me to calculate ext voltage
	// (2) batt_v = (1000000 + 360000) / 1000000 * Vdd * (batt_adc / 1023);
	// check: batt_adc = 928, batt_v = 4.12
	// expanding
	// (3) batt_v = (1000000 + 360000) / 1000000 * 1023 * (v_ref /ref_v) * (batt_adc / 1023);
	// extracting constants
	// (4) batt_v = v_ref * (1000000 + 360000) / 1000000 )  * batt_adc / ref_v );
	// check 1.496 * 929 / 337 = 4.12
	// actual 4.07
	// implies ref is really 4.12 / 4.07 = 1.085
	
	batt_v = 1.476 * batt_adc / ref_v; 
	dtostrf(batt_v, 5, 2, buff2);
	sprintf(buff, "%02x Batt=%sV", NODEID, buff2  );  
	radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	
	batteryloop_times = BATTERYLOOP_TIMES;
	
      }
    batteryloop_times--;
    
  }
  if (radio.receiveDone())
    CheckForWirelessHEX(radio, flash, true);
  
  
  windloop_times--;
  delay(2000);
  // LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF); //put microcontroller to sleep to save battery life
}



// N NNE NE ENE   

void dirn2str(uint16_t adcval, char *drnstr)
{
  uint8_t heading ;
  heading = adcval >> 2; 
  if (heading <  16 )
    strcpy(drnstr,"N");
  else if (heading < 32)
    strcpy(drnstr, "NNE");
  else if (heading < 48)
    strcpy(drnstr, "NE");
  else if (heading < 64)
    strcpy(drnstr, "ENE");
  else if (heading < 80)
    strcpy(drnstr, "E");
  else if (heading < 96)
    strcpy(drnstr, "ESE");
  else if (heading < 112)
    strcpy(drnstr, "SE");
  else if (heading < 128)
    strcpy(drnstr, "SSE");
  else if (heading < 144)
    strcpy(drnstr, "S");
  else if (heading < 160)
    strcpy(drnstr, "SSW");
  else if (heading < 176)
    strcpy(drnstr, "SW");
  else if (heading < 192)
    strcpy(drnstr, "WSW");
  else if (heading < 208)
    strcpy(drnstr, "W");
  else if (heading < 224)
    strcpy(drnstr, "WNW");
  else if (heading < 240)
    strcpy(drnstr, "NW");
  else 
    strcpy(drnstr, "NNW");
}


// Install Pin change interrupt for a pin, can be called multiple times
 
void pciSetup(byte pin)
{
  cli();
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
  sei();
}

// Use one Routine to handle each group

// oops, D8 is used by eerom. Wind is on D6 on this version of weatherino (rain + wind only)

//ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
//{
//  static byte lastWindLvl = 0;
//  
//  // read the port, once.
//  byte WindLvl = digitalRead(WindInt);
//  
//  // negedge only
//  if ((lastWindLvl == HIGH) && (WindLvl == LOW))
//    {
//      wind_raw++;
//    }  
//  lastWindLvl = WindLvl;  // save for next time 
//  
//  digitalWrite(13,digitalRead(WindInt));
//}
//

// RainInt - debounced
// wind int, which seems not to need a debounce. Its a 1ms pulse.
// at least on both I want to use negedge only
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  static unsigned long last_used_rain_interrupt_time = 0;
  static byte last_wind_lvl = 1;
  static byte last_rain_lvl = 1;
  unsigned long interrupt_time = millis();
  byte wind_lvl = digitalRead(WindInt);
  byte rain_lvl = digitalRead(RainInt);
  
  // use only negedge for wind
  if ((last_wind_lvl == HIGH) && (wind_lvl == LOW))
    wind_raw ++;
  
  last_wind_lvl = wind_lvl;
  
  // only use negedge for rain, 
  if ((last_rain_lvl == HIGH) && (rain_lvl == LOW))
    // and debounce it at 100ms.
      if (abs(interrupt_time - last_used_rain_interrupt_time) > 50)
	// apply 50ms debounce on rain transition.
	// the abs() is to cope with wrap around of millis. Doesn't happen often, but I'd lose a lot when it does finally wrap. 
	{
	  last_used_rain_interrupt_time = interrupt_time;
	  rain_raw++;
	}
  last_rain_lvl = rain_lvl;
  // digitalWrite(13,lvl);
}

unsigned long  get_wind_count (void)
{
  unsigned long  wind_count;
  cli();
  wind_count = wind_raw;
  sei();
  return (wind_count);
}

unsigned int get_rain_count (void)
{
  unsigned int rain_count;
  cli();
  rain_count = rain_raw;
  sei();
  return (rain_count);
}


int readVref() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  int result = (high<<8) | low;
  
  return result; // Vref as a fraction of vcc
}

 
 
