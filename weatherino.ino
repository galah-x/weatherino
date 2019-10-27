//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'weatherino temp humidity pressure rainfall wind for moteino Time-stamp: "2019-10-28 07:31:12 john"';

// $ grabserial -b 19200 -d /dev/ttyUSB1 | ts [%y%m%d%H%M%S]



#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPIFlash.h>      //get it here: https://www.github.com/lowpowerlab/spiflash
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <RFM69_OTA.h>     // allow OTA reprogramming
#include <LowPower.h>      //get library from: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
// #include <math.h>          // atan
#include <avr/wdt.h> // watchdog





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
// #define SLEEP_MULTIPLIER 300


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



// #define SERIAL_EN
#define RADIO
#define RAIN
#define WIND
#define BATTERY
// speed up poll rates
// #define DEBUG


// #define drn_cal 0
#ifdef DEBUG
#define drn_cal 0
#else
#define drn_cal 547
#endif
#define SERIAL_BAUD   115200


byte sendLoops=0;
//byte distReadLoops=0;
//byte battReadLoops=0;
//float distance=0;
//float prevDistance=0;
//float batteryVolts = 5;
#define BUFLEN 60
char buff[BUFLEN]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[20];    // for float conversion
char buff3[10];    // for float conversion

#ifdef RADIO
RFM69 radio;
#endif

SPIFlash flash(FLASH_SS, 0xEF30); //EF30 for 4mbit  Windbond chip (W25X40CL)

uint16_t gust_averages;
uint8_t batteryloop_times;
#ifdef DEBUG
const uint16_t GUST_AVERAGES=2;
const uint8_t BATTERYLOOP_TIMES=5;
#else
const uint16_t WINDLOOP_TIMES=300;
const uint8_t BATTERYLOOP_TIMES=45;
#endif
// due to a lightning strike, separating weatherino into 2 parts. This part is physically high
// mounted near the rain and wind sensors. Want short wires due to high field strength.
// Temperature sensors are 1.2m off the ground. If I connect to both of them I get long wires.


// about 4 times per day is 6 hrs or 360 minutes 
// with 8 minute inner loop update


uint16_t rain_raw;
uint32_t wind_raw;

uint32_t  current_msec;
uint32_t  last_gust_msec;

uint16_t  gust_period;
const uint16_t min_gust_period =8400;

float     biggest_gust;
uint16_t  this_gust_count;
uint16_t  biggest_gust_count;
uint32_t  this_wind_count;
uint32_t  last_wind_count;

// these variables are used to perform vector addition of wind per 8s gust period
// to allow wind direction averaging.
int32_t total_easting;
int32_t total_northing;

const uint8_t interp_points = 8;
const uint8_t interp_shift = 5;
const uint8_t interp_and = 31;
// sin * 10k. used for first quadrant only.
const uint16_t sin_points[interp_points+1] = { 0, 1951, 3827, 5556, 7071, 8315, 9239, 9808, 10000};
const uint8_t sin_slope[interp_points] = {61, 59, 54, 47, 39, 29, 18, 6};





void setup() {
  
#ifdef RADIO
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif	 
#ifdef USE_ENCRYPT
  radio.encrypt(ENCRYPTKEY);
#else
  radio.encrypt(null);
#endif
#endif

#ifdef SERIAL_EN
  Serial.begin(SERIAL_BAUD);
#endif
  
  sprintf(buff, "%02x weatherino 20191027", NODEID );  
#ifdef SERIAL_EN
  Serial.println(buff);
#endif

#ifdef RADIO    
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
  delay(200);
#endif
  
  flash.initialize();
  
  gust_averages = GUST_AVERAGES;
  batteryloop_times = 1;
    
#ifdef RADIO    
  radio.sleep();
#endif
  
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
  biggest_gust_count = 0;
  last_wind_count = 0;
  total_easting = 0;
  total_northing = 0;
  
#endif
  wdt_enable(WDTO_8S);
}

/************************** MAIN ***************/

  

  
void loop() {
  
  int batt_adc;
  int ref_v;
  float batt_v;
  uint8_t i;  
  uint16_t drn;   // 0..1023 adc reading
  uint16_t drn_temp;
  int16_t this_sin;
  int16_t this_cos;
  uint16_t angle;
  
  // gust update
  current_msec = millis();
  gust_period = current_msec - last_gust_msec;
  if (gust_period >= min_gust_period)
    {
      last_gust_msec = current_msec;
      this_wind_count = get_wind_count();
      this_gust_count = (uint16_t) this_wind_count - last_wind_count;
      last_wind_count = this_wind_count;
      
      // check the current wind direction for this period.
      drn = analogRead(WIND_DRN_ADC_CH);
      drn = drn + drn_cal;
      if (drn > 1023)
	{ 
	  drn = drn - 1024;
	}
      this_sin = my_sin(drn); // these are signed integers in range -10,000 to 10,000. Representing wind angle.
      this_cos = my_cos(drn);
      
      int32_t east_delta = (int32_t)  this_sin * this_gust_count;
      int32_t north_delta = (int32_t) this_cos * this_gust_count;
      
#ifdef DEBUG
#ifdef SERIAL_EN
      sprintf(buff, "%02x drn=%d sin=%d ed=%ld cos=%d nd=%ld", NODEID, drn, this_sin, east_delta, this_cos, north_delta);
      Serial.println(buff);
#endif
#endif
      total_easting +=  east_delta;
      total_northing += north_delta;
      
#ifdef DEBUG
#ifdef SERIAL_EN
      sprintf(buff, "%02x east=%ld north=%ld this_gust=%d", NODEID,  total_easting, total_northing, this_gust_count );
      Serial.println(buff);
#endif
#endif      
      
      if (this_gust_count > biggest_gust_count)
	{
	  biggest_gust_count = this_gust_count;
	  
	}
      gust_averages--;
      
      
      if (gust_averages == 0)
	{
	  gust_averages = GUST_AVERAGES;
	  wdt_reset();
	  
	  // *************** rainfall  **************
	  // rain count
	  float r = get_rain_count() / 5.0;  // 0.2mm per blip on int pin
	  dtostrf(r, 5, 1, buff2);
	  
	  sprintf(buff, "%02x rainfall=%smm", NODEID, buff2 );  
#ifdef RADIO
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
#endif
#ifdef SERIAL_EN
	  Serial.println(buff);
#endif
	  
	  // *************** wind direction **************
	  wdt_reset();
	  
	  angle = my_atan((float) total_northing, (float) total_easting); 
	  
	  float tn_f = total_northing;
	  float te_f = total_easting;
	  
	  float len_squared = tn_f * tn_f + te_f * te_f;
	  float    dist = ((float) sqrt ( (double) len_squared )) / 10000.0;
	  float    dist_kmh = dist *  ( 3620.0 /( (float) gust_period * GUST_AVERAGES)) ; //


	  
#ifdef DEBUG
#ifdef SERIAL_EN
	  //	  dtostrf(len_squared, 18, 1, buff2);
	  // sprintf(buff, "%02x len_sq=%s", NODEID,  buff2 );
	  // Serial.println(buff);
	  
	  dtostrf(dist, 5, 1, buff2);
	  dtostrf(dist_kmh, 5, 1, buff3);
	  sprintf(buff, "%02x dist=%s dist_kmh=%s", NODEID,  buff2, buff3 );
	  Serial.println(buff);
#endif
#endif      


	  
	  total_northing = 0;
	  total_easting = 0;
	  
	  dirn2str(angle, buff2);  // update drnstr
	  
	  // drn_temp = 36 * drn ;
	  dtostrf(dist_kmh, 5, 1, buff3);
	  
	  sprintf(buff, "%02x WindDrn=%s %d° spd=%s", NODEID, buff2, angle, buff3);
#ifdef RADIO
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  delay(50);
#endif
#ifdef SERIAL_EN
	  Serial.println(buff);
#endif
	  
	  wdt_reset();
	  
	  // **************** wind count *****************
	  
	  // gust count is max revs in 8.388 seconds
	  // where v[mph] = 2.25 * count / T
	  // or v[kph]    = 2.25 * 1.609 * count / T
	  // or v[kph]    = 3.62 * count / T
	  
	  biggest_gust = 3620.0 * (float) biggest_gust_count / (float) gust_period; 
	  
	  dtostrf(biggest_gust, 5, 1, buff2);
	  sprintf(buff, "%02x Wind gust=%s Kph dist=%lu", NODEID, buff2, this_wind_count);
	  
#ifdef RADIO
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  delay(50);
#endif
#ifdef SERIAL_EN
	  Serial.println(buff);
#endif
	  biggest_gust_count=0;
	  gust_averages = GUST_AVERAGES;
	  
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
	      
	      batt_v = 1.476 * batt_adc / ref_v; 
	      dtostrf(batt_v, 5, 2, buff2);
	      sprintf(buff, "%02x Batt=%sV", NODEID, buff2  );  
	      
#ifdef RADIO
	      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
#endif
#ifdef SERIAL_EN
	      Serial.println(buff);
#endif
	      batteryloop_times = BATTERYLOOP_TIMES;
	      
	    }
	  batteryloop_times--;
	  
	}
    }
  
  // speed up the poll rate
  for (i = 0; i < 10; i++)
    {
#ifdef RADIO
      if (radio.receiveDone())
	CheckForWirelessHEX(radio, flash, true);
#endif
      delay(200);
      
      wdt_reset();
    }
  // LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF); //put microcontroller to sleep to save battery life
}



// N NNE NE ENE   

// void dirn2str(uint16_t adcval, char *drnstr)
// {
//   uint8_t heading ;
//   heading = adcval >> 2; 
//   if (heading <  16 )
//     strcpy(drnstr,"N");
//   else if (heading < 32)
//     strcpy(drnstr, "NNE");
//   else if (heading < 48)
//     strcpy(drnstr, "NE");
//   else if (heading < 64)
//     strcpy(drnstr, "ENE");
//   else if (heading < 80)
//     strcpy(drnstr, "E");
//   else if (heading < 96)
//     strcpy(drnstr, "ESE");
//   else if (heading < 112)
//     strcpy(drnstr, "SE");
//   else if (heading < 128)
//     strcpy(drnstr, "SSE");
//   else if (heading < 144)
//     strcpy(drnstr, "S");
//   else if (heading < 160)
//     strcpy(drnstr, "SSW");
//   else if (heading < 176)
//     strcpy(drnstr, "SW");
//   else if (heading < 192)
//     strcpy(drnstr, "WSW");
//   else if (heading < 208)
//     strcpy(drnstr, "W");
//   else if (heading < 224)
//     strcpy(drnstr, "WNW");
//   else if (heading < 240)
//     strcpy(drnstr, "NW");
//   else 
//     strcpy(drnstr, "NNW");
// }


// try again with nested compares for better runtime

void dirn2str(uint16_t heading, char *drnstr)
{
  if (heading < 170)
    {
      if (heading < 80)
	{
	  if (heading < 34)
	    { // 0 .. 45
	      if (heading <  12 )
		strcpy(drnstr,"N");
	      else
		strcpy(drnstr, "NNE");
	    }
	  else
	    { // 45 .. 89
	      if (heading < 57)
		strcpy(drnstr, "NE");
	      else
		strcpy(drnstr, "ENE");
	    }
	}
      else
	{ // 90 .. 180
	  if (heading < 125)
	    {
	      if (heading < 102)
		strcpy(drnstr, "E");
	      else
		strcpy(drnstr, "ESE");
	    }
	  else
	    {
	      if (heading < 137)
		strcpy(drnstr, "SE");
	      else 
		strcpy(drnstr, "SSE");
	    }
	}
    } // 180..360
  else
    {
      if (heading < 260)
	{ // 180 .. 270
	  if (heading < 215)
	    { // 180 .. 225
	      if (heading < 192)
		strcpy(drnstr, "S");
	      else
		strcpy(drnstr, "SSW");
	    }
	  else
	    { // 226 .. 270
	      if (heading < 237)
		strcpy(drnstr, "SW");
	      else 
		strcpy(drnstr, "WSW");
	    }
	}
      else
	{ // 270.. 359
	  if (heading < 305)
	    { // 270 .. 314
	      if (heading < 282)
		strcpy(drnstr, "W");
	      else 
		strcpy(drnstr, "WNW");
	    }
	  else
	    { // 315 .. 359
	      if (heading < 327)
		strcpy(drnstr, "NW");
	      else if (heading < 350)
		strcpy(drnstr, "NNW");
	      else 
		strcpy(drnstr, "N");
	    }
	}
    }
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

#ifdef DEBUG
unsigned long  wind_count;
#endif

unsigned long  get_wind_count (void)
{
#ifndef DEBUG
  unsigned long  wind_count;
#endif
  
#ifdef DEBUG
  wind_count += 1000;
  return wind_count;
#else
  cli();
  wind_count = wind_raw;
  sei();
  return (wind_count);
#endif
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

 
// these routines are speed optimized and run at ~1/2 percent accuracy
// input is 0 .. 1023 representing adc reading, north is 0 (or 1024) east is 256.
// they return a signed integer in the range +10,000 to -10,000   ie sin*10k
int16_t  my_sin (uint16_t in)
{
  if (in > 512) {
    if (in < 768)               // manage the 4 quadrants
      {   // 513 ..  .. 767
	return (0 - sin_q1(in - 512));
      }
    else
      {
	// 768 .. 1023
	return (0 - sin_q1(1024 - in));
      }
  }
  else
    {
      // <= 512
      if (in <= 256)
	{   // 0 .. 256
	  return sin_q1(in);
	}
      else
	{
	  // 257 .. 512
	  return sin_q1(512 - in);
	}
    }
}
    

int16_t my_cos (uint16_t in)
{
    if (in > 512) {                // manage the 4 quadrants
	if (in < 768)
	  {   // 513 ..  .. 767
	    return (0 - sin_q1(768 - in));
	}
	else
	{
	  // 768 .. 1023
	    return sin_q1(in - 768);
	}
    }
    else
    {
      // <= 512
	if (in <= 256)
	  {   // 0 .. 256
	    return sin_q1(256 - in);
	}
	else
	{
	  // 257 .. 512
	  return (0 - sin_q1(in - 256));
	}
    }
}
    

    
uint16_t  sin_q1 (uint16_t in)
{
  // single quadrant sine calculation
  // returns sin*10,000  for angles between 0 and 256, the adc representation of 0 .. 90 degrees.
  // use 8 points lookup table and linear interpolation, which yields about 1/2 percent accuracy.
  uint8_t in_lookup = in >> interp_shift; // spacing 32 between lookup points
  uint8_t in_delta = in & interp_and;  // for calculating slope near lookup points
  
  return ((uint16_t)sin_points[in_lookup] + (uint16_t) (sin_slope[in_lookup] * in_delta));
} 

int16_t  my_atan (float dy, float dx)
{
  if (dx >= 0.0)
    {
      if (dy == 0.0)
	{
	  return 90;
	}
      else if (dy > 0.0)
	{ // dx > 0  dy > 0
	  return atan_q1(dx / dy);
	}
      else
	{ // dx > 0  dy < 0
	  return (180 - atan_q1(dx / (0.0 - dy)));
	}
    }
  else
    {
      if (dy == 0.0)
	{
	  return 270;
	}
      else if (dy > 0.0)
	{ // dx < 0  dy > 0
	  return (360.0 - atan_q1((0 -dx) / dy));
	}
      else
	{ // dx < 0  dy < 0
	  return (180 + atan_q1(dx / dy));
	}
    }

}



uint8_t atan_q1 (float in)
// single quadrant arc tan calc
// returns theta in degrees.
{
  if (in > 115.0)
    {
      return 90;
    }
  else if (in < 4.20)
    {
      if (in < 1.0)
	{
	  if ( in < 0.5)
	    { // 0 .. 0.5   0 .. 27 degrees
	      return (uint8_t) (in * 54.0);
	    }
	  else
	    { // 0.5 .. 1.0   28 .. 45 degrees
	      return (uint8_t) (in * 35.0 + 10.0);
	    }
	}
      else
	{
	  if ( in < 2.0)
	    { // 1.0 .. 2.0    46 .. 64 degrees
	      return (uint8_t) (in * 18.0 + 28.0);
	    }
	  else
	    { // 2.00 .. 4.1   67 .. 77 degrees
	      return (uint8_t) (in * 5.0 + 56.0);
	    }
	}
    }
  else
    {
      if (in < 11.0)
	{
	  if ( in < 6.5)
	    { // 4.2 .. 6.5   76 .. 81 degrees
	      return (uint8_t) (in * 2.5 + 66.0);
	    }
	  else
	    { // 6.5 .. 11.0   81 .. 84 degrees
	      return (uint8_t) (in  + 74.0);
	    }
	}
      else
	{
	  if ( in < 20.0)
	    { // 12.0 .. 19.0    85 .. 86 degrees
	      return (uint8_t) (in * 0.2 + 83.0);
	    }
	  else
	    { // 20.00 .. 60   87 .. 88 degrees
	      return (uint8_t) (in * 0.05 + 86.0);
	    }
	}
    }
}

	    
//  99146652000000.0
//  991466
//  sqrt = 995.72
// remove 10k squared = 10^8
