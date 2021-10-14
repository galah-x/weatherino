//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'weatherino   rainfall + wind for moteino Time-stamp: "2021-10-14 11:46:09 john"';


#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)
#include <LowPower.h>      //get library from: https://github.com/lowpowerlab/lowpower
                           //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <avr/wdt.h>       // watchdog


// define this to change network address and loop poll time for testing.
// #define DEBUG

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#ifdef DEBUG 
#define NODEID        0x23   //don't screw witch grafana when testing
#else 
#define NODEID        3   //unique for each node on same network
#endif 

#define GATEWAYID     1  //node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!. Not optional!
// #define USE_ENCRYP
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define SENDLOOPS    80 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send


//*********************************************************************************************

// IOs

#define LED             9 // Moteinos have a LED on D9
#define WIND_DRN_ADC_CH 7
#define BATT_ADC_CH     6
#define RainInt         6
#define WindInt         7
//

// calibration coefficient for wind drn.    ADC returns 0 .. 1023

// Windvane.
// Mount the windvane on the mast so it points in fixed direction, and measure that. 
// Use a magnetic compass from in front of the vane indicated it was at 200 degrees
// subtracting 180 implies it was pointed at 20 degrees magnetic
// local declination at 31.45 degrees S 151.62 E elevation 950 m is according to 
// http://www.ga.gov.au/oracle/geomag/agrfform.jsp
// Requested: Latitude -31o 27' 00", Longitude 151o 37' 00", Elevation .95 km, Date 2015/01/1 
// Calculated: Latitude -31.4500o, Longitude +151.6167o, Elevation 0.95 km, Epoch 2015.0000
// Magnetic Field Components
// D = 11.807 deg

// and from wikipedia Declination Magnetic
// D, the magnetic declination (sometimes called the magnetic variation), is the angle between the horizontal
// component of the magnetic field and true north. It is positive when the compass points east of true north,
// and negative when the compass points west of true north.
//
// So compass 20 degrees magnetic is 20-11.8 = 8 degrees east of north 
// so given the adc range is 1024 and that represents 360 degrees, thats an adc count of 12.2 * 1024/360 = 35  
// the system reported [181120192600] 03 WindDrn=S 180°
// or an adc count of 180/360 * 1024 = 512
// so I need a cal of 35 - 512 = -477  or 1024 - 477 = 547

//*********************** these #defines control various options in the code flow ********************/

// should the serial port be used for logging + debug?   I tend to use it for general rough debug, prior to final polish and checks
// #define SERIAL_EN
#define SERIAL_BAUD   115200

// should the radio be used to log with?   Yes, normally. Possibly not for some gross checks.
#define RADIO

// am I logging from the tipping bucket rain gauge? 
#define RAIN

// am I logging from the wind vane direction potentiometer and the wind rotation interrupt anemometer? 
#define WIND

// am I logging battery voltage. Since I'm generally using solar + battery, its generally good to keep an eye on this.
#define BATTERY



// Turn off the calibration term on the wind vane when DEBUG is on. ie do a one of cal calculation, not an iterative mess.  
#ifdef DEBUG
#define drn_cal 0
#else
#define drn_cal 547
#endif

//  some variables

byte sendLoops=0;
#define BUFLEN 60
char buff[BUFLEN]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[20];    // for float conversion
char buff3[10];    // for float conversion

#ifdef RADIO
RFM69 radio;
#endif

uint16_t gust_averages;
uint8_t batteryloop_times;

// this speeds up polls and lgging when testing. 
#ifdef DEBUG
const uint16_t GUST_AVERAGES=2;
const uint8_t BATTERYLOOP_TIMES=5;
#else
const uint16_t GUST_AVERAGES=60; // 8 m / 8.4s ~= 60 
const uint8_t BATTERYLOOP_TIMES=45;
#endif

// These are the constant used to calibrate the reported battery voltage.  
// I've changed between 12V lead acid and 1S LiOn several times. Mainly as I get the idle current under control so
// wind and rain interrupts are serviced reasonably cheaply. See the README for the ugly histroy here. 
//const float Kv = (12.22 / 11.82 ) *  5.70 * 3.3 / 1024; // yields 11.82 from 12.22
//
//                  fudge factor to fine tune      499K series +     3.3v from      10 bit  
//                  based on real measurement      1M shunt res      moteino LDO    ADC
//                  for resistor and LDO tol.      ADC input atten. 
const float Kv =    ( 4.657 / 4.647 )          *   1.499             * 3.3        / 1024.0;


// due to real damage from a few lightning strikes to an earlier iteration, I'm separating
// what used to be 'weatherino' into 2 parts.
// Both report via radio links, I've replaced far too many RS485 drivers.
// This part (new weatherino) does wind speed, direction, and rainfall. Its physically up high,
// just under the barn roof, close to the rain and wind sensors, but under cover. Gets hot in Summer.
// I want short wires due to high field strength from ESD.
// Then there is a second box. weatherino_temperature, monitoring temperature, humidity, air pressure.
// Its in a Stevenson screen box outside, on the southern side of the barn 10 or 20 metres away. 1.2m above the ground.

// I have learned over the past few years that powering weatherino_temperature is trivial as its usually asleep in very
// low power mode. A ~300maH LiPo and a 5x10cm solar cell glued to the eastern side wall of the stevenson screen works
// well.   It must idle at a few uA, then every ~10 minutes wakes the radio for a few dozen ms at a few dozen mA.

// Weatherino by comparison was sucking ~25mA @ 4V in to te moteino LDO due to a mix of radio receiver on
// (I tried to permit OTA updates) and leaving the core running so wind anemometer adn rain tipping bucket interrupts
// were processed.interrupts.
// That was too much for the little 6V solar cell I used. So I put in a bigger 5W 12V solar panel, a 12V 4.5AH lead
// acid Gel cell battery,  an Ebay 12V charge controller and a small ~1A 12V to 4V switcher module.
// Didn't bother to check the standing current of the charger and the switcher. Together there were over 20mA, which wore
// out the Pb battery in about a year. A few consecutive heavily overcast days deep discharged the battery too often.
// As did possums climbing on the wiring and unplugging the panel.
// Then the battery charger would load shed the weatherino. Ladder time.
//
// So rework the code again to find a better sleep mode, and ensure the radio is sleeping except when in use. All down to 4mA
// now which works with a 1S 18650 3000mA LiPo, a 6V panel and a TP4056 charge controller. (still need to add undervolt protection)

// more variables

uint16_t rain_raw;
uint32_t wind_raw;

uint32_t  current_msec;
uint32_t  last_gust_msec;

uint16_t  gust_period;

// about 4 times per day is 6 hrs or 360 minutes 
// with 8 minute inner loop update

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

// points for my sin and atan interpolation based table lookup conversions. trade off accuracy for speed. 
const uint8_t interp_points = 8;
const uint8_t interp_shift = 5;
const uint8_t interp_and = 31;
// sin * 10k. used for first quadrant only.
const uint16_t sin_points[interp_points+1] = { 0, 1951, 3827, 5556, 7071, 8315, 9239, 9808, 10000};
const uint8_t sin_slope[interp_points] = {61, 59, 54, 47, 39, 29, 18, 6};


// classic arduino setup routine

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
  
  sprintf(buff, "%02x weatherino 20211013", NODEID );  
#ifdef SERIAL_EN
  Serial.println(buff);
  Serial.flush();
#endif

#ifdef RADIO    
  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
  delay(500);
#endif
  
  analogReference(DEFAULT);
  
  gust_averages = GUST_AVERAGES;
  batteryloop_times = 0;
    
#ifdef RADIO    
  radio.sleep();
#endif
  
#ifdef RAIN
   pinMode(RainInt, INPUT);
   //  pinMode(RainInt, INPUT_PULLUP);  Input pullup is too big for the protection series R on weatherino
  // enable interrupt for pins
  pciSetup(RainInt);
  rain_raw = 0;
#endif
  
#ifdef WIND
  pinMode(WindInt, INPUT);
  // pinMode(WindInt, INPUT_PULLUP);   Input pullup is too big for the protection series R on weatherino
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
      Serial.flush();
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
	  // originally
	  //	  float r = get_rain_count() / 5.0;  // 0.2mm per blip on int pin
	  // lets rewrite as this, both to remove the division and to improve readability
	  //	  float r = get_rain_count() * 0.20;  // 0.2mm per blip on int pin according to Davis tipping bucket spec
 	  // now lets take into account the cal I did a while back. Rain collecter is getting old. (20 years now?)
	  float r = get_rain_count() * 0.25;  // 0.25mm per blip measured
	  dtostrf(r, 5, 1, buff2);
	  sprintf(buff, "%02x rainfall=%smm", NODEID, buff2 );  

#ifdef RADIO
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  for (i=0; i<10 ; i++)
	    {
	      // this delay is to allow relay to send data to computer
	      LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF); 
	    }

#endif
#ifdef SERIAL_EN
	  Serial.println(buff);
	  Serial.flush();
#endif
	  
	  // *************** wind direction **************
	  wdt_reset();
	  
	  angle = my_atan((float) total_northing, (float) total_easting); 
	  
	  float tn_f = total_northing;
	  float te_f = total_easting;
	  
	  float len_squared = tn_f * tn_f + te_f * te_f;
	  float dist = ((float) sqrt ( (double) len_squared )) / 10000.0;
	  float dist_kmh = dist *  ( 3620.0 /( (float) gust_period * GUST_AVERAGES)) ; //


#ifdef DEBUG
#ifdef SERIAL_EN
	  
	  dtostrf(dist, 5, 1, buff2);
	  dtostrf(dist_kmh, 5, 1, buff3);
	  sprintf(buff, "%02x dist=%s dist_kmh=%s", NODEID,  buff2, buff3 );
	  Serial.println(buff);
	  Serial.flush();
#endif
#endif      

	  total_northing = 0;
	  total_easting = 0;

	  // turn the heading into a 1..3 letter heading, ie NW, for the log
	  dirn2str(angle, buff2);
	  
	  // no float printf in basic arduino library. And 328P is just a tiddly little mem+ram cpu. 
	  dtostrf(dist_kmh, 5, 1, buff3);
	  
	  sprintf(buff, "%02x WindDrn=%s %d° spd=%s", NODEID, buff2, angle, buff3);
#ifdef RADIO
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  for (i=0 ; i<10 ; i++)
	    {
	      // wait for Relay to forward the message
	      LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
	    }
	  

#endif
#ifdef SERIAL_EN
	  Serial.println(buff);
	  Serial.flush();
#endif
	  
	  wdt_reset();
	  
	  // **************** wind count *****************
	  
	  // gust count is max revs in 8.388 seconds
	  // where v[mph] = 2.25 * count / T
	  // or v[kph]    = 2.25 * 1.609 * count / T
	  // or v[kph]    = 3.62 * count / T (in milliseconds) 
	  
	  biggest_gust = 3620.0 * (float) biggest_gust_count / (float) gust_period; 
	  
	  dtostrf(biggest_gust, 5, 1, buff2);
	  sprintf(buff, "%02x Wind gust=%s Kph dist=%lu", NODEID, buff2, this_wind_count);
	  
#ifdef RADIO
	  radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	  radio.sleep();
	  for (i=0; i<10; i++)
	    {
	      LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
	    }

#endif
#ifdef SERIAL_EN
	  Serial.println(buff);
	  Serial.flush();
#endif
	  biggest_gust_count=0;
	  gust_averages = GUST_AVERAGES;
	  
	  if (batteryloop_times == 0)
	    {
	      // average the ADC for cleaner measurement
	      batt_adc =  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc +=  analogRead(BATT_ADC_CH);
	      batt_adc = batt_adc >> 3;

	      batt_v = Kv * batt_adc; 
	      dtostrf(batt_v, 5, 3, buff2);

#ifdef DEBUG
#ifdef SERIAL_EN
	      sprintf(buff, "%02x Batt=%d", NODEID, batt_adc  );  
	      Serial.println(buff);
	      Serial.flush();
#endif
#endif
	      
#ifdef RADIO
	      sprintf(buff, "%02x Batt=%sV", NODEID, buff2  );
	      radio.sendWithRetry(GATEWAYID, buff, strlen(buff));
	      for (i=0; i<10; i++)
		{ // allow Relay to forward the message
		  LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
		}
	      radio.sleep();

#endif
#ifdef SERIAL_EN
	      Serial.println(buff);
	      Serial.flush();
#endif
	      batteryloop_times = BATTERYLOOP_TIMES;
	      
	    }
	  batteryloop_times--;
	}
    }
  
  // Timer0 generates millis();   I need millis() to work as its used to set the basic period.
  // Most low power modes don't run T0.   I also need pin change interrupts to be active. 
  // note radio in sleep gets current from ~25mA to ~9mA.   (radio just receiving is ~16mA.)
  // LowPower.idle goes from ~9mA (ie delay())  to ~4mA.
  // minimize current for the battery's sake.
  LowPower.idle(SLEEP_15MS, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
  wdt_reset();
}

//****************************** SUBROUTINES *****************************/

// dirn2str: turn a compass 0 .. 360 in ° degrees)  heading to a  1 to 3 letter acronym 
// eg N NNE NE ENE E ... S ... W  ... N  
// where 0° (or 360°) is N    and  90° is E.
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
    } 
  else
    {              // 180..360
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


// This is the ISR for the pin change interrupt. Both wind and rain are in the D0..D7 range so its a
// single int vector/routine to do either/both

// RainInt - debounced. Its a magnetic reed switch. Debounced 50ms.
// The Davis tipping bucket has a magnet at the centre of each tip, presumably the max speed point?
// unsure why they don't just put the relay on one side. Maybe that affects the amoutn of water in every second bucket? 
// It would allow much slower sensing.    Whatever, if I can do a wind anemometer I can do rain. 
// Guess there are power considerations too, this way allows more wetting current.

// The wind int seems not to need a debounce. Its a 1ms pulse, seems cleanly generated.
// Both use the negedge only here.

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
    wind_raw++;
  
  last_wind_lvl = wind_lvl;
  
  // only use negedge for rain, 
  if ((last_rain_lvl == HIGH) && (rain_lvl == LOW))
    // and debounce it 50ms.  Meaning ignore any negedge transitions for 50ms after a counted transition.  
      if (abs(interrupt_time - last_used_rain_interrupt_time) > 50)
	// apply 50ms debounce on rain transition.
	// the abs() is to cope with wrap around of millis. Doesn't happen often, but I'd lose a lot when it does finally wrap. 
	{
	  last_used_rain_interrupt_time = interrupt_time;
	  rain_raw++;
	}
  last_rain_lvl = rain_lvl;
}

// get the current wind total interrupt count in an interrupt safe manner

unsigned long  get_wind_count (void)
{
  unsigned long  wind_count;
  cli();
  wind_count = wind_raw;
  sei();
  return (wind_count);
}

// get the current rain total interrupt count in an interrupt safe manner
unsigned int get_rain_count (void)
{
  unsigned int rain_count;
  cli();
  rain_count = rain_raw;
  sei();
  return (rain_count);
}

// Hmm. get_rain_count is a uint16_t
// max 65k.
// at 0.2mm per tip thats 13,100 mm
// at 1200mm / year thats 10.9 years to overflow.  And a drought or 2 will stretch that.
// I don't think my batteries and solar charge system are ever going to let that be a problem.
// At least not often in my lifetime!          So 16 bits is sufficient for me here.



// don't think I use this any more. The 328P adc is generally used with range from 0 to supply (3V3 here)
// It does have an internal 1v1 ref that allows you to lose sensitivity to the 3V3 rail, which can
// help when I'm using the LDO 3v3 reference after the battery to measure the battery. Gets a bit broken when
// battery voltage is less than 3v3.  (+ LDO dropout, but thats not much at low current) corrupting the reference. 
// I'm not currently using that. Assume low voltage protect on the battery charge controller renders this moot.

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
// see sine.pl and atan.pl in working directory for accuracy justification/verification

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

// ARC TAN

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

// single quadrant ARC TAN.

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
