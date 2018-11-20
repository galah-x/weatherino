//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'weatherino temp humidity pressure rainfall wind for moteino Time-stamp: "2018-11-20 19:41:06 john"';

// $ grabserial -b 19200 -d /dev/ttyUSB1 | ts [%y%m%d%H%M%S]



#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //included with Arduino IDE (www.arduino.cc)
#include <LowPower.h> //get library from: https://github.com/lowpowerlab/lowpower
                      //writeup here: http://www.rocketscream.com/blog/2011/07/04/lightweight-low-power-arduino-library/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>






//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        3   //unique for each node on same network
#define GATEWAYID     1  //node Id of the receiver we are sending data to
#define NETWORKID     100  //the same on all nodes that talk to each other including this node and the gateway
#define FREQUENCY     RF69_915MHZ //others: RF69_433MHZ, RF69_868MHZ (this must match the RFM69 freq you have on your Moteino)
// #define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
// #define USE_ENCRYP
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define SENDLOOPS    80 //default:80 //if no message was sent for this many sleep loops/cycles, then force a send


//*********************************************************************************************
#define SLEEP_FASTEST SLEEP_15MS
#define SLEEP_FAST SLEEP_250MS
#define SLEEP_SEC SLEEP_1S
#define SLEEP_LONG SLEEP_2S
#define SLEEP_LONGER SLEEP_4S
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
#define WIND_DRN_CH 7
#define RainInt 7
#define WindInt 8
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

// #define BATT_MONITOR  A7  // Sense VBAT_COND signal (when powered externally should read ~3.25v/3.3v (1000-1023), when external power is cutoff it should start reading around 2.85v/3.3v * 1023 ~= 883 (ratio given by 10k+4.7K divider from VBAT_COND = 1.47 multiplier)
// #define BATT_READ_LOOPS  SENDLOOPS  // read and report battery voltage every this many sleep cycles (ex 30cycles * 8sec sleep = 240sec/4min). For 450 cycles you would get ~1 hour intervals between readings
// #define BATT_FORMULA(reading) reading * 0.00322 * 1.475  // >>> fine tune this parameter to match your voltage when fully charged
// #define BATT_LOW      3.55

byte sendLen;
byte sendLoops=0;
//byte distReadLoops=0;
//byte battReadLoops=0;
//float distance=0;
//float prevDistance=0;
//float batteryVolts = 5;
#define BUFLEN 50
char buff[BUFLEN]; //this is just an empty string used as a buffer to place the payload for the radio
char buff2[10];    // for float conversion

// char* BATstr="BAT:5.00v"; //longest battery voltage reading message = 9chars
// char* DISTstr="99999.99cm"; //longest distance reading message = 5chars
// float readDistance(byte samples=1);  //take 1 samples by default
RFM69 radio;

int outerloop_times;


Adafruit_BMP280 bme; // I2C
Adafruit_SHT31 sht31 = Adafruit_SHT31();

unsigned int rain_raw;
unsigned long wind_raw;

long unsigned current_msec;
long unsigned last_gust_msec;
unsigned int gust_period;
unsigned int used_gust_period;
float biggest_gust;
float this_gust;
unsigned int this_gust_count;
long unsigned this_wind_count;
long unsigned last_wind_count;
  


// #define SERIAL_EN


void setup() {
  // noInterrupts();
  // CLKPR = _BV(CLKPCE);  // enable change of the clock prescaler
  // CLKPR = _BV(CLKPS0);  // divide frequency by 2
  // interrupts();
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
  
  sprintf(buff, "%02x weatherino", NODEID );  
  sendLen = strlen(buff);
  //  radio.sendWithRetry(GATEWAYID, buff, sendLen);
  radio.sendWithRetry(GATEWAYID, buff, sendLen);
  
  outerloop_times = 0;
  
  if (!bme.begin())
    {  
      sprintf(buff, "%02x Could not find a valid BMP280 sensor", NODEID);  
      sendLen = strlen(buff);
      radio.sendWithRetry(GATEWAYID, buff, sendLen);
    }
  
  if (!sht31.begin(0x44))
    {   // Set to 0x45 for alternate i2c addr
      sprintf(buff, "%02x Could not find a valid SHT31 sensor", NODEID );  
      sendLen = strlen(buff);
      radio.sendWithRetry(GATEWAYID, buff, sendLen);
    }
    
    
  //  radio.sleep();
  
  //  pinMode(TRIG, OUTPUT);
  //  pinMode(FLOATSW, INPUT);
  //  pinMode(SENSOR_EN, OUTPUT);
    //  digitalWrite(SENSOR_EN, LOW);
  
  pinMode(RainInt, INPUT);
  pinMode(WindInt, INPUT);
  
  // enable interrupt for pins
  pciSetup(RainInt);
  pciSetup(WindInt);

  rain_raw = 0;
  wind_raw = 0;

  last_gust_msec = millis();
  biggest_gust = 0;;
  last_wind_count = 0;
}

/************************** MAIN ***************/


// #define BMP_SCK 13
// #define BMP_MISO 12
// #define BMP_MOSI 11 
// #define BMP_CS 10

//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
  
#define min_gust_period 8400

  
void loop() {
  

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
  
  if (outerloop_times == 0) {
  
    //    Serial.print("Approx altitude = ");
    //    Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase


    // *************** bmp280 temp and pressure **************
    
    // no %f in sprintf
    dtostrf(bme.readTemperature(), 5, 2, buff2);

    sprintf(buff, "%02x bmp280Temperature=%s°C", NODEID, buff2  );  
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);

    // no %f in sprintf
    dtostrf(bme.readPressure(), 8, 2, buff2);

    snprintf(buff, BUFLEN, "%02x Pressure=%s Pa", NODEID, buff2   );  
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);

    // *************** sht31 temp and humidity **************

    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    dtostrf(t, 5, 2, buff2);
    sprintf(buff, "%02x sht31Temperature=%s°C", NODEID, buff2  );  
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);
        
    dtostrf(h, 5, 2, buff2);
    sprintf(buff, "%02x sht31Humidity=%s%%", NODEID, buff2  );  
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);

    // *************** rainfall  **************
    // rain count
    float r = get_rain_count() / 5.0;
    dtostrf(r, 5, 1, buff2);
    
    sprintf(buff, "%02x rainfall=%smm", NODEID, buff2 );  
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);
    
    
    // *************** wind direction **************

    int drn;
    unsigned int drn_temp;
    drn = analogRead(WIND_DRN_CH);
    drn = drn + drn_cal
;
    if (drn > 1023)
      { 
	drn = drn - 1023;
      }

    dirn2str(drn, buff2);  // update drnstr
    
    drn_temp = 36 * drn ;
    
    sprintf(buff, "%02x WindDrn=%s %u°", NODEID, buff2, drn_temp / 103);
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);

    // **************** wind count *****************
    dtostrf(biggest_gust, 5, 1, buff2);
   
    sprintf(buff, "%02x Wind gust=%s Kph dist=%lu", NODEID, buff2, get_wind_count());
    sendLen = strlen(buff);
    radio.sendWithRetry(GATEWAYID, buff, sendLen);
    // sprintf(buff, "%02x gust_period=%u gust_count=%u", NODEID, used_gust_period, this_gust_count);
    // sendLen = strlen(buff);
    // radio.sendWithRetry(GATEWAYID, buff, sendLen);
    biggest_gust=0;
    
//    sprintf(buff, "%02x millis=%lu ", NODEID, millis());
//    sendLen = strlen(buff);
//    radio.sendWithRetry(GATEWAYID, buff, sendLen);

    
    
    radio.sleep();
    //    delay(2000);
    outerloop_times = SLEEP_MULTIPLIER;
  }
  outerloop_times--;
  delay(2000);
  // LowPower.powerDown(sleepTime, ADC_OFF, BOD_OFF); //put microcontroller to sleep to save battery life
}



//void Blink(byte pin)
//{
//  pinMode(pin, OUTPUT);
//  digitalWrite(pin, HIGH);
//  delay(2);
//  digitalWrite(pin, LOW);
//}
//

// rainfall




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
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

// Use one Routine to handle each group

// wind int, which seems not to need a debounce
ISR (PCINT0_vect) // handle pin change interrupt for D8 to D13 here
{
  static byte lastWindLvl = 0;
  
  // read the port, once.
  byte WindLvl = digitalRead(WindInt);
  
  // negedge only
  if ((lastWindLvl == HIGH) && (WindLvl == LOW))
    {
      wind_raw++;
    }  
  lastWindLvl = WindLvl;  // save for next time 
  
  digitalWrite(13,digitalRead(WindInt));
}


// RainInt - debounced
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  byte lvl = digitalRead(RainInt);

  // only use negedge
    if (lvl == LOW)
      {
	// debounce
	if ((interrupt_time - last_interrupt_time) > 100)
	  { // apply 50ms debounce on rain transition
	    
	    // accumulate
	    rain_raw++;
	  }
      }
    last_interrupt_time = interrupt_time;
    digitalWrite(13,lvl);
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


 
 
