#include <LibLacuna.h>

/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2019 Lacuna Space Ltd.
 *
 * Description: Automated satellite uplink Sketch.
 * 
 * Licensae: Revised BSD License, see LICENSE-LACUNA.TXT file included in the project
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * You can install these libraries via the Arduino library manager:
 *  
 *  http://librarymanager/All#Sodaq_LSM303AGR
 *  http://librarymanager/All#SparkFun_BME280
 *  http://librarymanager/All#SparkFun_Ublox
 *  http://librarymanager/All#SparkFun_ATECCX08a
 *  
 * Download SPG4 library and unzip in Arduino libraries folder:
 * 
 * https://github.com/LacunaSpace/Sgp4-Library/archive/v1.zip
 * 
 * Expermental code Sat node met Antenna switch Andries  18-03-2020
 */

#ifndef REGION
#define REGION R_EU868
#endif

#include <LibLacuna.h>
#include <SPI.h>
#include <time.h>
#include <EEPROM.h>
#include <RTC.h>
#include <Sgp4.h>
#include <SparkFunBME280.h>   // http://librarymanager/All#SparkFun_BME280
#include <Sodaq_LSM303AGR.h>  // http://librarymanager/All#Sodaq_LSM303AGR
#include <SparkFun_Ublox_Arduino_Library.h>   // http://librarymanager/All#SparkFun_Ublox

// Interval between satellite transmissions and number of messages
// Full pass is ca 10 min within range 6 min
// 
#define INTERVAL 10       // 5 seconden tussen elk bericht
#define NUMTX 40
//#define M6P              // If MP6 Ignore morning passes in period 3 .. 15 h

// Sleep time between periodic GPS checks (and TTN updates)
#define SLEEPMINUTES 30   // To match MJS values

// Minimum elevation for satellite
#define MINELEVATION 20       // Test also low orbit passages

// max. 250 seconds for GPS fix
#define FIXTIME 180

// Dont sync GPS within this interval before a scheduled transmission
#define GUARDTIME (FIXTIME+30)

// Only use for testing

// #define UPDATE // automatic TLE update
// #define FAKEPASS
// #define DEBUGSLEEP   //Comment out for production
#define OFFSET 1000 // additional time offset to avoir interference for multiple device

// Define this to get the settings for the four test nodes, or leave it undefined to fill in your own settings in the #else block below
#define LACUNA_TEST 1



#if LACUNA_TEST == 1
// Do not modify
static byte networkKey[] = {     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte appKey[] = {   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte deviceAddress[] = { 0x00, 0x00, 0x00, 0x00 };
#elif LACUNA_TEST == 2
// Do not modify
static byte networkKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static byte deviceAddress[] = { 0x00, 0x00, 0x00, 0x00 };


#else
///////////////////////////////////
// Fill in your own settings here
///////////////////////////////////
// Keys and device address are MSB
static byte networkKey[] = {
   // Replace with your network key
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static byte appKey[] = {
   // Replace with your application key
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// Replace with your device address
static byte deviceAddress[] = { 0x00, 0x00, 0x00, 0x00 };
#endif // LACUNA_TEST




  

// Firmware version to send. Should be incremented on release (i.e. when
// signficant changes happen, and/or a version is deployed onto
// production nodes). This value should correspond to a release tag.
// For untagged/experimental versions, use 255.
const uint8_t FIRMWARE_VERSION = 255;

// Set values experiment:
//#define DUT   1         // Antenna device under test Quadrifilar Helical
//#define DUT   2         // Assymetrische Kruisdipool ( Helicopter antenne )
//#define DUT   3         // Dibond Patchantenne V1 ( gefreesd door Mark )
//#define DUT   4         // 4 turn Helical Antenna, aimed  at max Altitude
//#define DUT   5         // Mikrotik LoRa Omni Antenna Kit 6/5 dBi ( reduced Terrestrial TX power )
//#define DUT   6         // Antenna device under test DYI Quadrifilar Helical Rev 0.9 ( with tuning fane )
//#define DUT   7         // Annular ring Patchantenne V1 ( nagefreesd door Mark )
//#define DUT   8         // Antenna device under test DYI Quadrifilar Helical Rev 0.9 make2 ( with tuning fane )
#define DUT   9         // Antenna device under test DYI Quadrifilar Helical Rev 0.9 make3 ( in PVCtube )
//#define DUT   10         // Annular ring Patchantenne V2 On Glass
#define TXPOWER  6      // Terrestrial TX power dBm
#define SATPOWER 21     // Satellite TX power dBm
#define BAUDRATE 9600   // define baudrate for debug
#define LORAPORT 1     // define the loRa portnumber
#define BATTERY true    // with battery voltage
    

float temperature;   // Global variable
float humidity;      // Global variable
uint16_t vcc = 0;    // Global variable vcc suplly voltage
#ifdef WITH_LUX
uint16_t lux = 0;
#endif
//int32_t lat24 = 0;   // Global variable lattitude converted to integer
//int32_t lng24 = 0;   // Global variable longitude converted to integer

// When sending extra data, use this many bits to specify the size
// (allows up to 32-bit values)
uint8_t const EXTRA_SIZE_BITS = 5;  // Some kind of runlength encoding, each data field is preceeded by a size field of 5 bits




//  AHL experimental specific setup 
float    azimuth = 1;             // Global azimuth result of Tracking function in degrees 
float    elevation = 2;           // Global azimuth result of Tracking function in degrees
float    distance = 3;            // Global azimuth result of Tracking function in km
uint8_t  txpower = TXPOWER;       // TxPower in dBm
uint8_t  antenna = 0 ;            // 0 is standard higher is Device under test
uint32_t framecounter = 0;        // variable copy of Lora Framecounter
uint32_t timestamp = 4;           // Initialise with example data 30 maart om 22:09:15
uint32_t unixtime = 0;
uint32_t tle_ddmmss = 0;
uint16_t tracked = 0;
uint32_t time_correction = 0;

// End MJS specfic setup


static lsLoraWANParams loraWANParams;   // Create loraWANParameter structure 
static lsLoraSatTxParams SattxParams;   // Create SattxParams structure
static lsLoraTxParams txParams;         // Create terrestrial txParams structure

static char payload[255];               // Create payload array of char structure
static int payloadLength;               // Create payloadLength structure

float gnss_lat;                         // Global node postition lattiude
float gnss_lon;                         // Global node postition longitude
float gnss_alt;                         // Global node postition altitude

uint32_t fixepoch;                      // 
uint8_t  TimeToFix;

uint32_t last_epoch;
uint32_t alarmepoch;
uint32_t txepoch = 0;
uint32_t txstop  = 0;                   // Set the endpoint of the TX window
uint8_t  txnumber = 0;                  // Number of messages to transmit
uint8_t  txinterval = 0;
uint8_t  nextsat = 01;
int32_t sleepseconds;
uint8_t wakeupreason = 0;               // flag for wakupe reason

// interval between GPS checks
uint16_t sleeptimesec = 60 * SLEEPMINUTES; // in seconds

// SGP4
Sgp4 LacunaSat;      // Create predictor instance
char satname[] = "LS2d";
uint32_t tle_epoch;
uint8_t  tle_age = 0xFF;


// BME280 sensor
BME280 bme280;       // Create sensor instance   

// Accelerometer
Sodaq_LSM303AGR lsm303(LSM303_WIRE);

// GPS
SFE_UBLOX_GPS myGPS;   // Create GPS instance

void setup() {

  // Initialise Relay GPIO control pin and switch off relay
  pinMode(PIN_SPI1_MOSI, OUTPUT);    // Set relay controlpin as output
  digitalWrite(PIN_SPI1_MOSI, LOW);  // Switch off relay
  
  delay(1000);  // wait 10 seconds before starting ( to enable programming )

//  Debug serial port 
//  Serial is the default USB serialport ( Crashes when device is pout into sleep
//  Serial sends debug output to pin PA2-TX  ( pin 8 of trhe LS200 developmentboard
//  You may set flag DEBUG-SLEEP to avoid prevent sleeping
//  or global change - replace Serial <-> Serial3 to switch debugchannel
  
  Serial.begin(BAUDRATE);               // Start serial verbose
 
  while (!Serial && millis() < 3000);   // Wait for the serialport to be ready

  
  Serial.println("Initializing");
  Serial.print("Firmware version: V");
  Serial.println(FIRMWARE_VERSION);
  Serial.print("Tx Interval : ");
  Serial.print(INTERVAL);
  Serial.print(" Satellite Transmissies : ");
  Serial.println(NUMTX);
  

  Serial.print("Configured Region: ");
#if REGION == R_EU868
  Serial.println("Europe 862-870 Mhz");
#elif REGION  == R_US915
  Serial.println("US 902-928 Mhz");
#elif REGION == R_AS923
  Serial.println("Asia 923 Mhz");
#elif REGION == R_IN865
  Serial.println("India 865-867 Mhz");
#else 
  Serial.println("Undefined");  
#endif

  analogReadResolution(12);             // Initialise ADC port?

  Wire.begin();
  LSM303_WIRE.begin();

  pinMode(LS_GPS_ENABLE, OUTPUT);
  digitalWrite(LS_GPS_ENABLE, HIGH);
  pinMode(LS_GPS_V_BCKP, OUTPUT);
  digitalWrite(LS_GPS_V_BCKP, HIGH);

  digitalWrite(LS_VERSION_ENABLE, LOW);

  pinMode(LS_INT_MAG, OUTPUT); // make INT_MAG LOW for low-power 
  digitalWrite(LS_INT_MAG, LOW);

  // LSM303
  lsm303.enableAccelerometer();

  // BME 280
  bme280.setI2CAddress(0x76);
  if (bme280.beginI2C(BME280_WIRE) == false)
  {
    Serial.println("The sensor did not respond. Please check wiring.");
  }

  delay(50);

  // GPS
  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
      Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
      //while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration();        //Save the current settings to flash and BBR
  
  pinMode(LS_LED_BLUE, OUTPUT);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(200);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(200);
  

  // SX1262 configuration for lacuna LS200 board
  lsSX126xConfig cfg;                   //Create SX126x configuration structure
  lsCreateDefaultSX126xConfig(&cfg);    //Initialise  SX126x  structure with values

  // Initialize SX1262
  int result = lsInitSX126x(&cfg);
  if(result) Serial.println(lsErrorToString(result));

  // LoRaWAN session parameters
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = LORAPORT;
  loraWANParams.rxEnable = true;
 
  // transmission parameters for Lacuna satellites
  lsCreateDefaultLoraSatTxParams(&SattxParams);

  // transmission parameters for terrestrial LoRa
  lsCreateDefaultLoraTxParams(&txParams);

  // modify LoRa parameters Satellite parameters
  SattxParams.power = SATPOWER;
  
  // modify LoRa parameters Terrestrial parameters
  txParams.power = TXPOWER;
  txParams.spreadingFactor = lsLoraSpreadingFactor_8;


#ifdef UPDATE
   // Update TLE
  get_new_tle(&txParams);
 #endif

  // Initialize satellite orbit data 
  LacunaSat.site(0,0,0);
  initialize_tle(1);   // start with initializing satellite 1



}



void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  
  last_epoch = RTC.getEpoch();

  if (wakeupreason == 0) {

    Serial.println("Periodic wakeup");

    byte fix = updategps();
    
      if (fix) {
          Serial.println("New position fix");
          
          // predict satellite passes
          uint32_t start;
          uint32_t start1;
          uint32_t duration;
          uint32_t duration1;
          initialize_tle(1);   // start with initializing satellite 1
          uint16_t maxelevation = predictpass(gnss_lat, gnss_lon, gnss_alt, &start1, &duration1);
          
          initialize_tle(2);   // try satellite 2
          uint16_t maxelevation2 = predictpass(gnss_lat, gnss_lon, gnss_alt, &start, &duration);
          nextsat=2;

          if (start1<start) {
            start=start1;
            duration=duration1;
            initialize_tle(1);   // Sat 1 is the first one to come
            nextsat=1;
            }
          
      
          if (maxelevation != 0) {
    
            Serial.print("  Next pass set, ");
            Serial.print("max. elevation: ");
            Serial.print(maxelevation);
            Serial.print(", start at ");
            Serial.print(start);
            Serial.print(", Sat N°");
            Serial.print(nextsat);
            Serial.print(" duration: ");
            Serial.println(duration);
            
    
            txinterval = INTERVAL + tle_age * 1;
            //  txepoch = start + (duration/2) - (txinterval*(NUMTX-1))/2;
            txepoch = start ;      // Start at the beginning of the overpass
            if ( duration > 300) {
              duration = 300 ;     // Limit the txwindow to 5 minutes to avoid loops
            }
            txnumber = uint8_t(duration/INTERVAL);
            // txstop = txepoch+duration;
            tracked = trackingdata(gnss_lat, gnss_lon, gnss_alt, txepoch , &azimuth, &elevation, &distance);  // Calculate epoch Sat position
    
            Serial.print("TX Epoch set to: ");
            Serial.print(txepoch);
            Serial.print(", interval: ");
            Serial.print(txinterval);
            Serial.println(" seconds");
    
          } else {
            Serial.println("No pass found?");
          }
          
          // Use the GPS fix time since startup combined with deviceAddress as a crude
          // random seed, to prevent nearby nodes from using the same hopping pattern at the same time.
          randomSeed(micros()+*((uint32_t*)deviceAddress));
        }

        for (uint8_t i=0; i < 3; ++i) {
          digitalWrite(LS_LED_BLUE, HIGH);
          delay(200);
          digitalWrite(LS_LED_BLUE, LOW);
          delay(200);
        }
        //digitalWrite(LS_LED_BLUE, HIGH);
        
//  Rotate antenna's on framecounter  
      framecounter = loraWANParams.framecounter ;  // Read LoraWANframecounter
      Serial.print("Current framcount = ");
      Serial.println(framecounter);
      antenna = 0;
      if ( (framecounter % 2) == 1 ){
        digitalWrite(PIN_SPI1_MOSI, HIGH);  // Switch on Relay
        delay(100) ;                        // Wait for stable contact
        Serial.println("Select external Antenna");
        antenna = DUT;
      }
      unixtime  = RTC.getEpoch();
      timestamp = UnixToDhhmmss(txepoch-unixtime);  // Display the time till next overpass
      
      generateMessage(0);         // It is no satellite message

      Serial.println("Sending terrestrial LoRa message");  
      int lora_result = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);
      Serial.print("Result: ");
      Serial.println(lsErrorToString(lora_result));


      // Update TLE if needed
      if (tle_age > 30) {
        get_new_tle(&txParams);
      }
      
      digitalWrite(PIN_SPI1_MOSI, LOW);  // Switch off Relay
      delay(100) ;                        // Wait for stable contact
      digitalWrite(LS_LED_BLUE, LOW);
  } else if (wakeupreason == 1) {

    Serial.println("satellite TX wakeup");

#ifdef OFFSET
    // Test additional delay to avoid inteference
    delay ( OFFSET);
#endif
 
    for (int numtx =0; numtx < txnumber; numtx++) {
      digitalWrite(LS_LED_BLUE, HIGH);       // Switch on LED
      delay(200);
      digitalWrite(LS_LED_BLUE, LOW);       // Switch on LED
      delay(200);
      
      unsigned long starttx = millis();      // get millis timer
      unsigned long endtx = ( starttx + INTERVAL*1000) ;

// retrieve fresh Trackingdata and timestamp   
      unixtime = RTC.getEpoch();
      tracked = trackingdata(gnss_lat, gnss_lon, gnss_alt, unixtime , &azimuth, &elevation, &distance);  // update trackingdata
      timestamp = unixtime;
      
      Serial.println("Transmit");
      
////  Rotate antenna's on framecounter  
//      framecounter = loraWANParams.framecounter ;  // Read LoraWANframecounter
//      Serial.print("Current framcount = ");
//      Serial.println(framecounter);
//      antenna = 0;
//      if ( (framecounter % 2) == 1 ){
//        digitalWrite(PIN_SPI1_MOSI, HIGH);  // Switch on Relay
//        delay(100) ;                        // Wait for stable contact
//        Serial.println("Select external Antenna");
//        antenna = DUT;
//      }
 /*
    //Use only External antanna for SAT transmission 
      digitalWrite(PIN_SPI1_MOSI, HIGH);  // Switch on Relay
      delay(100) ;                        // Wait for stable contact
      Serial.println("Select external Antenna");
      antenna = DUT;
 */         
      
      // Use the GPS fix time since startup combined with deviceAddress as a crude
          // random seed, to prevent nearby nodes from using the same hopping pattern at the same time.
          randomSeed(micros()+*((uint32_t*)deviceAddress));
      
      
      generateMessage(nextsat);
      Serial.println("Sending LoraSat message");
      int sat_result = lsSendLoraSatWAN(&loraWANParams, &SattxParams, (byte *)payload, payloadLength);
      Serial.print("Result: ");
      Serial.println(lsErrorToString(sat_result));
      digitalWrite(PIN_SPI1_MOSI, LOW);  // Switch off Relay
      digitalWrite(LS_LED_BLUE, LOW);
      
      while ( endtx > millis()) {
      delay(1); 
      }    // end of delay loop
     
    }      // end of for loop 
           // end of wakeup reason 1 
  } else {
    // this should not happen
    Serial.println("Unknown wakeup reason");
  }

  // Calculate sleep time
  
  if ( (txepoch != 0) && (last_epoch + sleeptimesec) > (txepoch - GUARDTIME) && (txepoch > RTC.getEpoch())) {

    Serial.println("Sleep till TX epoch");
    alarmepoch = txepoch;
    wakeupreason = 1;

  } else {

    Serial.println("Sleep till periodic update");
    alarmepoch = last_epoch + sleeptimesec;
    wakeupreason = 0;

  }

  sleepseconds = alarmepoch - RTC.getEpoch();

  Serial.println("-- SLEEP");
  Serial.print("  Sleep Time: ");
  Serial.println(sleepseconds);

#ifdef DEBUGSLEEP

  if (sleepseconds > 0 ) {
    delay(sleepseconds * 1000);
  }

#else

  if (alarmepoch > RTC.getEpoch()) {

    time_t t;
    struct tm tm;

    t = (time_t)alarmepoch;
    gmtime_r(&t, &tm);
    
    RTC.setAlarmTime(tm.tm_hour, tm.tm_min, tm.tm_sec);
    RTC.setAlarmDay(tm.tm_mday);
    
    RTC.enableAlarm(RTC.MATCH_HHMMSS);
    RTC.attachInterrupt(alarmMatch);

    // put LS200 board in deepsleep
    LS200_sleep();

    delay(100);
  }

#endif

  Serial.println("-- WAKE");
  
}   // End of Main Loop

void alarmMatch() { }

void LS200_sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, HIGH);
  
  bme280.setMode(MODE_SLEEP); 
  lsm303.disableMagnetometer();
  lsm303.disableAccelerometer();

  SPI.end();
  delay(10);
  STM32.stop();

  // Sleep...
  
  SPI.begin();
  digitalWrite(LS_GPS_ENABLE, HIGH);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, HIGH);
  bme280.setMode(MODE_FORCED);
  lsm303.enableAccelerometer();
}


// This is a Merge of the MJS generate message and the Lacuna message

void generateMessage(uint8_t satellite) {
  
 int16_t temp = (int16_t)(100*bme280.readTempC());
  uint8_t humidity = (int8_t)(bme280.readFloatHumidity());
  uint16_t pressure = (uint16_t)(bme280.readFloatPressure()/10);

  uint16_t voltage_adc = (uint16_t)analogRead(LS_BATVOLT_PIN);
  uint16_t voltage = (uint16_t)((LS_ADC_AREF / 4.096) * (LS_BATVOLT_R1 + LS_BATVOLT_R2) / LS_BATVOLT_R2 * (float)voltage_adc);

  int8_t acc_x = (int8_t)(100*lsm303.getX());
  int8_t acc_y = (int8_t)(100*lsm303.getY());
  int8_t acc_z = (int8_t)(100*lsm303.getZ());

  // STM32 vbat voltage
  // uint16_t voltage = (uint16_t)(1000*STM32.getVBAT());

 Serial.println("Read sensors");
  Serial.print("  Voltage: ");
  Serial.println(voltage/1000.0f);
  Serial.print("  Temp: ");
  Serial.println(temp/100.0f);
  Serial.print("  Humidity: ");
  Serial.println(humidity);
  Serial.print("  Pressure: ");
  Serial.println(pressure);

  Serial.print("  x = ");
  Serial.print(acc_x);
  Serial.print(", y = ");
  Serial.print(acc_y);
  Serial.print(", z = ");
  Serial.println(acc_z);
 
  uint32_t LatitudeBinary = ((gnss_lat + 90) / 180) * 16777215;
  uint32_t LongitudeBinary = ((gnss_lon + 180) / 360) * 16777215;
  int16_t  altitudeGps = gnss_alt; 
  
  uint32_t epoch = RTC.getEpoch();

  uint32_t fixage = (epoch - fixepoch)/60; // in minutes

  payload[0] = satellite;
  payload[1] = (temp >> 8) & 0xff;
  payload[2] = temp & 0xff;
  payload[3] = (voltage_adc >> 8) & 0xff;
  payload[4] = voltage_adc & 0xff;
  payload[5] = humidity; 
  payload[6] = (pressure >> 8) & 0xff;
  payload[7] = pressure & 0xff;
  payload[8] = acc_x;
  payload[9] = acc_y;
  payload[10] = acc_z;
  payload[11] = ( LatitudeBinary >> 16 ) & 0xFF;
  payload[12] = ( LatitudeBinary >> 8 ) & 0xFF;
  payload[13] = LatitudeBinary & 0xFF;
  payload[14] = ( LongitudeBinary >> 16 ) & 0xFF;
  payload[15] = ( LongitudeBinary >> 8 ) & 0xFF;
  payload[16] = LongitudeBinary & 0xFF;
  payload[17] = tle_age;
  payload[18] = TimeToFix;
  payloadLength = 19;
}


void initialize_tle(uint8_t satnumber) {

  Serial.println("Initialize TLE");

  char tle_buf1[69];
  char tle_buf2[69];

  
 // TLE data sat1                  
  char tle_ls1_1[] = "1 44109U 19018AF  20328.91425974 +.00004997 +00000-0 +15809-3 0  9998";
  char tle_ls1_2[] = "2 44109 097.4496 039.1443 0057749 006.4400 353.7571 15.31610125091982"; 
  
 // TLE data sat2                   
  char tle_ls2d_1[] = "1 46492U 20068G   20329.25095194 +.00000475 +00000-0 +39232-4 0  9991";
  char tle_ls2d_2[] = "2 46492 097.6733 262.2449 0020045 051.8117 308.4967 15.03508808008503";   

               

  switch (satnumber) {
     case 1:  
     LacunaSat.init(satname,tle_ls1_1,tle_ls1_2);     //initialize satellite parameters from firmware
     break;
     case 2:
     EEPROM.get(0,tle_buf1);
     EEPROM.get(69,tle_buf2);
     if (twolineChecksum(tle_buf1) && twolineChecksum(tle_buf2)) {
     Serial.println("  EEPROM TLE checksum ok");
     } else {
     Serial.println("  EEPROM TLE checksum failed");
     }
     char tempstr[12];
     memcpy( tempstr, &tle_buf1[18] , 5); tempstr[5] = '\0'; uint16_t bufepoch = atoi(tempstr);
     memcpy( tempstr, &tle_ls2d_1[18] , 5); tempstr[5] = '\0'; uint16_t firmepoch = atof(tempstr);
    Serial.print("  TLE EEPROM epoch: ");
    Serial.println(bufepoch);
    Serial.print("  TLE firmware epoch: ");
    Serial.println(firmepoch);
    if (twolineChecksum(tle_buf1) && twolineChecksum(tle_buf2) && bufepoch>firmepoch) {
     Serial.println("  Use EEPROM TLE");
     LacunaSat.init(satname,tle_buf1,tle_buf2);     //initialize satellite parameters from EEPROM
    } else {
     Serial.println("  Use firmware TLE");
     LacunaSat.init(satname,tle_ls2d_1,tle_ls2d_2);     //initialize satellite parameters from firmware
    }    
     break;
  }
  tle_epoch = getUnixFromJulian(LacunaSat.satrec.jdsatepoch);
}



uint32_t crc32b(uint8_t *data) {
   int i, j;
   uint32_t byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (i < (4*3)) {
      byte = data[i];            // Get next byte.
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--) {    // Do eight times.
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

byte updategps() {

   digitalWrite(LS_GPS_ENABLE, HIGH);
   delay(200);
   
   long startTime = millis(); 
    
   while (millis() - startTime < (FIXTIME*1000))
   {
      long latitude = myGPS.getLatitude();
      long longitude = myGPS.getLongitude();    
      long altitude = myGPS.getAltitudeMSL();      
      byte SIV = myGPS.getSIV();

      digitalWrite(LS_LED_BLUE, HIGH);
          delay(200);
          digitalWrite(LS_LED_BLUE, LOW);
          delay(200);
      Serial.println("Checking GPS");

      Serial.print("  GPS position: ");
      Serial.print(latitude/1.0e7,4);
      Serial.print(", ");
      Serial.print(longitude/1.0e7,4);
      Serial.print(" alt: ");
      Serial.print(altitude/1.0e6,2);
      Serial.print(" (");
      Serial.print(SIV);
      Serial.println(" satellites)");
 
      byte fixType = myGPS.getFixType();
      Serial.print(F("  GPS Fix: "));
      if(fixType == 0) Serial.print(F("No fix"));
      else if(fixType == 1) Serial.print(F("Dead reckoning"));
      else if(fixType == 2) Serial.print(F("2D"));
      else if(fixType == 3) Serial.print(F("3D"));
      else if(fixType == 4) Serial.print(F("GNSS+Dead reckoning"));
      else if(fixType == 5) Serial.print(F("Time only"));
      
      uint16_t new_pdop = (uint16_t)myGPS.getPDOP();
      Serial.print(", PDOP: ");
      Serial.println(new_pdop);
      
      Serial.print("  GPS time: ");
      Serial.print(myGPS.getYear());
      Serial.print("-");
      Serial.print(myGPS.getMonth());
      Serial.print("-");
      Serial.print(myGPS.getDay());
      Serial.print(" ");
      Serial.print(myGPS.getHour());
      Serial.print(":");
      Serial.print(myGPS.getMinute());
      Serial.print(":");
      Serial.println(myGPS.getSecond());
      
      uint32_t unixt = unixTimestamp(myGPS.getYear(),myGPS.getMonth(),myGPS.getDay(),myGPS.getHour(),myGPS.getMinute(),myGPS.getSecond());
      
      Serial.print("  Unix time GPS: ");
      Serial.println(unixt);
      
      if (fixType == 2 || fixType == 3 ) {
      // wait for 3D fix
      //if ( fixType == 3 ) {
        Serial.println("Updating position");
        gnss_lat = latitude/1e7;
        gnss_lon = longitude/1e7;
        if (fixType == 3) {
          gnss_alt = altitude/1e3;
        } else {
          gnss_alt = 0;
        }
        fixepoch=unixt;
    
        tle_epoch = getUnixFromJulian(LacunaSat.satrec.jdsatepoch);
        tle_age = (unixt-tle_epoch)/86400;
      
        Serial.print("  TLE Epoch age (days): ");
        Serial.println( tle_age );
      
        // if we have a fix, time is valid
        int32_t diff = unixt - RTC.getEpoch();
        time_correction = uint32_t( diff );
        Serial.print("  Correcting RTC by ");
        Serial.print(diff);
        Serial.println(" seconds.");
        // compensate our wakeup time
        last_epoch = last_epoch + diff;
        RTC.setEpoch(unixt);
        TimeToFix = (millis() - startTime)/1000;
        return(1);
    }
  }
  digitalWrite(LS_LED_BLUE, LOW);
  return(0);  
}

unsigned long unixTimestamp(int year, int month, int day, int hour, int min, int sec) {
  const short days_since_beginning_of_year[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  int leap_years = ((year-1)-1968)/4
                  - ((year-1)-1900)/100
                  + ((year-1)-1600)/400;
  long days_since_1970 = (year-1970)*365 + leap_years
                      + days_since_beginning_of_year[month-1] + day-1;
  if ( (month>2) && (year%4==0 && (year%100!=0 || year%400==0)) )
    days_since_1970 += 1; /* +leap day, if year is a leap year */
  return sec + 60 * ( min + 60 * (hour + 24*days_since_1970) );
}

uint16_t predictpass(float latitude, float longitude, float altitude, uint32_t* start, uint32_t* duration){

    LacunaSat.site(latitude,longitude,altitude);
 
    passinfo overpass;                            //structure to store overpass info
    LacunaSat.initpredpoint(RTC.getEpoch(), MINELEVATION);     //finds the startpoint

    int timezone = 0; // UTC 
    int  year; int mon; int day; int hr; 
    int mymin; 
    double sec;   
    bool error; 
    unsigned long startcalc = millis();

    error = LacunaSat.nextpass(&overpass,50);     //search for the next overpass, if there are more than 50 maximums below the horizon it returns false
 
    if ( error == 1) { //no error, prints overpass information
        
        invjday(overpass.jdstart ,timezone ,false , year, mon, day, hr, mymin, sec);
        
#ifdef M6P
        if (( hr > 3 )&&( hr < 15 )) {            // If between 15h and 3 h ignore 
         Serial.println("Ignore morning pass");
          return(0); 
        }
#endif 

        
        Serial.println("  Overpass " + String(day) + ' ' + String(mon) + ' ' + String(year));
        Serial.println("  Start: az=" + String(overpass.azstart) + "° " + String(hr) + ':' + String(mymin) + ':' + String(sec));
       
        invjday(overpass.jdmax ,timezone ,false , year, mon, day, hr, mymin, sec);
        Serial.println("  Max: elev=" + String(overpass.maxelevation) + "° " + String(hr) + ':' + String(mymin) + ':' + String(sec));
                
        invjday(overpass.jdstop ,timezone ,false , year, mon, day, hr, mymin, sec);
        Serial.println("  Stop: az=" + String(overpass.azstop) + "° " + String(hr) + ':' + String(mymin) + ':' + String(sec));

       

#ifdef FAKEPASS
        *start = RTC.getEpoch()+240;
        *duration = 30;
#else
        *start = getUnixFromJulian(overpass.jdstart);
        *duration = getUnixFromJulian(overpass.jdstop) - getUnixFromJulian(overpass.jdstart);
#endif
       
    }else{
          Serial.println("Prediction error");
          return(0);
    }
    
    unsigned long einde = millis();
    Serial.println("  (computation time: " + String(einde-startcalc) + " milliseconds)");
    return( (uint16_t)overpass.maxelevation);
}


uint16_t trackingdata(float latitude, float longitude, float altitude, uint32_t unixt ,float* azimuth, float* elevation, float* distance){

    LacunaSat.site(latitude,longitude,altitude);
    LacunaSat.findsat(unixt);

    //  In case of satVis = -2 the satellite is under the horizon
    
    if (LacunaSat.satVis == -2){                   
           Serial.println("Tracking error");
           *azimuth = 0;
           *elevation = 0;
           *distance =0;
           return(0);          
          } else {
           Serial.println("azimuth = " + String( LacunaSat.satAz) + " elevation = " + String(LacunaSat.satEl) + " distance = " + String(LacunaSat.satDist));
           *azimuth   = float(LacunaSat.satAz);
           *elevation = float(LacunaSat.satEl);
           *distance  = float(LacunaSat.satDist);
           return(1); 
          }
     }  

uint32_t UnixToDhhmmss(uint32_t unixtime) {
      uint32_t days =0;
      uint32_t hours =0;
      uint32_t minutes =0;
      uint32_t seconds =0;
      
      days    = uint32_t(unixtime/86400);                                         // Calculate days
      hours   = uint32_t((unixtime - days*86400)/3600);                           // Calculate hours
      minutes = uint32_t((unixtime - days*86400 - hours*3600)/60);                // Calculate minutes
      seconds = uint32_t(unixtime - days*86400 - hours*3600 - minutes*60);        // Calculate seconds
     return(days*1000000+hours*10000+minutes*100+seconds);                        // Return dddhhmmss
    }   

  
  
  void get_new_tle(lsLoraTxParams *TLEtxParams) {

  // lacuna-tle-update generic-device keys
  byte networkKey[] = { 0xA4, 0x81, 0x98, 0xCD, 0x28, 0xEB, 0x31, 0x3F, 0xC9, 0x27, 0x21, 0xDE, 0x59, 0x4E, 0x40, 0x67 };
  byte appKey[] = { 0xD7, 0xD4, 0xD6, 0xF9, 0xC8, 0x62, 0xB2, 0x10, 0x32, 0x1A, 0x39, 0xC9, 0x76, 0x80, 0xAA, 0x2F };
  byte deviceAddress[] = { 0x26, 0x01, 0x1B, 0xDB };

  lsLoraWANParams TLEloraWANParams;
 
  // LoRaWAN session parameters
  lsCreateDefaultLoraWANParams(&TLEloraWANParams, networkKey, appKey, deviceAddress);
  TLEloraWANParams.txPort = 1;
  TLEloraWANParams.rxEnable = true;

  uint32_t uid[3];
  STM32.getUID(uid);
  uint32_t ls_devid = crc32b((uint8_t*)uid);
 
  char tlepayload[255];
  char tle_buf1[69];
  char tle_buf2[69];

  memcpy(tlepayload, &tle_epoch , 4);
  memcpy(tlepayload+4, &ls_devid , 4);
  
  int result = lsSendLoraWAN(&TLEloraWANParams, TLEtxParams, (byte *)tlepayload, 8);
  Serial.print("TTN uplink result: ");
  Serial.println(lsErrorToString(result));

  if (TLEloraWANParams.rxpayloadLength) {    
    if (TLEloraWANParams.rxpayloadLength == 138) {
      Serial.println("  Received TLE via The Things Network");     
      for (uint8_t n = 0; n < 69; n++)
        {
          tle_buf1[n] = tlepayload[n];
          tle_buf2[n] = tlepayload[n+69];
        }
      if (twolineChecksum(tle_buf1) && twolineChecksum(tle_buf2)) {
         Serial.println("  Update TLE and write to EEPROM");
         //initialize satellite parameters with new TLE 
         LacunaSat.init(satname,tle_buf1,tle_buf2);
         tle_epoch = getUnixFromJulian(LacunaSat.satrec.jdsatepoch);
         // Write TLE to EEPROM
         EEPROM.put(0,tle_buf1);
         EEPROM.put(69,tle_buf2);
       } else { 
         Serial.println("  Checksum failed, invalid TLE"); 
       }
    } else {
      Serial.println("  No TLE received");
    } 
  } else {
      Serial.println("  No TLE downlink received");
  }
}
