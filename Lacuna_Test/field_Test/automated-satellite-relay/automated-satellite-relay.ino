/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2020 Lacuna Space Ltd.
 *
 * Description: Automated satellite relay
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
#define INTERVAL 10
#define NUMTX 3

// Sleep time between periodic GPS checks (and TTN updates)
#define SLEEPMINUTES 30

// Minimum elevation for satellite
#define MINELEVATION 28

// max. 250 seconds for GPS fix
#define FIXTIME 180

// Dont sync GPS within this interval before a scheduled transmission
#define GUARDTIME (FIXTIME+30)

// Only use for testing
//#define FAKEPASS
//#define DEBUGSLEEP

// Keys and device address are MSB

// Keys and device address are MSB
//static byte networkKey[] = { 0xB3, 0xB8, 0x9E, 0x4B, 0x54, 0xA0, 0x16, 0xFC, 0x5E, 0x1B, 0xDC, 0xFE, 0x12, 0x04, 0x2F, 0x1B }; //test1
//static byte networkKey[] = { 0xD9, 0xA7, 0x57, 0xD6, 0xF1, 0xED, 0x56, 0xA4, 0x90, 0xE5, 0xA3, 0xAA, 0x09, 0x8D, 0x78, 0x8A }; //test2
//static byte networkKey[] = { 0x75, 0xA3, 0x0E, 0xD2, 0xD0, 0xB7, 0xC7, 0xDA, 0x35, 0xC5, 0xE9, 0xE5, 0xD7, 0xFE, 0x28, 0xE5 }; //test3
//static byte networkKey[] = { 0x0A, 0xEB, 0x4B, 0xA6, 0x4E, 0xE2, 0xE9, 0xCC, 0xA1, 0xD3, 0x75, 0x18, 0x6E, 0xFE, 0xE1, 0x91 }; //test4
//static byte networkKey[] = { 0x14, 0xC6, 0xA5, 0x48, 0x54, 0x49, 0x16, 0x99, 0xB7, 0x12, 0xFD, 0x16, 0x8A, 0x31, 0x3F, 0xE3 }; //test5
//static byte networkKey[] = { 0xB0, 0x55, 0xBF, 0x28, 0x6A, 0x02, 0x55, 0x46, 0xF8, 0x2D, 0x90, 0xC1, 0xD3, 0x29, 0x83, 0x28 }; //test6
//static byte networkKey[] = { 0x34, 0xDC, 0x1C, 0xED, 0x4C, 0x55, 0x8C, 0x26, 0xC0, 0xCA, 0x47, 0x9F, 0x6B, 0x84, 0xBD, 0x14 }; //test7
//static byte networkKey[] = { 0x76, 0x7B, 0xAF, 0xA3, 0xD6, 0x0E, 0x0E, 0xC4, 0x92, 0x8A, 0x69, 0x18, 0x5E, 0xE7, 0xA8, 0x4C }; //test8
static byte networkKey[] = { 0x55, 0x1F, 0xEE, 0x94, 0x6B, 0xEF, 0xC9, 0x20, 0xD9, 0x02, 0x8B, 0x30, 0x3C, 0xC3, 0xB6, 0x01 }; //relay1
//static byte networkKey[] = { 0xA4, 0xFB, 0x92, 0xAC, 0x60, 0xA1, 0x44, 0xF3, 0x47, 0x41, 0x6A, 0xBB, 0x86, 0xCE, 0x14, 0xFA }; //relay2
//static byte networkKey[] = { 0x26, 0x57, 0xDC, 0xCD, 0x09, 0xFC, 0x1D, 0x93, 0x4C, 0x1E, 0x3B, 0x69, 0xAF, 0x89, 0xF3, 0x15 }; //relay3
//static byte networkKey[] = { 0xE6, 0x70, 0xA9, 0xB1, 0x80, 0x2F, 0x6F, 0xC2, 0x35, 0x6B, 0x3F, 0x37, 0x69, 0xBF, 0xAA, 0x62 }; //relay4


//static byte appKey[] = { 0x4D, 0x93, 0xF6, 0x0A, 0x2E, 0x6B, 0xA7, 0x95, 0xA9, 0x79, 0x9E, 0xB5, 0x67, 0xB3, 0x46, 0x10 };//test1
//static byte appKey[] = { 0xA3, 0xE0, 0xB9, 0x52, 0xE4, 0x6A, 0xB8, 0x20, 0xB3, 0x9D, 0xE4, 0xFC, 0xD2, 0x98, 0x76, 0x9E };//test2
//static byte appKey[] = { 0x71, 0x1E, 0xAE, 0xBA, 0x4B, 0xFE, 0x3F, 0x25, 0xE8, 0x7F, 0x5D, 0x9C, 0x8D, 0xC0, 0xA3, 0xDA };//test3
//static byte appKey[] = { 0x29, 0x0F, 0x53, 0xE3, 0xC1, 0x60, 0x76, 0x2B, 0x73, 0xCD, 0x13, 0x86, 0xF5, 0x4E, 0xB7, 0x82 };//test4
//static byte appKey[] = { 0x5F, 0x57, 0xDD, 0x2F, 0x13, 0x6A, 0x6A, 0x7B, 0x07, 0xF8, 0x4C, 0xC9, 0xDA, 0x35, 0xC6, 0x71 };//test5
//static byte appKey[] = { 0xAD, 0x12, 0x7F, 0xD1, 0x52, 0x87, 0xCF, 0x57, 0x3B, 0xA9, 0x86, 0x8E, 0x0A, 0xEB, 0xA1, 0x40 };//test6
//static byte appKey[] = { 0xE5, 0xCA, 0x9E, 0x66, 0x71, 0xFA, 0xB3, 0x87, 0xC6, 0xA6, 0xFA, 0x3A, 0xA9, 0x27, 0xE7, 0x38 };//test7
//static byte appKey[] = { 0xCE, 0x1C, 0x3C, 0xA2, 0x01, 0x22, 0x49, 0x72, 0x82, 0xDE, 0x31, 0x94, 0x70, 0xCF, 0xAC, 0x19 };//test8
static byte appKey[] = { 0xD1, 0x23, 0xD3, 0x96, 0x7E, 0xE4, 0xBC, 0xA6, 0x1E, 0x3E, 0x60, 0x4F, 0x81, 0x11, 0xAF, 0xF8 };//relay1
//static byte appKey[] = { 0xFC, 0xB6, 0x64, 0x2F, 0xDE, 0xA8, 0x00, 0xCC, 0x27, 0xFF, 0xB9, 0x06, 0xA2, 0xB0, 0xCD, 0xA8 };//relay2
//static byte appKey[] = { 0x75, 0xE1, 0xAC, 0xFE, 0xAE, 0xA5, 0x91, 0x58, 0xDA, 0x41, 0x4B, 0x67, 0x9E, 0x6E, 0xA6, 0x1F };//relay3
//static byte appKey[] = { 0xC0, 0x2C, 0xE6, 0x0A, 0x52, 0x0C, 0xF3, 0x8E, 0x92, 0x17, 0x51, 0xF5, 0xFE, 0x84, 0xE5, 0x7B };//relay4


// Replace with your device address

//static byte deviceAddress[] = { 0x26, 0x01, 0x1B, 0x1A };//test1
//static byte deviceAddress[] = { 0x26, 0x01, 0x1B, 0x9E };//test2
//static byte deviceAddress[] = { 0x26, 0x01, 0x15, 0xEF };//test3
//static byte deviceAddress[] = { 0x26, 0x01, 0x11, 0xE1 };//test4
//static byte deviceAddress[] = { 0x26, 0x01, 0x16, 0xE1 };//test5
//static byte deviceAddress[] = { 0x26, 0x01, 0x3F, 0xCF };//test6
//static byte deviceAddress[] = { 0x26, 0x01, 0x38, 0x80 };//test7
//static byte deviceAddress[] = { 0x26, 0x01, 0x3F, 0x0B };//test8
static byte deviceAddress[] = { 0x26, 0x01, 0x33, 0x1D }; //relay
//static byte deviceAddress[] = { 0x26, 0x01, 0x31, 0xE9 };//relay2
//static byte deviceAddress[] = { 0x26, 0x01, 0x30, 0xB5 };//relay3
//static byte deviceAddress[] = { 0x26, 0x01, 0x3A, 0x8F };//relay4



// Device to be relayed (receive)
static byte relay_networkKey[] = { 0x9F, 0x74, 0x4F, 0x40, 0xC3, 0x91, 0x7E, 0x01, 0x6C, 0xD7, 0x87, 0x5E, 0xF5, 0xC0, 0xDB, 0x40 };
static byte relay_appKey[] = { 0x10, 0xF1, 0xA5, 0x34, 0xCB, 0x63, 0x5B, 0xFF, 0x5A, 0xB4, 0x85, 0x7E, 0x2B, 0x3F, 0xD9, 0xC0 };
static byte relay_deviceAddress[] = { 0x26, 0x01, 0x1D, 0x57 };


static lsLoraWANParams loraWANParams;
static lsLoraWANParams relay_loraWANParams;

static lsLoraSatTxParams SattxParams;
static lsLoraTxParams txParams;
static lsLoraTxParams relayParams;

volatile uint8_t alarm = 0;
extern volatile int preamble_detect;

static byte relay_payload[255];
uint8_t relay_length;

uint32_t relay_epoch = 0;
uint16_t relay_false_detects = 0;
uint16_t relay_uplinks = 0;
uint16_t relay_framecounter = 0;
int8_t relay_rssi = 0;
int8_t relay_snr = 0;

static char payload[255];
static int payloadLength;

float gnss_lat;
float gnss_lon;
float gnss_alt;

uint32_t fixepoch;
uint8_t  TimeToFix;

uint32_t last_epoch;
uint32_t alarmepoch;
uint32_t txepoch = 0;
uint8_t  txinterval = 0;

int32_t sleepseconds;
uint8_t wakeupreason = 0;

// interval between GPS checks
uint16_t sleeptimesec = 60 * SLEEPMINUTES; // in seconds

// SGP4
Sgp4 LacunaSat;
char satname[] = "LS1";
uint32_t tle_epoch;
uint8_t  tle_age = 0;  // 0xFF;

// BME280 sensor
BME280 bme280;

// Accelerometer
Sodaq_LSM303AGR lsm303(LSM303_WIRE);

// GPS
SFE_UBLOX_GPS myGPS;

void setup() {
  
  Serial.begin(9600);

  while (!Serial && millis() < 3000);
  
  Serial.println("Initializing");

  Serial.print("LibLacuna version: ");
  Serial.println(LIBLACUNA_VERSION);

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

  analogReadResolution(12);

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
  lsm303.disableMagnetometer();

  // BME 280
  bme280.setI2CAddress(0x76);
  if (bme280.beginI2C(BME280_WIRE) == false)
  {
    Serial.println("The sensor did not respond. Please check wiring.");
  }

  delay(70);

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

  // SX1262 configuration for lacuna LS200 board
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg);

  // Initialize SX1262
  int result = lsInitSX126x(&cfg);
  if(result) Serial.println(lsErrorToString(result));

  // LoRaWAN session parameters
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = 1;
  loraWANParams.rxEnable = true;

  lsCreateDefaultLoraWANParams(&relay_loraWANParams, relay_networkKey, relay_appKey, relay_deviceAddress);
 
  // transmission parameters for Lacuna satellites
  lsCreateDefaultLoraSatTxParams(&SattxParams);
  SattxParams.power = 21;
  SattxParams.nbSync = 2;
  SattxParams.codingRate = lsLoraECodingRate_1_2; 
  
  // transmission parameters for terrestrial LoRa
  lsCreateDefaultLoraTxParams(&txParams);

  // modify LoRa parameters
  txParams.power = 10;
  txParams.spreadingFactor = lsLoraSpreadingFactor_8;

   /* LoRa parameters for relay (receive) */
  lsCreateDefaultLoraTxParams(&relayParams);

  relayParams.spreadingFactor = lsLoraSpreadingFactor_7;
  relayParams.invertIq = false;
  relayParams.frequency = 865000000;
  //relayParams.bandwidth = lsLoraBandwidth_41_66_khz;
  relayParams.bandwidth = lsLoraBandwidth_125_khz;
  relayParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;

  // Initialize satellite orbit data 
  LacunaSat.site(0,0,0);
  initialize_tle();

  // Update TLE
  get_new_tle(&txParams);
  
}

void loop() {

  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);

  if (wakeupreason != 3) {
    last_epoch = RTC.getEpoch();
  }

  alarm = 0;

  if (wakeupreason == 0) {

    Serial.println("Periodic wakeup");

    byte fix = updategps();
    
      if (fix) {
          Serial.println("New position fix");
          // predict satellite passes
          uint32_t start;
          uint32_t duration;
          uint16_t maxelevation = predictpass(gnss_lat, gnss_lon, gnss_alt, &start, &duration);
      
          if (maxelevation != 0) {
    
            Serial.print("  Next pass set, ");
            Serial.print("max. elevation: ");
            Serial.print(maxelevation);
            Serial.print(", start at ");
            Serial.print(start);
            Serial.print(" duration: ");
            Serial.println(duration);
    
            txinterval = INTERVAL + tle_age * 1;
            txepoch = start + (duration/2) - (txinterval*(NUMTX-1))/2;
    
            Serial.print("TX Epoch set to: ");
            Serial.print(txepoch);
            Serial.print(", interval: ");
            Serial.print(txinterval);
            Serial.println(" seconds");
    
          } else {
            Serial.println("No pass found?");
          }
          
        } 
    
      generateRelayStatus(false);
     
      Serial.println("Sending LoRa (TTN) message");  
      int lora_result = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);
      Serial.print("Result: ");
      Serial.println(lsErrorToString(lora_result));

      // Update TLE if needed
      if (tle_age > 4) {
        get_new_tle(&txParams);
      }
      
  } else if (wakeupreason == 1) {

    Serial.println("satellite TX wakeup");
  
    uint8_t seq = 1;

    while (seq <= NUMTX) {

      Serial.println("Transmit");

      unsigned long starttx = millis();

      generateRelayStatus(true);
      Serial.println("Sending LoraSat status message");
      int sat_result = lsSendLoraSatWAN(&loraWANParams, &SattxParams, (byte *)payload, payloadLength);
      Serial.print("TX status Result: ");
      Serial.println(lsErrorToString(sat_result));
      if (relay_length) {
        Serial.println("Sending LoraSat relay message");
        int relay_result = lsSendLoraSat(&SattxParams, (byte *)relay_payload, relay_length);
        Serial.print("TX relay Result: ");
        Serial.println(lsErrorToString(relay_result)); 
      } else {
        Serial.println("Nothing to relay");
      }

      unsigned long endtx = millis();

      int16_t waittime = txinterval*1000 - (endtx - starttx);
   
      Serial.print("Wait ");
      Serial.print(waittime);
      Serial.println(" milliseconds");

      // don't wait after the last transmission
      if (waittime>0 && seq < NUMTX) { 
        // TODO: MAKE THIS LOW-POWER!
        delay( waittime );
      }

      seq++;

    } 

    /* set relay length to 0 */
    relay_length = 0;
  
  } else if (wakeupreason == 3) {
    Serial.println("Relay received");
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
    
    /* set sleep_func to 'LS200_Sleep' for low-power or 'NULL' for debug/serial */
#ifdef DEBUGSLEEP
    uint32_t rxlength = lsRelayLora(&relay_loraWANParams, &relayParams, relay_payload, NULL);
#else 
    uint32_t rxlength = lsRelayLora(&relay_loraWANParams, &relayParams, relay_payload, LS200_sleep);
#endif

    if (rxlength) { 
      /* valid relay data received */
      relay_length = rxlength;
      relay_uplinks++;
      relay_rssi = relayParams.rssi;
      relay_snr = relayParams.snr;
      relay_framecounter = (relay_payload[7] << 8) + relay_payload[6];
      relay_epoch = RTC.getEpoch();
  
      Serial.println("Valid uplink received.");
      Serial.print("  SNR: ");
      Serial.println(relayParams.snr);
      Serial.print("  RSSI: ");
      Serial.println(relayParams.rssi);
      Serial.print("  SignalRSSI: ");
      Serial.println(relayParams.signalrssi);
      Serial.print("  ");
      for (char n = 0; n < rxlength; n++)
          {
            Serial.print (relay_payload[n],HEX);
            Serial.write (" ");    
          }
      Serial.println();    
    }
    
    if (!alarm) {
      wakeupreason = 3;
      alarm = 0;
    }

  }

  Serial.println("-- WAKE");
  
}

void alarmMatch() {
  alarm = 1; 
  preamble_detect = 1;
}

void LS200_sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, HIGH);
  
  bme280.setMode(MODE_SLEEP); 
  lsm303.disableAccelerometer();

  SPI.end();
  delay(10);
  STM32.stop();

  // Sleep...
  
  SPI.begin();

}

void generateRelayStatus(bool satellite) 
{

  bme280.setMode(MODE_FORCED);
  lsm303.enableAccelerometer();
 
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
  uint32_t relay_delay = epoch - relay_epoch;

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
  payload[11] = (relay_false_detects >> 8) & 0xff;
  payload[12] = relay_false_detects & 0xff;
  payload[13] = (relay_uplinks >> 8) & 0xff;
  payload[14] = relay_uplinks & 0xff;
  payload[15] = relay_rssi;
  payload[16] = relay_snr;
  payload[17] = (relay_framecounter >> 8) & 0xff;
  payload[18] = relay_framecounter & 0xff;
  payload[19] = (relay_delay >> 24) & 0xff;
  payload[20] = (relay_delay >> 16) & 0xff;
  payload[21] = (relay_delay >> 8) & 0xff;
  payload[22] = relay_delay & 0xff;
  payload[23] = ( LatitudeBinary >> 16 ) & 0xFF;
  payload[24] = ( LatitudeBinary >> 8 ) & 0xFF;
  payload[25] = LatitudeBinary & 0xFF;
  payload[26] = ( LongitudeBinary >> 16 ) & 0xFF;
  payload[27] = ( LongitudeBinary >> 8 ) & 0xFF;
  payload[28] = LongitudeBinary & 0xFF;
  payload[29] = tle_age;
  payload[30] = TimeToFix;
  
  payloadLength = 31;
}


void initialize_tle() {

  Serial.println("Initialize TLE");
  
  char tle_buf1[69];
  char tle_buf2[69];

  char tle_line1[] = "1 44109U 19018AF  20299.90466589 +.00004584 +00000-0 +14564-3 0  9990";
  char tle_line2[] = "2 44109 097.4530 009.9886 0059624 107.5337 253.2431 15.31409454087540";

  EEPROM.get(0,tle_buf1);
  EEPROM.get(69,tle_buf2);

  if (twolineChecksum(tle_buf1) && twolineChecksum(tle_buf2)) {
    Serial.println("  EEPROM TLE checksum ok");
  } else {
    Serial.println("  EEPROM TLE checksum failed");
  }

  char tempstr[12];
  memcpy( tempstr, &tle_buf1[18] , 5); tempstr[5] = '\0'; uint16_t bufepoch = atoi(tempstr);
  memcpy( tempstr, &tle_line1[18] , 5); tempstr[5] = '\0'; uint16_t firmepoch = atof(tempstr);

  Serial.print("  TLE EEPROM epoch: ");
  Serial.println(bufepoch);
  Serial.print("  TLE firmware epoch: ");
  Serial.println(firmepoch);

  if (twolineChecksum(tle_buf1) && twolineChecksum(tle_buf2) && bufepoch>firmepoch) {
     Serial.println("  Use EEPROM TLE");
     LacunaSat.init(satname,tle_buf1,tle_buf2);     //initialize satellite parameters from EEPROM
  } else {
     Serial.println("  Use firmware TLE");
     LacunaSat.init(satname,tle_line1,tle_line2);     //initialize satellite parameters from firmware
  }
  tle_epoch = getUnixFromJulian(LacunaSat.satrec.jdsatepoch);
}


void get_new_tle(lsLoraTxParams *TLEtxParams) {

  // lacuna-tle-update generic-device keys
  byte networkKey[] = { 0xCB, 0x8C, 0xA6, 0x0B, 0xF7, 0xA5, 0xBC, 0x21, 0x16, 0x81, 0x28, 0x70, 0x76, 0x1B, 0xFF, 0xE8 };
  byte appKey[] = { 0x69, 0x95, 0xDB, 0x2F, 0x04, 0x55, 0x35, 0x84, 0x8C, 0xB8, 0x50, 0x13, 0x40, 0xC7, 0x27, 0xD2 };
  byte deviceAddress[] = { 0x26, 0x01, 0x14, 0x06 };

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
        Serial.println("  Overpass " + String(day) + ' ' + String(mon) + ' ' + String(year));
        Serial.println("  Start: az=" + String(overpass.azstart) + "° " + String(hr) + ':' + String(mymin) + ':' + String(sec));
       
        invjday(overpass.jdmax ,timezone ,false , year, mon, day, hr, mymin, sec);
        Serial.println("  Max: elev=" + String(overpass.maxelevation) + "° " + String(hr) + ':' + String(mymin) + ':' + String(sec));
                
        invjday(overpass.jdstop ,timezone ,false , year, mon, day, hr, mymin, sec);
        Serial.println("  Stop: az=" + String(overpass.azstop) + "° " + String(hr) + ':' + String(mymin) + ':' + String(sec));

#ifdef FAKEPASS
        *start = RTC.getEpoch()+60;
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
