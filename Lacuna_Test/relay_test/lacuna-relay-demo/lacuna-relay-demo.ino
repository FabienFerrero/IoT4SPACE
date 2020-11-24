/*
 *  _                                   ____                       
 * | |    __ _  ___ _   _ _ __   __ _  / ___| _ __   __ _  ___ ___ 
 * | |   / _` |/ __| | | | '_ \ / _` | \___ \| '_ \ / _` |/ __/ _ \
 * | |__| (_| | (__| |_| | | | | (_| |  ___) | |_) | (_| | (_|  __/
 * |_____\__,_|\___|\__,_|_| |_|\__,_| |____/| .__/ \__,_|\___\___|
 *                                           |_|                   
 * Copyright (C) 2020 Lacuna Space Ltd.
 *
 * Description: Lacuna TTN Relay (demo)
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
 */

#ifndef REGION
#define REGION R_EU868
#endif

/* LOWPOWER enables sleep mode*/
#define LOWPOWER

/* Relay interval in minutes */
#define RELAY_INTERVAL  3

#include <LibLacuna.h>
#include <SPI.h>
#include <time.h>
#include <EEPROM.h>
#include <RTC.h>
#include <SparkFunBME280.h>   // http://librarymanager/All#SparkFun_BME280
#include <Sodaq_LSM303AGR.h>  // http://librarymanager/All#Sodaq_LSM303AGR
#include <SparkFun_Ublox_Arduino_Library.h>   // http://librarymanager/All#SparkFun_Ublox

// This device (transmit)
//static byte networkKey[] = { 0x55, 0x1F, 0xEE, 0x94, 0x6B, 0xEF, 0xC9, 0x20, 0xD9, 0x02, 0x8B, 0x30, 0x3C, 0xC3, 0xB6, 0x01 }; //relay
//static byte networkKey[] = { 0x26, 0x57, 0xDC, 0xCD, 0x09, 0xFC, 0x1D, 0x93, 0x4C, 0x1E, 0x3B, 0x69, 0xAF, 0x89, 0xF3, 0x15 }; //relay3
static byte networkKey[] = { 0xE6, 0x70, 0xA9, 0xB1, 0x80, 0x2F, 0x6F, 0xC2, 0x35, 0x6B, 0x3F, 0x37, 0x69, 0xBF, 0xAA, 0x62 }; //relay4

   
//static byte appKey[] = { 0xD1, 0x23, 0xD3, 0x96, 0x7E, 0xE4, 0xBC, 0xA6, 0x1E, 0x3E, 0x60, 0x4F, 0x81, 0x11, 0xAF, 0xF8 };//relay
//static byte appKey[] = { 0x75, 0xE1, 0xAC, 0xFE, 0xAE, 0xA5, 0x91, 0x58, 0xDA, 0x41, 0x4B, 0x67, 0x9E, 0x6E, 0xA6, 0x1F };//relay3
static byte appKey[] = { 0xC0, 0x2C, 0xE6, 0x0A, 0x52, 0x0C, 0xF3, 0x8E, 0x92, 0x17, 0x51, 0xF5, 0xFE, 0x84, 0xE5, 0x7B };//relay4


// Replace with your device address
//static byte deviceAddress[] = { 0x26, 0x01, 0x33, 0x1D }; //relay
//static byte deviceAddress[] = { 0x26, 0x01, 0x30, 0xB5 };//relay3
static byte deviceAddress[] = { 0x26, 0x01, 0x3A, 0x8F };//relay4




// Device to be relayed (receive)
static byte relay_networkKey[] = { 0x9F, 0x74, 0x4F, 0x40, 0xC3, 0x91, 0x7E, 0x01, 0x6C, 0xD7, 0x87, 0x5E, 0xF5, 0xC0, 0xDB, 0x40 };
static byte relay_appKey[] = { 0x10, 0xF1, 0xA5, 0x34, 0xCB, 0x63, 0x5B, 0xFF, 0x5A, 0xB4, 0x85, 0x7E, 0x2B, 0x3F, 0xD9, 0xC0 };
static byte relay_deviceAddress[] = { 0x26, 0x01, 0x1D, 0x57 };


static lsLoraWANParams loraWANParams;
static lsLoraWANParams relay_loraWANParams;

static char payload[255];
static int payloadLength;

static byte relay_payload[255];
uint8_t relay_length;

uint32_t relay_epoch = 0;
uint16_t relay_false_detects = 0;
uint16_t relay_uplinks = 0;
uint16_t relay_framecounter = 0;
int8_t relay_rssi = 0;
int8_t relay_snr = 0;

volatile uint8_t alarm = 0;
extern volatile int preamble_detect;

lsLoraSatTxParams etxParams;
lsLoraTxParams txParams;
lsLoraTxParams relayParams;

// BME280 sensor
BME280 bme280;

// Accelerometer
Sodaq_LSM303AGR lsm303(LSM303_WIRE);

// GPS
SFE_UBLOX_GPS myGPS;

void setup()
{
  pinMode(LS_LED_BLUE, OUTPUT);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(200);
  digitalWrite(LS_LED_BLUE, LOW);
  
  Serial.begin(9600);
  while (!Serial && millis() < 3000);

  Serial.println("Initializing");

  Serial.print("LibLacuna version: ");
  Serial.println(LIBLACUNA_VERSION);

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

  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  
  bme280.setMode(MODE_SLEEP); 
  lsm303.disableMagnetometer();
  lsm303.disableAccelerometer();
  
  lsCreateDefaultLoraWANParams(&loraWANParams, networkKey, appKey, deviceAddress);
  loraWANParams.txPort = 1;
  loraWANParams.rxEnable = false;

  lsCreateDefaultLoraWANParams(&relay_loraWANParams, relay_networkKey, relay_appKey, relay_deviceAddress);
 
  lsSX126xConfig cfg;
  lsCreateDefaultSX126xConfig(&cfg);
  
  int result = lsInitSX126x(&cfg);
  if(result) Serial.println(lsErrorToString(result));

  /* LoRa parameters for device (forward) */
  lsCreateDefaultLoraTxParams(&txParams);

  txParams.power = 14;
  txParams.spreadingFactor = lsLoraSpreadingFactor_7;
  txParams.codingRate = lsLoraCodingRate_4_5;
  txParams.invertIq = false;
  txParams.frequency = 867300000;
  txParams.bandwidth = lsLoraBandwidth_125_khz;
  txParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;
  txParams.preambleLength = 8;

  /* LoRa parameters for relay (receive) */
  lsCreateDefaultLoraTxParams(&relayParams);

  relayParams.spreadingFactor = lsLoraSpreadingFactor_7;
  relayParams.invertIq = false;
  relayParams.frequency = 865000000;
  //relayParams.bandwidth = lsLoraBandwidth_41_66_khz;
  relayParams.bandwidth = lsLoraBandwidth_125_khz;
  relayParams.syncWord = LS_LORA_SYNCWORD_PUBLIC;


  setAlarm(RTC.getEpoch() + RELAY_INTERVAL*60);
}

void loop() 
{
  /* set sleep_func to 'LS200_Sleep' for low-power or 'NULL' for debug/serial */
#ifdef LOWPOWER
  uint32_t rxlength = lsRelayLora(&relay_loraWANParams, &relayParams, relay_payload, LS200_Sleep);
#else
  uint32_t rxlength = lsRelayLora(&relay_loraWANParams, &relayParams, relay_payload, NULL);
#endif

  if (rxlength) { 

digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
  delay(100);
  digitalWrite(LS_LED_BLUE, HIGH);
  delay(50);
  digitalWrite(LS_LED_BLUE, LOW);
    
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

    //alarm = 1; // send directly the packet
  }

  if (alarm == 1) {
    /* wakeup to relay */
    Serial.println("Alarm");
    alarm = 0;
    digitalWrite(LS_LED_BLUE, HIGH);
    delay(100);
    digitalWrite(LS_LED_BLUE, LOW);
    Serial.print("Epoch: ");
    Serial.println(RTC.getEpoch());
    generateRelayStatus(false);
    int lora_result  = lsSendLoraWAN(&loraWANParams, &txParams, (byte *)payload, payloadLength);
    Serial.print("LoRa TX Status Result: ");
    Serial.println(lsErrorToString(lora_result)); 
    if (relay_length) {
      int lora_result  = lsSendLora(&txParams, (byte *)relay_payload, relay_length, false);
      Serial.print("LoRa TX Relay Result: ");
      Serial.println(lsErrorToString(lora_result)); 
      relay_length = 0;
    } else {
      Serial.println("Nothing to relay");
    }
    setAlarm(RTC.getEpoch() + RELAY_INTERVAL*60);  
  } else if (!rxlength) {
      relay_false_detects++;
      Serial.println("False detect");
  }
}

void alarmMatch() { 
  alarm = 1; 
  preamble_detect = 1;
}

void setAlarm(uint32_t alarmepoch)
{  
    time_t t;
    struct tm tm;

    t = (time_t)alarmepoch;
    gmtime_r(&t, &tm);
    
    RTC.setAlarmTime(tm.tm_hour, tm.tm_min, tm.tm_sec);
    RTC.setAlarmDay(tm.tm_mday);
    
    RTC.enableAlarm(RTC.MATCH_HHMMSS);
    RTC.attachInterrupt(alarmMatch);
}

void LS200_Sleep()  
{
  digitalWrite(LS_GPS_ENABLE, LOW);
  digitalWrite(LS_VERSION_ENABLE, LOW);
  digitalWrite(LS_GPS_V_BCKP, LOW);
  
  bme280.setMode(MODE_SLEEP); 
  lsm303.disableMagnetometer();
  lsm303.disableAccelerometer();
  
  SPI.end();
  STM32.stop();
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

  uint32_t relay_delay = RTC.getEpoch() - relay_epoch;

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
        
  payloadLength = 23;
}
