/*
     This sketch uses a SIM7000A fona shield on an Arduino Mega to send water level, pressure, location, temperature, and battery voltage
     data to an Adafruit IO dashboard. The sampling rate, initial sea level, and deployment are controlled from the
     dashboard. MQTT protocol is used.

     Written by Corinne Smith Mar 2022
     Adapted from: botletics, Adafruit
*/

#include "Adafruit_FONA.h"            // from botletics: https://github.com/botletics/SIM7000-LTE-Shield/tree/master/Code
#include "Adafruit_MQTT.h"            // from adafruit:  https://github.com/adafruit/Adafruit_MQTT_Library
#include "Adafruit_MQTT_FONA.h"

#include <Adafruit_MPRLS.h>           // MPRLS (PRESSURE) SENSOR LIBRARY
#include <Adafruit_MCP9808.h>         // MCP9808 (TEMPERATURE) SENSOR LIBRARY 
#include <NewPing.h>                  // JSN-SR04 LIBRARY https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home 
#include <SD.h>                       // SD CARD LIBRARY
#include <SPI.h>                      // SERIAL PERIPHERAL INTERFACE LIBRARY
#include <DS3232RTC.h>                // RTC LIBRARY https://github.com/JChristensen/DS3232RTC
#include <Wire.h>                     // I2C COMMUNICATION LIBRARY
#include <OneWire.h>                  // 1 WIRE COMMUNICATION LIBRARY
#include <DallasTemperature.h>        // EXTERNAL DS18B20 TEMPERATURE SENSOR LIBRARY
#include <Adafruit_INA219.h>          // INA219 CURRENT SENSOR LIBRARY
#include <avr/sleep.h>                // SLEEP LIBRARIES
#include <avr/power.h>

/*
   SPI PINS FOR MEGA:
      SCK - 13, 52
      MISO - 12, 50
      MOSI - 11, 51
      SS - general GPIO (53 here)
*/

#define SIMCOM_7000                   // cellular MCU we are using

// FONA PINS -----------------------------------------------------------------------------------------
#define FONA_PWRKEY 6
#define FONA_RST 7
#define FONA_DTR 8
#define FONA_TX 10
#define FONA_RX 11

// indicator LEDs
#define redLed 26                      // MQTT publish error
#define yellowLed 24                   // data collection occurring
#define greenLed 23                    // MQTT connected
#define blueLed 22                     // network connected
#define whiteLed 25                    // GPS error
#define interrupt 5                    // interrupt pin for RTC (NOTE: 5 is INT5, the hardware interrupt that is on pin 18)

int sampling_rate = 15;                // initialize the delay between loops. This can be changed with subscribe
const int keepAlive_mins = round(MQTT_CONN_KEEPALIVE / 60);

// send AT commands via fona's software serial
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

// construct an instance of the LTE fona
Adafruit_FONA_LTE fona = Adafruit_FONA_LTE();

// MQTT SETUP WITH ADAFRUIT IO -----------------------------------------------------------------------
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "YOUR AIO USER"
#define AIO_KEY         "YOUR AIO KEY"

// pass in fona class and server details to mqtt class
Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//// ******************* Uncomment for the WHS 1 sensor ***************************
//// THE PUBLISHING FEEDS ------------------------------------------------------------------------------
//Adafruit_MQTT_Publish feed_location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/location/csv");
//Adafruit_MQTT_Publish feed_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperature");
//Adafruit_MQTT_Publish feed_pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure");
//Adafruit_MQTT_Publish feed_stage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/stage");
//Adafruit_MQTT_Publish feed_pts = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressure-to-stage");
//Adafruit_MQTT_Publish feed_update_gps_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/update-gps");
//Adafruit_MQTT_Publish feed_fona_lipo = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lipo-battery");
//Adafruit_MQTT_Publish feed_external_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/external-temperature");
//Adafruit_MQTT_Publish feed_voltage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/package-battery");
//Adafruit_MQTT_Publish feed_power = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/package-power");
//Adafruit_MQTT_Publish feed_contact = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/contact-sensor");
//
//// THE SUBSCRIBING FEEDS -----------------------------------------------------------------------------
//Adafruit_MQTT_Subscribe feed_deploy = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/deploy");
//Adafruit_MQTT_Subscribe feed_sampling_rate = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/sampling-rate");
//Adafruit_MQTT_Subscribe feed_sea_level = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/initial-sea-level");
//Adafruit_MQTT_Subscribe feed_update_gps_sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/update-gps");



//// ******************* Uncomment for the WHS 2 sensor ***************************
// THE PUBLISHING FEEDS ------------------------------------------------------------------------------
Adafruit_MQTT_Publish feed_location = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.location/csv");
Adafruit_MQTT_Publish feed_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.temperature");
Adafruit_MQTT_Publish feed_pressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.pressure");
Adafruit_MQTT_Publish feed_stage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.stage");
Adafruit_MQTT_Publish feed_pts = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.pressure-to-stage");
Adafruit_MQTT_Publish feed_update_gps_pub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.update-gps");
Adafruit_MQTT_Publish feed_fona_lipo = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.lipo-battery");
Adafruit_MQTT_Publish feed_external_temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.external-temperature");
Adafruit_MQTT_Publish feed_voltage = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.package-battery");
Adafruit_MQTT_Publish feed_power = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.package-power");
Adafruit_MQTT_Publish feed_contact = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.contact-sensor");
Adafruit_MQTT_Publish feed_connection = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/whs-2.cellular-connection");

// THE SUBSCRIBING FEEDS -----------------------------------------------------------------------------
Adafruit_MQTT_Subscribe feed_deploy = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/whs-2.deploy");
Adafruit_MQTT_Subscribe feed_sampling_rate = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/whs-2.sampling-rate");
Adafruit_MQTT_Subscribe feed_sea_level = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/whs-2.initial-sea-level");
Adafruit_MQTT_Subscribe feed_update_gps_sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/whs-2.update-gps");

// define the SS for SD card
#define chipSelect 53

// construct the MCP9808
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// construct the MPRLS
Adafruit_MPRLS mpr = Adafruit_MPRLS(-1, -1);      // Adafruit_MPRLS(RESET_PIN, EOC_PIN)

// construct the JSN-SR04
#define trigPin 47
#define echoPin 46
NewPing sonar(trigPin, echoPin);

// define the pressure sensor pin
#define pressurePin A10

// define the water contact sensor pin
#define contactPin 3

// construct the DS18B20
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);

// construct the INA219
Adafruit_INA219 ina219;

// some global variables
uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
char imei[16] = {0};
bool deployed = false;
bool new_time = false;
bool new_loc = false;
float initial_distance = 0;
float initial_feet_of_water = 0;
float sea_level = 0;


void setup() {
  Serial.begin(9600);
  Serial.println(F("*** Executing WHS_v2.0.6.ino ***"));

  // configure the led
  pinMode(redLed, OUTPUT);
  digitalWrite(redLed, LOW);
  pinMode(yellowLed, OUTPUT);
  digitalWrite(yellowLed, LOW);
  pinMode(greenLed, OUTPUT);
  digitalWrite(greenLed, LOW);
  pinMode(blueLed, OUTPUT);
  digitalWrite(blueLed, LOW);
  pinMode(whiteLed, OUTPUT);
  digitalWrite(whiteLed, LOW);

  // configure the JSN-SR04 pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // configure the SD SS pin
  pinMode(chipSelect, OUTPUT);

  // configure the RTC interrupt
  pinMode(interrupt, INPUT_PULLUP);

  // check if SD card began
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card not found");
    while (1);
  }

  // check if MPRLS began
  if (! mpr.begin()) {
    Serial.println(F("MPRLS not found"));
    while (1);
  }
  
  // wake up the MCP9808
  tempsensor.wake();                                  
  if (!tempsensor.begin(0x19)) {
    Serial.println("MCP9808 not found");
    while (1);
  }

  // check if INA219 began
  if( !ina219.begin()) {
    Serial.println("INA219 not found");
    while(1);
  }

  // begin the ds18b20
  ds18b20.begin();

  // begin the RTC
  RTC.begin();

  // initialize alarms and clear any past alarm flags or interrupts
  RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
  RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
  RTC.alarm(ALARM_1);
  RTC.alarm(ALARM_2);
  RTC.alarmInterrupt(ALARM_1, true);
  RTC.alarmInterrupt(ALARM_2, false);
  RTC.squareWave(SQWAVE_NONE);


  pinMode(FONA_RST, OUTPUT);
  digitalWrite(FONA_RST, HIGH);             // reset is default high

  pinMode(FONA_DTR, OUTPUT);
  digitalWrite(FONA_DTR, LOW);              // initialize to LOW so that if clock is high fona will be awake

  fona.powerOn(FONA_PWRKEY);                // power on fona by pulsing power key

  moduleSetup();                            // establish serial communication, find fona, determine device IMEI

  fona.setFunctionality(1);                 // set fona to its full functionality (AT+CFUN=1)

  fona.setNetworkSettings(F("hologram"));   // sets APN as 'hologram', used with Hologram SIM card

  //fona.set_eDRX(1,5,0100);                // sets eDRX mode (not supported by T-mobile LTE

  // first disable data
  if (!fona.enableGPRS(false)) Serial.println(F("Failed to disable data!"));

  // enable data
  while (!fona.enableGPRS(true)) {
    Serial.println(F("Failed to enable data, retrying..."));
    delay(2000); // Retry every 2s
  }
  Serial.println(F("Enabled data!"));

  // subscribe to the subscription feeds
  mqtt.subscribe(&feed_deploy);
  mqtt.subscribe(&feed_sampling_rate);
  mqtt.subscribe(&feed_sea_level);
  mqtt.subscribe(&feed_update_gps_sub);

  // set the keepalive interval in seconds
  mqtt.setKeepAliveInterval(600);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  readRSSI();

  // first ensure that we are finished configuring everything in Adafruit IO and want the package to start collecting data
  while (! deployed) {

    while (!netStatus()) {
      Serial.println(F("Failed to connect to cell network, retrying..."));
      delay(2000); // Retry every 2s
      digitalWrite(blueLed, LOW);
    }

    MQTT_connect();
    // subscription packet subloop, this runs and waits for the toggle switch in Adafruit IO to turn on
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(5000))) {
      if (subscription == &feed_deploy) {
        Serial.print(F("*** Package: ")); Serial.println((char *)feed_deploy.lastread);
        if (strcmp(feed_deploy.lastread, "ON") == 0) {
          Serial.println(F("***Package is deployed"));
          deployed = true;
        }
        else if (strcmp(feed_deploy.lastread, "OFF") == 0) {
          Serial.println(F("***Package not ready"));
          //delay(5000);    // wait five seconds to not spam the serial monitor
        }
      }
      if (subscription == &feed_sea_level) {
        Serial.print(F("*** Elevation: "));
        Serial.println((char *)feed_sea_level.lastread);
        delay(100);
        sea_level = atof((char *)feed_sea_level.lastread);
      }
      if (subscription == &feed_sampling_rate) {
        Serial.print(F("*** Sampling rate: "));
        Serial.println((char *)feed_sampling_rate.lastread);
        delay(100);
        new_time = true;
      }
      if (subscription == &feed_update_gps_sub) {
        Serial.print(F("*** GPS: "));
        Serial.println((char *)feed_update_gps_sub.lastread);
        delay(100);
        //new_loc = true;
      }
    }
    delay(3000);
  }

  // find the initial distance that will correspond to sea level
  while (initial_distance <= 23) {
    initial_distance = sonar.ping_cm();
    delay(50);
  }
  initial_distance = initial_distance / 30.48;
  Serial.print("initial distance: "); Serial.print(initial_distance); Serial.println(" ft");
  delay(50);

  // find the initial pressure reading that will correspond to sea level
  float voltage = analogRead(pressurePin);      // convert 10 bit analog reading to voltage
  voltage = voltage * 5 / 1024;
  initial_feet_of_water = voltage * 5.7831 - 2.2404;      // 0.5-4.5V maps to 0-23.1 feet of water
  Serial.print("initial feet of water: "); Serial.print(initial_feet_of_water); Serial.println(" ft");
  delay(50);
}

void loop() {
  // wake up the fona (should be done in the interrupt function, add here too for error handling)
  digitalWrite(FONA_DTR, LOW);

  // connect to cell network
  while (!netStatus()) {
    Serial.println(F("Failed to connect to cell network, retrying..."));
    delay(2000); // Retry every 2s
    digitalWrite(blueLed, LOW);
  }
  Serial.println(F("Connected to cell network!"));
  digitalWrite(blueLed, HIGH);
  readRSSI();

  digitalWrite(yellowLed, HIGH);

  // find the time at which we are logging all this data
  time_t t = RTC.get();

  // take package temperature data
  tempsensor.wake();
  float tempF = tempsensor.readTempF();
  Serial.print("Package temperature: "); Serial.print(tempF); Serial.print("*F\t");
  char tempBuff[6];
  dtostrf(tempF, 1, 2, tempBuff);
  Serial.println("Shutting down the MCP9808...");
  tempsensor.shutdown();                                // In this mode the MCP9808 draws only about 0.1uA

  // take external temperature data
  ds18b20.requestTemperatures();
  float tempEx = ds18b20.getTempCByIndex(0);
  Serial.print("External temperature: "); Serial.print(tempEx); Serial.println("*C");
  char tempExBuff[6];
  dtostrf(tempEx, 1, 2, tempExBuff);

  // take ambient pressure data
  float pressure = mpr.readPressure() / 68.947572932;
  Serial.print(F("Pressure: ")); Serial.print(pressure); Serial.println(F(" psi"));
  char pressBuff[6];
  dtostrf(pressure, 1, 2, pressBuff);

  //take cell connection data
  uint8_t n = fona.getRSSI();
  Serial.print(F("RSSI:")); Serial.println(n);
  char cellBuff[6];
  dtostrf(n, 1, 2, cellBuff);


  // take stage data
  float distance = 0;
  while (distance <= 23) {
    distance = sonar.ping_cm();
    delay(50);
  }
  distance = distance / 30.48;
  Serial.print("Ultrasonic distance: "); Serial.print(distance); Serial.println(" ft");
  distance = sea_level + (initial_distance - distance);
  Serial.print("\tabove sea level: "); Serial.print(distance); Serial.println(" ft");
  char stageBuff[7];
  dtostrf(distance, 1, 2, stageBuff);

  // take stage pressure data
  float voltage = analogRead(pressurePin);                                                     // convert 10 bit analog reading to voltage
  voltage = voltage * 5 / 1023;
  float feet_of_water = voltage * 5.7831 - 2.2404;                                             // 0.5-4.5V maps to 0-23.1 feet of water
  Serial.print("Feet above sensor: "); Serial.print(feet_of_water); Serial.println(" ft");
  feet_of_water = sea_level + (feet_of_water - initial_feet_of_water);
  Serial.print("\tabove sea level: "); Serial.print(feet_of_water); Serial.println(" ft");
  char ptsBuff[7];
  dtostrf(feet_of_water, 1, 2, ptsBuff);

  // take fona lipo battery data
  uint16_t vbat;
  if (! fona.getBattVoltage(&vbat)) {
    Serial.println(F("Failed to read Batt"));
  } else {
    Serial.print(F("VBat = ")); Serial.print(vbat); Serial.println(F(" mV"));
  }
  char lipoBuff[6];
  sprintf(lipoBuff, "%u", vbat);


  // take package power data
  float busvoltage = 0;
  float power_mW = 0;
  busvoltage = ina219.getBusVoltage_V();
  power_mW = ina219.getPower_mW();
  busvoltage = busvoltage + 0.8;      // account for voltage drop across diode
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
  char voltBuff[6];
  char powerBuff[6];
  dtostrf(busvoltage, 1, 2, voltBuff);
  dtostrf(power_mW, 1, 2, powerBuff);

  // take contact sensor data
  int contact = digitalRead(contactPin);
  Serial.print("Contact pin state: "); Serial.println(contact);
  char contactBuff[2];
  itoa(contact, contactBuff, 3);


  Serial.println(F("---------------------"));

  // connect to MQTT
  MQTT_connect();

  // This is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(5000))) {
    // this checks if the time was changed
    if (subscription == &feed_sampling_rate) {
      Serial.print(F("*** Got: "));
      Serial.println((char *)feed_sampling_rate.lastread);
      delay(100);
      new_time = true;
    }
    // this checks if we need to get the gps location
    if (subscription == &feed_update_gps_sub) {
      Serial.print(F("*** GPS: "));
      Serial.println((char *)feed_update_gps_sub.lastread);
      delay(100);
      //new_loc = true;
    }
    // this checks if the deployment switch is ever turned off
    if (subscription == &feed_deploy) {
      Serial.print(F("***Deployed: ")); Serial.print((char *)feed_deploy.lastread);
    }
  }

  // save to SD card
  File file = SD.open("001.csv", FILE_WRITE);
  if (file) {

    // write the time stamp
    file.print(String(month(t)));
    file.print("/");
    file.print(String(day(t)));
    file.print("/");
    file.print(String(year(t)));
    file.print(" ");
    file.print(String(hour(t)));
    file.print(":");
    file.print(String(minute(t)));
    file.print(":");
    file.print(String(second(t)));
    file.print(" ");

    // write the data
    file.print(pressure); file.print(" ");
    file.print(distance); file.print(" ");
    file.print(feet_of_water); file.print(" ");
    file.print(busvoltage); file.print(" ");
    file.println("");
    file.close();
  }
  else {
    Serial.println(F("Unable to open file"));
  }

  // initialize gps variables
  char updateBuff[3];
  float latitude, longitude, speed_kph, heading, altitude;
  char latBuff[12], longBuff[12], locBuff[50], speedBuff[12], headBuff[12], altBuff[12];
  int gps_fails = 0;

  // if we got ON, then find the location
  if (strcmp(feed_update_gps_sub.lastread, "ON") == 0) {

    // enable GPS
    while (!fona.enableGPS(true)) {
      Serial.println(F("Failed to turn on GPS, retrying..."));
      delay(2000); // Retry every 2s
    }
    Serial.println(F("Turned on GPS!"));

    // take gps data
    while (!fona.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude) && gps_fails < 5) {     // go into Adafruit_FONA.h and uncomment //#define MQTT_DEBUG on line 37 for this to work (https://github.com/adafruit/Adafruit_MQTT_Library/issues/54)
      digitalWrite(whiteLed, HIGH);
      Serial.println(F("Failed to get GPS location, retrying..."));
      delay(2000); // Retry every 2s
      gps_fails++;
    }
    if (gps_fails == 5) {
      Serial.println("Giving up on GPS");
    }
    if (gps_fails < 5) {
      digitalWrite(whiteLed, LOW);
      Serial.println(F("Found 'eeeeem!"));
      Serial.println(F("---------------------"));
      Serial.print(F("Latitude: ")); Serial.println(latitude, 6);
      Serial.print(F("Longitude: ")); Serial.println(longitude, 6);
      Serial.print(F("Speed: ")); Serial.println(speed_kph);
      Serial.print(F("Heading: ")); Serial.println(heading);
      Serial.print(F("Altitude: ")); Serial.println(altitude);

      dtostrf(latitude, 1, 6, latBuff); // float_val, min_width, digits_after_decimal, char_buffer
      dtostrf(longitude, 1, 6, longBuff);
      dtostrf(speed_kph, 1, 0, speedBuff);
      dtostrf(heading, 1, 0, headBuff);
      dtostrf(altitude, 1, 1, altBuff);

      sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);
      new_loc = true;
    }
    // turn off GPS
    fona.enableGPS(false);
    // update GPS query status
    gps_fails = 0;
    updateBuff[0] = "OFF";
    MQTT_publish_checkSuccess(feed_update_gps_pub, updateBuff);
  }

  // publish data to Adafruit IO
  if (new_loc == true) {
    MQTT_publish_checkSuccess(feed_location, locBuff);
    new_loc = false;
  }
  MQTT_publish_checkSuccess(feed_temp, tempBuff);
  MQTT_publish_checkSuccess(feed_external_temp, tempExBuff);
  MQTT_publish_checkSuccess(feed_stage, stageBuff);
  MQTT_publish_checkSuccess(feed_pressure, pressBuff);
  MQTT_publish_checkSuccess(feed_pts, ptsBuff);
  MQTT_publish_checkSuccess(feed_fona_lipo, lipoBuff);
  MQTT_publish_checkSuccess(feed_voltage, voltBuff);
  MQTT_publish_checkSuccess(feed_power, powerBuff);
  MQTT_publish_checkSuccess(feed_contact, contactBuff);
  MQTT_publish_checkSuccess(feed_connection, cellBuff);

  // reassign the sampling rate
  if (new_time == true) {
    sampling_rate = atoi((char *)feed_sampling_rate.lastread);
    delay(100);
    Serial.print(F("New sampling rate: ")); Serial.println(sampling_rate);
    new_time = false;   // reset the boolean
  }

  // check if deployment was turned off
  if (strcmp(feed_deploy.lastread, "OFF") == 0) {
    Serial.println(F("Deployment turned off"));
    deployed = false;
  }

  Serial.print(F("Waiting for ")); Serial.print(sampling_rate); Serial.println(F(" minutes\r\n"));
  digitalWrite(yellowLed, LOW);

  int remainder = sampling_rate;

  // check if our sampling rate is slower than the server timeout
  if (sampling_rate > keepAlive_mins) {
    int ping_count = sampling_rate / keepAlive_mins;      // number of times we will have to ping per sampling interval
    Serial.print("Ping count: "); Serial.println(ping_count);
    delay(100);
    remainder = sampling_rate % keepAlive_mins;              // alarm frequency between last ping and next data collection cycle
    Serial.print("Remainder value: "); Serial.println(remainder);
    delay(100);
    if (remainder == 0) {
      ping_count--;
      remainder = keepAlive_mins;
    }
    while (ping_count > 0) {     // only run pinging functions when we do not need to sample data
      pingSleep();
      // ping the MQTT broker
      if (!mqtt.ping()) {
        mqtt.disconnect();
      }
      delay(1000);
      ping_count--;
      Serial.print("Ping count: "); Serial.println(ping_count);
    }
  }
  
  Serial.print("Remaining minutes after pings: "); Serial.println(remainder);

  t = RTC.get();
  delay(100);
  if (remainder == keepAlive_mins) {
    int tolerance = 20;
    // set an alarm for the next data collection cycle after pings
    if (minute(t) < 60 - keepAlive_mins) {
      if (second(t) < tolerance) {
        RTC.setAlarm(ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) + keepAlive_mins - 1, 0, 0);
      }
      else {
        RTC.setAlarm(ALM1_MATCH_MINUTES, 0, minute(t) + keepAlive_mins, 0, 0);
      }
    }
    else {
      if (second(t) < tolerance) {
        RTC.setAlarm(ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) - 60 + keepAlive_mins - 1, 0, 0);
      }
      else {
        RTC.setAlarm(ALM1_MATCH_MINUTES, 0, minute(t) - 60 + keepAlive_mins, 0, 0);
      }
    }
  }
  else {
    if (minute(t) < 60 - remainder) {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, minute(t) + remainder, 0, 0);
    }
    else {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, minute(t) - 60 + remainder, 0, 0);
    }
  }

  RTC.alarm(ALARM_1);

  delay(1000);

  goSleep();
}

void pingSleep() {
  Serial.println("Sleeping until next ping");

  // find the current time
  time_t t = RTC.get();
  delay(10);

  int tolerance = 20;     // how many seconds we want to make sure the ping has before the mqtt broker times out

  // set an alarm for the next ping required
  if (minute(t) < 60 - keepAlive_mins) {
    if (second(t) < tolerance) {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) + keepAlive_mins - 1, 0, 0);
    }
    else {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, minute(t) + keepAlive_mins, 0, 0);
    }
  }
  else {
    if (second(t) < tolerance) {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 60 - tolerance, minute(t) - 60 + keepAlive_mins - 1, 0, 0);
    }
    else {
      RTC.setAlarm(ALM1_MATCH_MINUTES, 0, minute(t) - 60 + keepAlive_mins, 0, 0);
    }
  }

  RTC.alarm(ALARM_1);

  digitalWrite(FONA_DTR, HIGH);
  delay(500);

  sleep_enable();
  attachInterrupt(interrupt, pingWake, LOW);        // the wake up function is set to pingWake
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();
}

void pingWake() {
  Serial.println("Pinging the server...");
  delay(500);
  sleep_disable();
  detachInterrupt(interrupt);                   // clear the interrupt flag

  digitalWrite(FONA_DTR, LOW);
  delay(1000);

  mqtt.ping();
}


void goSleep() {
  Serial.println("Going to sleep...");
  delay(100);

  fona.println("AT+CSCLK=1");
  if (fona.available()) {
    Serial.println(fona.read());
  }

  delay(500);

  digitalWrite(FONA_DTR, HIGH);

  // activate sleep mode, attach interrupt and assign a waking function to run
  sleep_enable();
  attachInterrupt(interrupt, wakeUp, LOW);
  delay(100);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);              // set to full sleep mode
  sleep_cpu();
}

void wakeUp() {
  Serial.println("RTC Interrupt fired");
  delay(100);
  sleep_disable();
  detachInterrupt(interrupt);

  digitalWrite(FONA_DTR, LOW);
  delay(500);

}


void moduleSetup() {
  // this should all take about 3s for the SIMCOM 7000 module
  fonaSS.begin(115200); // Default SIM7000 shield baud rate
  Serial.println(F("Configuring to 9600 baud"));
  fonaSS.println("AT+IPR=9600"); // Set baud rate
  delay(100); // Short pause to let the command run
  fonaSS.begin(9600);
  if (! fona.begin(fonaSS)) {
    Serial.println(F("FONA not found"));
    while (1); // Don't proceed if it couldn't find the device
  }

  uint8_t type = fona.type();
  Serial.println(F("FONA is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L")); break;
    case SIM800H:
      Serial.println(F("SIM800H")); break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)")); break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)")); break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)")); break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)")); break;
    case SIM7000:
      Serial.println(F("SIM7000")); break;
    case SIM7070:
      Serial.println(F("SIM7070")); break;
    case SIM7500:
      Serial.println(F("SIM7500")); break;
    case SIM7600:
      Serial.println(F("SIM7600")); break;
    default:
      Serial.println(F("???")); break;
  }

  // Print module IMEI number.
  uint8_t imeiLen = fona.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: "); Serial.println(imei);
  }
}

bool netStatus() {
  int n = fona.getNetworkStatus();

  Serial.print(F("Network status ")); Serial.print(n); Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));

  if (!(n == 1 || n == 5)) {
    return false;
    digitalWrite(blueLed, LOW);
  }
  else return true;
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    digitalWrite(greenLed, HIGH);
    return;
  }

  Serial.println("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    digitalWrite(greenLed, LOW);
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  digitalWrite(greenLed, HIGH);
  Serial.println("MQTT Connected!");
}

void MQTT_publish_checkSuccess(Adafruit_MQTT_Publish &feed, const char *feedContent) {
  Serial.println(F("Sending data..."));
  uint8_t txfailures = 0;
  if (! feed.publish(feedContent)) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(redLed, HIGH);
      delay(200);
      digitalWrite(redLed, LOW);
    }
    Serial.println(F("Failed"));
    txfailures++;
  }
  else {
    digitalWrite(redLed, LOW);
    Serial.println(F("OK!"));
    txfailures = 0;
  }
}

void readRSSI() {
  uint8_t n = fona.getRSSI();
  int8_t r;

  Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
  if (n == 0) r = -115;
  if (n == 1) r = -111;
  if (n == 31) r = -52;
  if ((n >= 2) && (n <= 30)) {
    r = map(n, 2, 30, -110, -54);
  }
  Serial.print(r); Serial.println(F(" dBm"));
}
