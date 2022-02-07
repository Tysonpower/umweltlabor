/*
 * ESP32 environmental laboratory for IGHamspirit by Manuel DO5TY
 *  
 *  Libraries:
 *  * Adafruit - IO, Unified Sensors, BMP085, BME280, CCS811
 *  * Pawel Kolodziejczyk - Nova SDS011
 *  * Nick O'Leary - PubSubClient
 *  * Dirk - EspSoftwareSerial
 *  
 *  And Board definitions for ESP32 of course :)
 *  
 *  Enter your WIFI Credentials, change the Station Name as well as your MQTT Server Credentials and MQTT_DEVICE Name
 *  Also select if you use a BMP/BME280 or BMP180 sensor  
 * 
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include "SdsDustSensor.h"
#include <PubSubClient.h>
#include "WiFi.h"

#define STATION_HEIGHT 65                       // height over sea height in meters (used for pressure correction)

#define USE_BME280 true
#define STATION_NAME "do5ty"                    // your Callsign in lowercase

#define WIFI_SSID "myWifi"                      // AP SSID
#define WIFI_PASSWORD "goodPassword"            // AP Password
#define MQTT_SERVER "uwlab.tynet.eu"            // MQTT Server
#define MQTT_USER "do5ty"                       // MQTT User     (if any)
#define MQTT_PASS "mymqttpass"                  // MQTT Password (if any)
#define MQTT_DEVICE "DO5TY"                     // MQTT Device ID - Your Callsign in uppercase

//------------------------------------------------------------------------------------------
#define TOPIC_BASE "sensors/environment/"STATION_NAME
#define TOPIC_TEMP TOPIC_BASE"/temperature"
#define TOPIC_HUM TOPIC_BASE"/humidity"
#define TOPIC_PRES TOPIC_BASE"/pressure"
#define TOPIC_BRIGHT TOPIC_BASE"/brightness"
#define TOPIC_CO2 TOPIC_BASE"/co2"
#define TOPIC_TVOC TOPIC_BASE"/tvoc"
#define TOPIC_PM10 TOPIC_BASE"/pm10"
#define TOPIC_PM25 TOPIC_BASE"/pm25"
#define TOPIC_GEIGER TOPIC_BASE"/geiger"

#define TEMT_PIN 33         // TEMT6000 PIN for Analog Read, needs to be on ! ADC1 !
#define TEMT_ADJ 700        // TEMT adjust value that the value is reduced by
#define MEASURE_DELAY 60    // Time between Measurements
#define SDS_ON_TIME 5       // Time the Sensor Runs for at a time

#define GEIGER_CALCBASE 123.14709
#define GEIGER_PIN 13
#define GEIGER_PERIOD 60000 

Adafruit_BME280 bme;        // BME280 I2C
Adafruit_BMP085 bmp;        // BMP180 I2C
Adafruit_CCS811 ccs;        // CCS811 I2C

SdsDustSensor sds(Serial2); // passing HardwareSerial& as parameter

WiFiClient wifiClient;
PubSubClient client(wifiClient);
int status = WL_IDLE_STATUS;

unsigned long counts;
unsigned long previousMillis; //variable for measuring time

// sensor vars
float bmx_temp, bmx_hum, bmx_pres, geiger_usv, sds_pm10, sds_pm25;
int temtValue, ccs_co2, ccs_tvoc;

void impulse() { // dipanggil setiap ada sinyal FALLING di pin 2
  counts++;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Umweltlabor Projekt - IG Hamspirit");
  Serial.print("Measure Delay: ");
  Serial.println(MEASURE_DELAY);
  
  Serial.print("Connecting to WIFI...");
  Serial.println(WIFI_SSID);

  InitWiFi();
  //WIFi OK, connect to MQTT server
  client.setServer( MQTT_SERVER, 1883 );
  if ( !client.connected() ) {
    reconnect();
  }

  if(USE_BME280){
    if(bme.begin(0x76, &Wire)){
      Serial.println("BME280 Started...");
    }  
  } else {
    if (bmp.begin()) {
      Serial.println("BMP180 Started...");
    }
  }

  if(ccs.begin()){
    Serial.println("CCS811 Started...");
  }

  sds.begin();
  // loop until sds is connected correctly to avoid burnup of the laser when not running in query mode
  while(sds.queryFirmwareVersion().toString() == "Firmware version [year.month.day]: -1.-1.-1"){
    Serial.println("no connection to SDS Sensor...retry in 5s..");
    delay(5000);
  }
  Serial.println(sds.queryFirmwareVersion().toString()); // prints firmware version
  Serial.println(sds.setQueryReportingMode().toString()); // ensures sensor is in 'query' reporting mode 

  // init Geiger Interrupt
  pinMode(GEIGER_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(GEIGER_PIN), impulse, FALLING);
  
}

void loop() {
    unsigned long currentMillis = millis();
    
    
    if ( !client.connected() ) {
      reconnect();
    }
    
    if (currentMillis - previousMillis > MEASURE_DELAY*1000) {
      //start sds011 and wait on time, then read and let it sleep
      Serial.println("SDS Wakeup..");
      sds.wakeup();
      delay(SDS_ON_TIME*1000);
  
      PmResult pm = sds.queryPm();
      if (pm.isOk()) {
        sds_pm10 = pm.pm10;
        sds_pm25 = pm.pm25;
        Serial.println(pm.toString());
      } else {
        Serial.print("Could not read values from sensor, reason: ");
        Serial.println(pm.statusToString());
      }
      //sds back to sleep
      WorkingStateResult state = sds.sleep();
      if (state.isWorking()) {
        Serial.println("Problem with sleeping the sensor.");
      } else {
        Serial.println("SDS Sleeping..");
      }
    
      //Read BMP or BME Sensor
      if(USE_BME280){
        bmx_temp = bme.readTemperature();
        bmx_pres = bme.readPressure() + ((STATION_HEIGHT/8)*100);
        bmx_hum = bme.readHumidity();
        
        Serial.print("Temperature = ");
        Serial.print(bmx_temp);
        Serial.println(" °C");
    
        Serial.print("Pressure = ");
        Serial.print(bmx_pres / 100.0F);
        Serial.println(" hPa");
      
        Serial.print("Humidity = ");
        Serial.print(bmx_hum);
        Serial.println(" %");
      } else {
        bmx_temp = bmp.readTemperature();
        bmx_pres = bmp.readPressure() + ((STATION_HEIGHT/8)*100);
        
        Serial.print("Temperature = ");
        Serial.print(bmx_temp);
        Serial.println(" °C");
    
        Serial.print("Pressure = ");
        Serial.print(bmx_pres / 100.0F);
        Serial.println(" hPa");
      }
  
      // Read TEMT6000 Sensor
      // ADC2 control register restoring
      temtValue = analogRead(TEMT_PIN);
      temtValue = map(temtValue, TEMT_ADJ, 4096, 0, 1000);
      Serial.print("Brightness = ");
      Serial.print(temtValue);
      Serial.println(" lux");
  
      // Read CCS811
      if(ccs.available()){
        if(!ccs.readData()){
          ccs_co2 = ccs.geteCO2();
          ccs_tvoc = ccs.getTVOC();
          
          Serial.print("CO2: ");
          Serial.print(ccs_co2);
          Serial.println(" ppm");
          Serial.print("TVOC: ");
          Serial.print(ccs_tvoc);
          Serial.println(" ppb");
        }
      }

      geiger_usv = counts/GEIGER_CALCBASE;
      Serial.print("Geiger: ");
      Serial.print(geiger_usv);  
      Serial.print(" uSv");
      counts = 0;    
      Serial.println();
      previousMillis = currentMillis;

      //send data
      sendDataMqtt();
    }
    
    client.loop();    //keep connected, look for messages from server
}

void sendDataMqtt(){
  //buld influx datastring

  String prefixString = "environment,site="+String(STATION_NAME)+" value=";

  char attributes[100];
  String payload = "";
  // Send Temperature
  payload = prefixString + String(bmx_temp);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_TEMP, attributes );

  // Send Humidity
  payload = prefixString + String(bmx_hum);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_HUM, attributes );


  // Send Pressure
  payload = prefixString + String(bmx_pres);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_PRES, attributes );

  // Send Brightness
  payload = prefixString + String(temtValue);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_BRIGHT, attributes );

  // Send CO2
  payload = prefixString + String(ccs_co2);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_CO2, attributes );

  // Send TVOC
  payload = prefixString + String(ccs_tvoc);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_TVOC, attributes );

  // Send PM10
  payload = prefixString + String(sds_pm10);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_PM10, attributes );

  // Send PM25
  payload = prefixString + String(sds_pm25);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_PM25, attributes );

  // Send GEIGER
  payload = prefixString + String(geiger_usv);
  memset(attributes,0,sizeof(attributes));
  payload.toCharArray( attributes, 100 );
  client.publish( TOPIC_GEIGER, attributes );
}

void InitWiFi()
{
  Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("Connected to AP");
    }
    Serial.print("Connecting to broker ...");
    // Attempt to connect (clientId, username, password)
    if ( client.connect(MQTT_DEVICE, MQTT_USER, MQTT_PASS) ) {
      Serial.println( "[DONE]" );
      Serial.println(client.connected());
    } else {
      Serial.print( "[FAILED] [ rc = " );
      Serial.print( client.state() );
      Serial.println( " : retrying in 5 seconds]" );
      // Wait 5 seconds before retrying
      delay( 5000 );
    }
    delay(1000);
  }
}
