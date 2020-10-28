#include <Arduino.h>
#include <wiring_private.h>
// Arduino NANO 33 IoT
#include <TEE_UC20.h>
#include <internet.h>
#include <uc_mqtt.h>
#include <gnss.h>
// UC20 Library
#define APN  "internet"
#define USER ""
#define PASS ""
// Internet parameter
#define mqtt_server   "34.126.110.212"
#define mqtt_port     "1883"
#define mqtt_ID       ""
#define mqtt_user     "greenbuoy"
#define mqtt_password "1234"
// Server parameter

GNSS gps;
INTERNET net;
UCxMQTT mqtt;
// UC20 Object
Uart mySerial(&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// NANO 33 Hardware serial object


void SERCOM0_Handler(); 
void wake();
void internet_connect();
void server_connect();
String get_value(String data, char separate, int index);
void sleep();
int availableMemory(); 
// Prototype functions 


void setup()
{
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  // Set hardware serial ports
  Serial.begin(9600); 
  gsm.begin(&mySerial, 9600);
  // Default baudrate
}


unsigned long previous_time = -60000*90;
const long interval_time = 60000*90;
int count = 0;

void loop()
{
 if(millis()-previous_time >= interval_time)
 {
  wake(); // UC20 wake up  


  Serial.print("GPS Starting");
  String gps_data, longitude, latitude = "";

  unsigned long gps_start = millis();
  do
  {
   Serial.print(".");
   gps_data = gps.GetPosition();
   latitude  = get_value(gps_data, ',', 1 );
   longitude = get_value(gps_data, ',', 2 );
   delay(5000);
  } while(latitude == "" && longitude == "" && millis()-gps_start < 300000); 
  Serial.println("\nlatitude : " + latitude);
  Serial.println("longitude : " + longitude);
  Serial.println("GPS Started\n");
  // GPS setup

  
  int           a = availableMemory();
  uint8_t       b = random(0, 20);
  long          c = random(0, 20);
  unsigned long d = random(0, 20);
  double        e = random(0, 20);
  float         f = random(0, 20);
  short         g = random(0, 20);
  int short     h = random(0, 20);
  unsigned int  i = random(0, 20);
  // define variable 
  String lat = "{\"lat\": 13.8018";
  String lon = ",\"lon\": 100.5493";
  if(latitude != "" && longitude != "")
  {
   lat = "{\"lat\": " + latitude;
   lon = ",\"lon\": " + longitude;
  }
  String val1 = ",\"V1\": " + String(a, DEC);
  String val2 = ",\"V2\": " + String(b, DEC);
  String val3 = ",\"V3\": " + String(c, DEC);
  String val4 = ",\"V4\": " + String(d, DEC);
  String val5 = ",\"V5\": " + String(e, 3);
  String val6 = ",\"V6\": " + String(f, 3);
  String val7 = ",\"V7\": " + String(g, DEC);
  String val8 = ",\"V8\": " + String(h, DEC);
  String val9 = ",\"V9\": " + String(i, DEC)+"}";
  // convert variable  


  String payload = lat + lon + val1 + val2 + val3 
                       + val4 + val5 + val6
                       + val7 + val8 + val9;
  Serial.println(payload);
  
  internet_connect();
  server_connect();
  // 
  mqtt.Publish("data", payload);
  // send data

  sleep(); // UC20 sleep  
  previous_time = millis();
 }
 Serial.print(count); Serial.print(" ");  
 count += 1;
 delay(60000*5);
}
