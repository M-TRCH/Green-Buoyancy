#include <Arduino.h>
#include <wiring_private.h>
#include <MemoryFree.h>
#include <pgmStrToRAM.h>
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
unsigned long minTomsec(float _min);
void data_package(String lat, String lon, float V1,  float V2,  float V3,  float V4,  float V5,  float V6,  float V7,
                                float V8, float V9, float V10, float V11, float V12, float V13, float V14, String *pay);
// Prototype functions 

void setup()
{
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  // Set hardware serial ports
  Serial.begin(9600); 
  gsm.begin(&mySerial, 9600);
  // Default baudrate

  pinMode(13, OUTPUT);
  // Set LED pin
}


const unsigned long interval_time = minTomsec(30);
long previous_time = -interval_time;
float count = 0.0f;
// interval active
unsigned long led_prevTime = millis();
boolean led_state = 1;
// LED Blink
unsigned long sleep_prevTime = millis();
// interval sleep


void loop()
{
 if(millis()-previous_time >= interval_time)
 {
  previous_time = millis();
  wake(); // UC20 wake up  


  Serial.print("GPS Starting");
  String gps_data, longitude, latitude = "";
  unsigned long gps_time, gps_start = millis();
  do
  {
   delay(random(3000, 9000));
   Serial.print(".");
   gps_data = gps.GetPosition();
   latitude  = get_value(gps_data, ',', 1 );
   longitude = get_value(gps_data, ',', 2 );
   gps_time = millis()-gps_start;
  } while(latitude == "" && longitude == "" && gps_time < 3000); 
  Serial.println("\nlatitude : " + latitude);
  Serial.println("longitude : " + longitude);
  Serial.println("GPS Started\n");
  // GPS setup

  
  String lat = "13.8019";
  String lon = "100.5548"; // โรงจอดรถไฟฟ้า BTS
  if(latitude != "" && longitude != ""){ lat = latitude; lon = longitude; }  
  float V1 = float(millis())/60000.0f;
  float V2 = count;
  float V3 = float(gps_time)/1000.0f;
  float V4 = freeMemory();
  float V5  = random(1000,  3000)/100.0f;
  float V6  = random(   0,  1400)/100.0f;
  float V7  = random(   0, 50000)/100.0f;
  float V8  = random(   0,   150)/100.0f;
  float V9  = random(   0,  2200)/100.0f;
  float V10 = random(   0,  1000)/100.0f;
  float V11 = random(   0,  3000)/100.0f;
  float V12 = random(   0,  1000)/100.0f;
  float V13 = random( 800,  1600)/100.0f;
  float V14 = random(   0,  1000)/100.0f;
  // define variable


  String payload[2];
  data_package(lat, lon, V1, V2, V3, V4, V5, V6, V7, V8, V9, V10, V11, V12, V13, V14, payload);
  Serial.println("payload(0)[info]: "  + payload[0]); Serial.println(payload[0].length());
  Serial.println("payload(1)[info]: "  + payload[1]); Serial.println(payload[1].length());
  // data compile


  internet_connect(); server_connect();
  // 
  mqtt.Publish("data" , payload[0]); delay(10);
  mqtt.Publish("data2", payload[1]);
  Serial.println("Published Success");
  // send data

  sleep(); // UC20 sleep  
 }

 
 count += 0.1; Serial.print(count); Serial.print(" ");  
 
 sleep_prevTime = millis();
 while(millis()-sleep_prevTime <= minTomsec(0.1))
 {
  if(millis()-led_prevTime >= 100)
  {
   led_prevTime = millis();
   digitalWrite(13, led_state);
   led_state = !led_state;
  }
 } // bedtime
}

// Function
void SERCOM0_Handler()
{
  mySerial.IrqHandler();
} 
void wake()
{
  gsm.PowerOn();          Serial.println("\nPower[status]: on");
  while(gsm.WaitReady()); Serial.println("Power[status]: ready");

  gps.Start(); Serial.println("GPS[status]: start");
  
  Serial.print("Signal quality: "); Serial.println(gsm.SignalQuality());
  Serial.print("Get operator: ");   Serial.println(gsm.GetOperator());
  Serial.println();
}
void internet_connect()
{
  Serial.println("Set APN & Password");
  net.Configure(APN, USER, PASS);

  boolean connect = false;  
  do
  {
    Serial.println("Internet connecting"); 
    connect = net.Connect();
  } while(!connect);
  
  Serial.println("Internet connected");
  Serial.print("Get IP: "); Serial.println(net.GetIP());
  Serial.println();
}
void server_connect()
{
  boolean connect = false;
  Serial.println(F("Server connecting"));
  do
  {  
     if(mqtt.DisconnectMQTTServer())
     {
        mqtt.ConnectMQTTServer(mqtt_server, mqtt_port);
     }
     connect = mqtt.ConnectState();
     Serial.print("Server connect[state]: ");
     Serial.println(connect);
     delay(500);
  } while(!connect); 
  
  Serial.println("Server connected");
  unsigned char re_turn = mqtt.Connect(mqtt_ID, mqtt_user, mqtt_password);
  //Serial.println(mqtt.ConnectReturnCode(re_turn)); 
  Serial.println();
}
String get_value(String data, char separate, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i=0; i<=maxIndex && found<=index; i++)
  {
    if (data.charAt(i) == separate || i == maxIndex) 
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void sleep()
{
  mqtt.clear_buffer();
  net.DisConnect(); Serial.println("Server disconnected");
  gsm.PowerOff();   Serial.println("Power[status]: off");
  Serial.println();
}
unsigned long minTomsec(float _min)
{
  return _min*60000;  
}
void data_package(String lat, String lon, float V1,  float V2,  float V3,  float V4,  float V5,  float V6,  float V7,
                                float V8, float V9, float V10, float V11, float V12, float V13, float V14, String *pay)
{
  uint8_t DP = 2; // Ex. 9.47

  lat = "{\"lat\": " + lat;
  lon = ",\"lon\": " + lon;
  String _V1  = ",\"V1\":"  + String(V1,  DP);
  String _V2  = ",\"V2\":"  + String(V2,  DP);
  String _V3  = ",\"V3\":"  + String(V3,  DP);
  String _V4  = ",\"V4\":"  + String(V4,  DP);
  String _V5  = ",\"V5\":"  + String(V5,  DP);
  String _V6  = ",\"V6\":"  + String(V6,  DP)+"}";
  String _V7  = "{\"V7\":"  + String(V7,  DP);
  String _V8  = ",\"V8\":"  + String(V8,  DP);
  String _V9  = ",\"V9\":"  + String(V9,  DP);
  String _V10 = ",\"V10\":" + String(V10, DP);
  String _V11 = ",\"V11\":" + String(V11, DP);
  String _V12 = ",\"V12\":" + String(V12, DP);
  String _V13 = ",\"V13\":" + String(V13, DP);
  String _V14 = ",\"V14\":" + String(V14, DP)+"}";
  // convert variable  
  
  pay[0] = lat + lon + _V1 + _V2 + _V3 + _V4 + _V5 + _V6;
  pay[1] = _V7 + _V8 + _V9 + _V10 + _V11 + _V12 + _V13 + _V14;
}