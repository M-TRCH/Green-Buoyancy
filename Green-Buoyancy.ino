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
   delay(random(3000,5000));
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

  
  float a = float(millis())/60000.0f;
  float b = count;
  float c = float(gps_time)/1000.0f;
  float d = freeMemory();
  float e = random(1000, 9000)/100.0f;
  float f = random(1000, 9000)/100.0f;
  float g = random(1000, 9000)/100.0f;
  float h = random(1000, 9000)/100.0f;
  float i = random(1000, 9000)/100.0f;
  // define variable

  String lat = "{\"lat\":13.8019";
  String lon = ",\"lon\":100.5548";
  if(latitude != "" && longitude != "")
  {
   lat = "{\"lat\": " + latitude;
   lon = ",\"lon\": " + longitude;
  }
  String val1 = ",\"V1\":" + String(a, 2);
  String val2 = ",\"V2\":" + String(b, 2);
  String val3 = ",\"V3\":" + String(c, 2);
  String val4 = ",\"V4\":" + String(d, 2)+"}";
  String val5 = "{\"V5\":" + String(e, 2);
  String val6 = ",\"V6\":" + String(f, 2);
  String val7 = ",\"V7\":" + String(g, 2);
  String val8 = ",\"V8\":" + String(h, 2);
  String val9 = ",\"V9\":" + String(i, 2)+"}";
  // convert variable  


  String payload = lat + lon + val1 + val2 + val3 + val4;
  String payload2 = val5 + val6 + val7 + val8 + val9;
  Serial.println(payload);
  Serial.println(payload2);
  Serial.println(payload.length());
  Serial.println(payload2.length());
  
  internet_connect(); server_connect();
  // 
  mqtt.Publish("data", payload); delay(10);
  mqtt.Publish("data2", payload2);
  
  Serial.println("Published Success");
  // send data


  sleep(); // UC20 sleep  
 }
 count += 0.5; Serial.print(count); Serial.print(" ");  
 
 sleep_prevTime = millis();
 while(millis()-sleep_prevTime <= minTomsec(0.5))
 {
  if(millis()-led_prevTime >= 100)
  {
   led_prevTime = millis();
   digitalWrite(13, led_state);
   led_state = !led_state;
  }
 }
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