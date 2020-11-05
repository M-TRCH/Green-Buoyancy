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


unsigned long previous_time = -60000*2;
const long interval_time = 60000*2;
int count = 0;

void loop()
{
 if(millis()-previous_time >= interval_time)
 {
  wake(); // UC20 wake up  


  Serial.print("GPS Starting");
  String gps_data, longitude, latitude = "";

  unsigned long current_time, gps_start = millis();
  do
  {
   delay(5000);
   Serial.print(".");
   gps_data = gps.GetPosition();
   latitude  = get_value(gps_data, ',', 1 );
   longitude = get_value(gps_data, ',', 2 );
   current_time = millis()-gps_start;
  } while(latitude == "" && longitude == "" && current_time < 3000); 
  Serial.println("\nlatitude : " + latitude);
  Serial.println("longitude : " + longitude);
  Serial.println("GPS Started\n");
  // GPS setup

  
  float         a = float(millis())/1000.0f;
  uint16_t      b = count/2;
  long          c = current_time/1000;
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
  String val1 = ",\"V1\": " + String(a, 4);
  String val2 = ",\"V2\": " + String(b, DEC);
  String val3 = ",\"V3\": " + String(c, DEC);
  String val4 = ",\"V4\": " + String(d, DEC);
  String val5 = ",\"V5\": " + String(e, 1);
  String val6 = ",\"V6\": " + String(f, 1);
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
 delay(30000);
}

void SERCOM0_Handler()
{
  mySerial.IrqHandler();
} 
void wake()
{
  gsm.PowerOn();          Serial.println("Power[status]: on");
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
  do
  {
     Serial.println(F("Server connecting"));
     if(mqtt.DisconnectMQTTServer())
     {
        mqtt.ConnectMQTTServer(mqtt_server, mqtt_port);
     }
     connect = mqtt.ConnectState();
     Serial.print("Server connect[state]: ");
     Serial.println(connect);
     delay(2000);
  } while(!connect); 
  
  Serial.println("Server connected");
  unsigned char re_turn = mqtt.Connect(mqtt_ID, mqtt_user, mqtt_password);
  Serial.println(mqtt.ConnectReturnCode(re_turn)); 
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
  net.DisConnect(); Serial.println("Server disconnected");
  delay(1000);
  gsm.PowerOff();   Serial.println("Power[status]: off");
  Serial.println();
}
void Publish2(String topic , String payload)
{
  char chartopic[topic.length()+2];
  char charpay[payload.length()+2];
  unsigned char i=0;
  for(i=0;i<topic.length();i++)
  {
    chartopic[i] = topic[i];
  }
  chartopic[i]=0;
  
  for(i=0;i<payload.length();i++)
  {
    charpay[i] = payload[i];
  }
  charpay[i]=0;
  
  mqtt.Publish(chartopic,topic.length(),charpay,payload.length());
}
int availableMemory() 
{
    // Use 1024 with ATmega168
    int size = 2048;
    byte *buf;
    while ((buf = (byte *) malloc(--size)) == NULL);
        free(buf);
    return size;
}