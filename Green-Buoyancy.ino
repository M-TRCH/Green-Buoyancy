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
#define APN  "internet" // leave it as it is, may change according to the service provider
#define USER ""  // leave it empty for AIS, may change according to the service provider
#define PASS ""  // leave it empty for AIS, may change according to the service provider
// Internet parameter
#define mqtt_server   "34.126.110.212" // w.r.t. server setting
#define mqtt_port     "1883"  // fixed port for sending the data package to server
#define mqtt_ID       ""
#define mqtt_user     "greenbuoy" // w.r.t. server setting
#define mqtt_password "1234" // w.r.t. server setting
#define timeout_gpsFix 20000 // TODO: spare enough time to get gps fix. [ms] timeout for getting the gps fix
#define timeout_internet 5000
#define timeout_server 20000 
// Server parameter

GNSS gps;
INTERNET net;
UCxMQTT mqtt;
// UC20 Object
Uart mySerialGSM(&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// NANO 33 Hardware serial object

void setup()
{
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  // Set hardware serial ports
  Serial.begin(9600); 
  gsm.begin(&mySerialGSM, 9600);
  // Default baudrate

}

const unsigned long interval_time = minTomsec(1);
long previous_time = -interval_time;
float count = 0.0f;
// interval active

boolean led_state = 1; // LED Blink
unsigned long sleep_prevTime = millis();
// interval sleep

String longitudePREV = "100.8772571", latitudePREV = "13.1298563";

void loop()
{
 if(millis()-previous_time >= interval_time)
 {
  previous_time = millis();
  GSM_wake(); // UC20 wake up  
  
  Serial.print("GPS Starting");
  String gps_data, longitudeNOW = "", latitudeNOW = "";
  unsigned long gps_time, gps_start = millis();
  bool gpsFix = false;
  while(!gpsFix)
  {
   delay(3000); Serial.print(".");
   gps_data = gps.GetPosition();
   latitudeNOW  = get_LatLon(gps_data, ',', 1);
   longitudeNOW = get_LatLon(gps_data, ',', 2);
   gps_time = millis()-gps_start;

   if(latitudeNOW != "" && longitudeNOW != "")
   {
    gpsFix = true;
    Serial.println("\nGPS Fixed");
   }
   else if(gps_time >= timeout_gpsFix)
   { 
    Serial.println("\nGPS Timeout!");
    break;
   }
  }

  if(!gpsFix)
  {
   latitudeNOW = latitudePREV;
   longitudeNOW = longitudePREV;
  }
  else
  {
   latitudePREV = latitudeNOW;
   longitudePREV = longitudeNOW;
  }
  Serial.println("\nlatitude : " + latitudeNOW);
  Serial.println("longitude : " + longitudeNOW);
  Serial.println("GPS Started\n");

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

  uint8_t max_count = 2; 
  String payload[max_count];
  construct_data_package(latitudeNOW, longitudeNOW, V1, V2, V3, V4, V5, V6, V7, V8, V9, V10, V11, V12, V13, V14, payload);
  // data compile

  boolean internetConnected = internet_connect(timeout_internet);  // TODO: should we use the connection status 
  boolean serverConnected = server_connect(timeout_server);  // TODO: should we use the connection status
  for(uint8_t i=0; i < max_count; i++)
  {
   mqtt.Publish("data", payload[i]); 
   Serial.println("payload[info]: "  + payload[i]); 
   Serial.println(payload[i].length());
   delay(10);
  }
  Serial.println("Published Success");
  // send data

  GSM_sleep(); // UC20 sleep  
 }

 //TODO: need to verify if the delay is necessary, or just let it spin without delay is Okay
 sleep_prevTime = millis();
 while(millis()-sleep_prevTime <= minTomsec(0.2))
 {
   digitalWrite(13, led_state);
   led_state = !led_state;
   delay(2000);
 } // bedtime
}

// Function
void SERCOM0_Handler()
{
  mySerialGSM.IrqHandler();
} 
void GSM_wake()
{
  gsm.PowerOn();          Serial.println("\nPower[status]: on");
  while(gsm.WaitReady()); Serial.println("Power[status]: ready");

  gps.Start(); Serial.println("GPS[status]: start");
  
  Serial.print("Signal quality: "); Serial.println(gsm.SignalQuality());
  Serial.print("Get operator: ");   Serial.println(gsm.GetOperator());
  Serial.println();
}

boolean internet_connect(unsigned long timeout)
{
  Serial.println("Set APN & Password");
  net.Configure(APN, USER, PASS);
  
  boolean connected = false;
  unsigned long start_time = millis();  
  while(!connected) // TODO: include timeout mechanism into this block (to prevent dead lock when the internet connection cannot estrablish)
  {
    Serial.println("Internet connecting"); delay(2000);
    if(net.Connect())
    { 
     Serial.println("Internet connected");
     connected = true;
    }
    else if(millis()-start_time >= timeout)
    { 
     Serial.println("Internet connection timeout!"); 
     break;
    }
  }
  Serial.print("Get IP: "); Serial.println(net.GetIP());
  Serial.println();
  return connected;
}

boolean server_connect(unsigned long timeout)
{
  boolean connected = false;
  unsigned long start_time = millis();  
  while(!connected)  // TODO: include timeout mechanism into this block (to prevent dead lock when the connection cannot estrablish)
  {  
   if(mqtt.DisconnectMQTTServer())
   {
    mqtt.ConnectMQTTServer(mqtt_server, mqtt_port);
   }
   if(mqtt.ConnectState())
   { 
    connected = true;
    Serial.println("Server connected\n");
   }
   else if(millis()-start_time >= timeout)
   { 
    Serial.println("Server timeout!\n"); 
    break;
   }
   Serial.print("Server connect[state]: ");
   Serial.println(connected);
   delay(500);
  } 
  unsigned char re_turn = mqtt.Connect(mqtt_ID, mqtt_user, mqtt_password);
  // Serial.println(mqtt.ConnectReturnCode(re_turn)); 
  return connected;
}

String get_LatLon(String data, char separate, int index)
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

void GSM_sleep()
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

void construct_data_package(String lat, String lon, float V1,  float V2,  float V3,  float V4,  float V5,  float V6,  float V7,
                                float V8, float V9, float V10, float V11, float V12, float V13, float V14, String *pay)
{
  uint8_t DP = 2; // Ex. 9.47

  lat = "{\"lat\": " + lat;
  lon = ",\"lon\": " + lon;
  String _V1  = ",\"V1\":"  + String(V1,  DP);
  String _V2  = ",\"V2\":"  + String(V2,  DP);
  String _V3  = ",\"V3\":"  + String(V3,  DP);
  String _V4  = ",\"V4\":"  + String(V4,  DP);
  String _V5  = ",\"V5\":"  + String(V5,  DP)+"}";
  String _V6  = "{\"V6\":"  + String(V6,  DP);
  String _V7  = ",\"V7\":"  + String(V7,  DP);
  String _V8  = ",\"V8\":"  + String(V8,  DP);
  String _V9  = ",\"V9\":"  + String(V9,  DP);
  String _V10 = ",\"V10\":" + String(V10, DP);
  String _V11 = ",\"V11\":" + String(V11, DP);
  String _V12 = ",\"V12\":" + String(V12, DP);
  String _V13 = ",\"V13\":" + String(V13, DP);
  String _V14 = ",\"V14\":" + String(V14, DP)+"}";
  // convert variable

  // TODO: may need to create more line as the message string gets bigger
  pay[0] = lat + lon + _V1 + _V2 + _V3  + _V4  + _V5;
  pay[1] = _V6 + _V7 + _V8 + _V9 + _V10 + _V11 + _V12 + _V13 + _V14;
}
