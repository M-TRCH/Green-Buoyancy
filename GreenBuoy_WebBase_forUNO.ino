//-------- Library --------//
#include <TEE_UC20.h>
#include <SoftwareSerial.h>
#include <internet.h>
#include <uc_mqtt.h>
#include <gnss.h>
#include <MemoryFree.h>
//-------- Object ---------//
INTERNET  net;
UCxMQTT   mqtt;
GNSS      gps;
SoftwareSerial gsmSerial(8, 9);
SoftwareSerial serviceSerial(10, 11);

//------------- monitor setting --------------//
#define GSM_DEBUG_MODE
#define BAUD_RATE 9600
//-------- Internet service parameter --------//
#define APN     "internet"
#define USER    ""
#define PASS    ""
//------------- Server parameter -------------//
#define MQTT_SERVER     "34.126.110.212"
#define MQTT_PORT       "1883"
#define MQTT_ID         ""
#define MQTT_USER       "greenbuoy"
#define MQTT_PASSWORD   "1234"
#define MQTT_TOPIC      "data"
//------------- timeout setting --------------//
#define timeout_powerOn       45000
#define timeout_ready         30000
#define timeout_internet      15000
#define delayTime_internet    3000
#define timeout_server        15000
#define delayTime_server      500
#define timeout_gps           50000
#define delayTime_gps         2000
//--------- state of sequence ---------// 
unsigned long prevTime = millis();
boolean prev_success = false;
//----------- gps variable -----------//
float latitude = -1.00000f, longitude = -1.00000f;  
uint8_t gpsFix = 0; 
//-------- web based variable --------//
#define VIRTUAL_VALUE_MAX   12 
#define GPS_DP              5
#define VALUE_DP            2   
#define BUF_MAX             10
char buffer[BUF_MAX]; 
String payload = "";
float sum, checksum;
float V[VIRTUAL_VALUE_MAX] = {-1};
const char V1[]  PROGMEM = "{\"V1\":";
const char V3[]  PROGMEM = "{\"V3\":";
const char V4[]  PROGMEM = "{\"V4\":";
const char V5[]  PROGMEM = "{\"V5\":";
const char V6[]  PROGMEM = "{\"V6\":";
const char V7[]  PROGMEM = "{\"V7\":";
const char V8[]  PROGMEM = "{\"V8\":";
const char V9[]  PROGMEM = "{\"V9\":";
const char V10[] PROGMEM = "{\"V10\":";
const char V11[] PROGMEM = "{\"V11\":";
const char V12[] PROGMEM = "{\"V12\":";
const char V13[] PROGMEM = "{\"V13\":";
const char LAT[] PROGMEM = "{\"lat\":";
const char LON[] PROGMEM = "{\"lon\":";
const char FIX[] PROGMEM = "{\"V2\":";
const char *const obj_table[] PROGMEM = {V1,  V3,  V4,  V5,  V6,  V7,  V8, 
                                         V9, V10, V11, V12, V13, LAT, LON, FIX};

//------------- function -------------//
#ifdef GSM_DEBUG_MODE
void debug(String data)
{
  Serial.println(data);
}
void data_out(char data)
{
  Serial.write(data);
}
#else
#endif
void callback(String topic ,char *payload,unsigned char length)
{
  Serial.println();
  Serial.println(F("%%%%%%%%%%%%%%%%%%%%%%%%%%%%"));
  Serial.print(F("Topic --> "));
  Serial.println(topic);
  payload[length]=0;
  String str_data(payload);
  Serial.print(F("Payload --> "));
  Serial.println(str_data);
}
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++)
  {
    if(data.charAt(i)==separator || i==maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
void gsm_wakeup(unsigned long pw_timeout, unsigned long rd_timeout)
{
  Serial.println(F("GSM state : power on"));
  
  prevTime = millis(); prev_success = false;
  while(!prev_success)
  {
    if(gsm.PowerOn())
    {
      prev_success = true;
      Serial.println(F("          : done"));
      break;
    }
    else if(millis()-prevTime >= pw_timeout)
    {
      Serial.println(F("          : timeout"));
      break;
    } 
  }
  
  if(prev_success)
  {
    Serial.println(F("GSM state : ready"));
    
    prevTime = millis(); prev_success = false;
    while(!prev_success)
    {
      if(gsm.WaitReady())
      {
        prev_success = true;
        Serial.println(F("          : done"));
        break;
      }
      else if(millis()-prevTime >= rd_timeout)
      {
        Serial.println(F("          : timeout"));
        break;
      }
    }   
  }

  Serial.println(F("GPS state : start"));
  gps.Start();
  
}
void internet_connect(unsigned long net_timeout, unsigned long delayTime)
{
  if(prev_success)
  {
    Serial.print(F("GetOperator --> ")); Serial.println(gsm.GetOperator());
    Serial.print(F("SignalQuality --> ")); Serial.println(gsm.SignalQuality()); 
    Serial.println(F("Disconnect net")); net.DisConnect(); 
    Serial.println(F("Set APN and Password")); net.Configure(APN,USER,PASS); 
  
    Serial.println(F("Internet state : connecting"));
    
    prevTime = millis(); prev_success = false;
    while(!prev_success)
    {
      delay(delayTime);
      if(net.Connect())
      {
        prev_success = true;
        Serial.println(F("               : connected"));
        break;
      }
      else if(millis()-prevTime >= net_timeout)
      {
        Serial.println(F("               : timeout"));
        break;
      }
    }
    Serial.println(F("Show My IP")); Serial.println(net.GetIP());   
  } 
}
void server_connect(unsigned long sv_timeout, unsigned long delayTime)
{
  if(prev_success)
  {
    Serial.println(F("Server state : connecting"));
    
    prevTime = millis(); prev_success = false;
    while(!prev_success)
    {
      if(mqtt.DisconnectMQTTServer())
      {
        mqtt.ConnectMQTTServer(MQTT_SERVER, MQTT_PORT);
      }
      delay(delayTime);
      if(mqtt.ConnectState())
      {
        prev_success = true;
        Serial.println(F("             : connected"));
        break;          
      }
      else if(millis()-prevTime >= sv_timeout)
      {
        Serial.println(F("             : timeout"));
        break;
      }
    }
    unsigned char ret = mqtt.Connect(MQTT_ID,MQTT_USER,MQTT_PASSWORD);
    Serial.println(mqtt.ConnectReturnCode(ret));
  }
}
void getGPS(unsigned long gps_timeout, unsigned long delayTime, float *float_lat, float *float_lon)
{
  String string_lat = "", string_lon = "";
        
  if(prev_success)
  {
    Serial.println(F("GPS state : get value"));

    prevTime = millis(); prev_success = false;
    while(!prev_success)
    {
      delay(delayTime);
      
      String GPS_DATA = gps.GetPosition(); Serial.println(GPS_DATA);
      string_lat = getValue(GPS_DATA, ',', 1);
      string_lon = getValue(GPS_DATA, ',', 2);
      
      if(string_lon != "" && string_lon != "")
      {
        prev_success = true;
        gpsFix = 1;
        Serial.println(F("          : fixed"));
        *float_lat = string_lat.toFloat();
        *float_lon = string_lon.toFloat();
        break;
      }
      else if(millis()-prevTime >= gps_timeout)
      {
        prev_success = true;
        Serial.println(F("          : timeout"));
        if(gpsFix == 0)
        {
          Serial.println(F("          : not fixed"));
        }
        break;
      }
    }
  }
}
void data_publish(String topic, uint8_t index, float value, uint8_t DEC_point)
{
  // clear buffer 
  payload = ""; 
  for(int i=0; i<BUF_MAX; i++){ buffer[i] = ""; }

  strcpy_P(buffer, (char*)pgm_read_word(&(obj_table[index])));   
  payload = String(buffer) + String(value, DEC_point) + "}";
  mqtt.Publish(topic, payload);  
  delay(1);
}
void data_publish(String topic, uint8_t index, uint8_t value)
{
  // clear buffer 
  payload = ""; 
  for(int i=0; i<BUF_MAX; i++){ buffer[i] = ""; }

  strcpy_P(buffer, (char*)pgm_read_word(&(obj_table[index])));   
  payload = String(buffer) + String(value, DEC) + "}";
  mqtt.Publish(topic, payload);  
  delay(1);
}
void send_state_to_nano33()
{
  serviceSerial.write('C');
  uint8_t val_A = 0, val_B = 0;
  if(prev_success){ val_A = 1; } 
  else            { val_A = 2; }
  if(gpsFix)      { val_B = 3; }
  else            { val_B = 4; }
  serviceSerial.println(val_A);
  serviceSerial.println(val_B);
  serviceSerial.println(val_A+val_B);
  serviceSerial.write('#');
}
void send_gps_data_to_nano33()
{
  serviceSerial.write('G');
  serviceSerial.println(latitude);
  serviceSerial.println(longitude);
  serviceSerial.println(latitude+longitude);
  serviceSerial.write('#');
}
void receive_data_from_nano33()
{ 
  //////////////////////////////////////////////////
  sum = 0, checksum = 0;
  
  for(int i=0; i<VIRTUAL_VALUE_MAX; i++)
  {
    V[i] = serviceSerial.parseFloat();
    sum += V[i];
    Serial.print("data receive[V");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(V[i]);
  }
  checksum = serviceSerial.parseFloat();
  
  serviceSerial.read(); // read \n
  serviceSerial.read(); // read \r

  if(serviceSerial.find('#')) 
  {
    if((int)sum == (int)checksum) 
    {
      prev_success = true; 
      Serial.println("\nget complete package");
    }
    else
    { 
      prev_success = false;
      for(int i=0; i<VIRTUAL_VALUE_MAX; i++)
      {
        V[i] = -1;
      }
    }
  } 
  //////////////////////////////////////////////////
  if(prev_success)
  {
    if(!mqtt.ConnectState()) server_connect(timeout_server, delayTime_server);
    prev_success = true;
    
    serviceSerial.end();
    gsm.begin(&gsmSerial, BAUD_RATE);
    
    //////////////// package publish ///////////////
    for(int i=0; i<VIRTUAL_VALUE_MAX; i++)
    {
      data_publish(MQTT_TOPIC, i, V[i], VALUE_DP);
      delay(1);
    }
    data_publish(MQTT_TOPIC, 12, latitude , GPS_DP); 
    data_publish(MQTT_TOPIC, 13, longitude, GPS_DP);
    data_publish(MQTT_TOPIC, 14, gpsFix);
    Serial.println(F("\npublished.")); 
    ////////////////////////////////////////////////   
    
    gsmSerial.end();
    serviceSerial.begin(BAUD_RATE);
  }
}
void setup() 
{
  Serial.begin(BAUD_RATE);          // hardware serial begin
  gsm.begin(&gsmSerial, BAUD_RATE); // software serial begin(gsm comunication)

  #ifdef GSM_DEBUG_MODE
  gsm.Event_debug = debug;
  mqtt.callback = callback;
  #else
  #endif  
  
  // while(!Serial); // wait open serial monitor 
  delay(2000);
  Serial.println(F("Serial state : begin"));
  Serial.print(F("FreeMemory : "));
  Serial.print(freeMemory());  
  Serial.println(F(" KB"));  
  delay(2000);
  
  gsm_wakeup(timeout_powerOn, timeout_ready);                  // gsm wake up 
  getGPS(timeout_gps, delayTime_gps, &latitude, &longitude);   // get gps within 50 s
  internet_connect(timeout_internet, delayTime_internet);      // wait internet conecting
  server_connect(timeout_server, delayTime_server);            // wait mqtt server conecting

  gsmSerial.end();           // software serial end(gsm comunication)
  serviceSerial.begin(BAUD_RATE); // software serial begin(nano33 comunication)
}
void loop() 
{
  if(serviceSerial.find('@')) 
  {
    delay(5);
    int ID = serviceSerial.read();
      
         if(ID == 'C'){ send_state_to_nano33();     } // state check 
    else if(ID == 'G'){ send_gps_data_to_nano33();  } // gps data request
    else if(ID == 'P'){ receive_data_from_nano33(); } // push data request
  }   
}
