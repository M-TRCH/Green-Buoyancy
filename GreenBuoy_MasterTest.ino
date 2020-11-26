#include <SoftwareSerial.h>
#include <MemoryFree.h>

SoftwareSerial mySerial(10, 11);
/////////////////////////////////////////////
// gps data
float latitude  = -1.00000f,
      longitude = -1.00000f;
// state data
uint8_t _ready = 0, gpsFix = 0;
// dummy data
#define MAX_DATA 12
float V[MAX_DATA];
float sum, checksum;
/////////////////////////////////////////////

void getState()
{
  Serial.print("\nWait");
  while(true)
  {
    delay(3000); Serial.print(".");

    sum = 0, checksum = 0;
    mySerial.write('@');
    mySerial.write('C');
    
    if(mySerial.find('C')) 
    {
      _ready = mySerial.parseInt();
      gpsFix = mySerial.parseInt();
      sum    = mySerial.parseInt();
   
      checksum = _ready+gpsFix;
    }
    mySerial.read(); // read \n
    mySerial.read(); // read \r
    
    if(mySerial.find('#')) 
    {
      if((int)sum == (int)checksum)
      {
        Serial.println("\nget complete package");
        
             if(_ready == 1)  Serial.println("UC20 : Ready"); 
        else if(_ready == 2)  Serial.println("UC20 : Not Ready"); 
             if(gpsFix == 3)  Serial.println("GPS : Fixed"); 
        else if(gpsFix == 4)  Serial.println("GPS : Not Fixed");  
        break;
      }
      else 
      {
        _ready = 0;
        gpsFix = 0;
      }
    }
  }
}
void getGPS()
{
  sum = 0, checksum = 0;
  
  mySerial.write('@');
  mySerial.write('G');

  if(mySerial.find('G')) 
  {
    latitude  = mySerial.parseFloat();
    longitude = mySerial.parseFloat();
    sum = mySerial.parseFloat();

    checksum = latitude+longitude;
  }
  mySerial.read(); // read \n
  mySerial.read(); // read \r

  if(mySerial.find('#')) 
  {
    if(sum == checksum)
    {
      Serial.println("\nget complete package");
    }
    else
    {
      latitude  = -1;
      longitude = -1;
    }
  }
}
void sendData()
{
  float dummy;
  sum = 0, checksum = 0;
  for(int i=0; i<MAX_DATA; i++)
  {
    dummy = random(1000, 9000)/100.0f;
    V[i] = dummy;
    sum += V[i]; 
  } // data random

  mySerial.write('@');
  mySerial.write('P');
  
  for(int i=0; i<MAX_DATA; i++)
  {
    mySerial.println(V[i]);
    
    Serial.print("data random[V");
    Serial.print(i);
    Serial.print("] : ");
    Serial.println(V[i]); 
  }
  mySerial.println(sum);
  mySerial.write('#');
  Serial.println("\nsend data finished\n");
}
void setup() 
{
  Serial.begin(9600);   
  mySerial.begin(9600); 
  
//  while(!Serial); // wait open serial monitor 
  delay(2000);
  Serial.println(F("Serial state : begin"));
  Serial.print(F("FreeMemory : "));
  Serial.print(freeMemory());  
  Serial.println(F(" KB"));  
  delay(2000);

  getState();
  if(gpsFix == 3)
  {
    getGPS();
    Serial.print("latitude: ");  Serial.println(latitude); 
    Serial.print("longitude: "); Serial.println(longitude);
    Serial.println(); delay(3000);
  }
}

const unsigned long mqtt_interval = 600000;
unsigned long mqtt_previous = -mqtt_interval;

void loop() 
{
  if(millis()-mqtt_previous >= mqtt_interval)
  {
      mqtt_previous = millis();
      sendData();   
  }
}
