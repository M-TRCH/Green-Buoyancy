// TODO: set serial verbose to false in order to reduce computational power
// TODO: record everyday instead of every hour
// TODO: ensure sufficient decimal value is used when saving the data
// TODO: verify the header agrees with the data field

//#include <MemoryFree.h>  // TODO: remove me: for debugging only

#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <VL53L0X.h>
#include <Adafruit_INA219.h>
#include "SdFat.h"

#include <SoftwareSerial.h>
SoftwareSerial mySerial(9, 8); // RX, TX

bool verbose = true;  // print info. to help in the debugging process

VL53L0X sensorRange;  // for range finder

Adafruit_INA219 ina219;

OneWire oneWire(7);
DallasTemperature sensorSeaTemp(&oneWire);

SdFat sd;  // SD card object
const uint8_t chipSelect = 10;  // SD chip select pin
SdFile file;  // Log file.

int VL53L0X_readingError = 1;
int INA219_readingError = 1;

int record_year;
int record_month;
int record_day;
int record_hour;
int record_minute;
float temp_box;
float heave_f_peak;
float heave_a_peak;
float batteryVoltage;
float pHValue;
float temp_sea;
float range_f_peak;
float range_a_peak;
float rms_voltage_gen;
float rms_current_gen;

void waitFor_ms(uint32_t timeWait){
  uint32_t timeLoop_Start = millis();

  while ( (uint32_t)(millis() - timeLoop_Start) < timeWait ) {
//    delay(1);  // [ms]
  }

}

void loggingData() {

  String log_date = String(record_year, DEC) + \
                    ":" + String(record_month, DEC) + \
                    ":" + String(record_day, DEC);
      
  String log_time = String(record_hour, DEC) + \
                    ":" + String(record_minute, DEC);
                    
  if(verbose){
    Serial.print("log date: ");
    Serial.println(log_date);
    Serial.print("log time: ");
    Serial.println(log_time);    
  }

  file.print(log_date);
  file.print(F(","));
  file.print(log_time);
  file.print(F(","));
  file.print(temp_box);
  file.print(F(","));
  file.print(heave_f_peak);
  file.print(F(","));
  file.print(heave_a_peak);
  file.print(F(","));
  file.print(batteryVoltage);
  file.print(F(","));
  file.print(pHValue);
  file.print(F(","));
  file.print(temp_sea);
  file.print(F(","));
  file.print(rms_voltage_gen);
  file.print(F(","));
  file.print(rms_current_gen);
  file.print(F(","));
  file.print(range_f_peak);
  file.print(F(","));
  file.print(range_a_peak);
  file.println();
}

void writeHeader() {
  // TODO: modify the header according to the data structure
  file.print(F("time"));
  file.print(F(","));
  file.print(F("data1"));
  file.println();
}

void setup() {

  if(verbose){
    Serial.begin(9600);    
  }

  waitFor_ms(1000);

  if(verbose){
    Serial.print("Opening the serial port: ");
  }

  mySerial.begin(19200);
  if(!mySerial){
    if(verbose){
      Serial.println("Fail");
    }
  }
  if(verbose){
    Serial.println("Succees");
  }
  
  Wire.begin();

  sensorSeaTemp.begin();

  if (!sensorRange.init())
  {
    VL53L0X_readingError = 1;
  }else{
    VL53L0X_readingError = 0;
  }

  if (!ina219.begin())
  {
    INA219_readingError = 1;
  }else{
    INA219_readingError = 0;
  }
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensorRange.setMeasurementTimingBudget(20000);
  sensorRange.setTimeout(40);
}

void loop() {
 
//  Serial.println("hello from Slave:");

  if (mySerial.find('@')) {      // If the data header is found
//    Serial.println("serial is available:");
    waitFor_ms(5);  // spare enough time for the data ID to arrive
    int _ID = mySerial.read();

    if(_ID == 'R'){
      if(verbose){
        Serial.println("request range flag is found:");
      }
      int range = 0;
      if (VL53L0X_readingError){
//        Serial.println("Fail to find VL53LOX");
        range = -1;
      }else{
//        Serial.println("range is measured");
        range = sensorRange.readRangeSingleMillimeters();
        if (sensorRange.timeoutOccurred()) { range = -2; }
      }

      mySerial.write('r');  // send a charactor as a header of the reply
      mySerial.println(range);
      if(verbose){
        Serial.println(range);
      }
           
    }else if(_ID == 'P'){
      if(verbose){
        Serial.println("request power flag is found:");
      }
      float busvoltage_V = 26.123;
      float current_mA = 31.456;
      if (INA219_readingError) {
//        Serial.println(F("Failed to find INA219 chip"));
      } else{
        // TODO
        busvoltage_V = ina219.getBusVoltage_V();
        current_mA = ina219.getCurrent_mA();
      }

      mySerial.write('p');
      mySerial.println(busvoltage_V);
      mySerial.println(current_mA);
      if(verbose){
        Serial.println(busvoltage_V);
        Serial.println(current_mA);
      }

    }else if (_ID == 'T'){
      if(verbose){
        Serial.println("request temperature flag is found:");
      }

      sensorSeaTemp.requestTemperatures(); // Send the command to get temperatures
      float celsius = sensorSeaTemp.getTempCByIndex(0);

      // if the sensor does not response, the reading will be -127
      if(celsius<0){  
        celsius = -1;  // in that case, set the reading to -1
      }

      mySerial.write('t');
      mySerial.println(celsius);
      if(verbose){
        Serial.println(celsius);
      }
    }else if(_ID == 'S'){
      if(verbose){
        Serial.println("request logging flag is found:");
      }
      record_year = mySerial.parseInt();
      record_month = mySerial.parseInt();
      record_day = mySerial.parseInt();
      record_hour = mySerial.parseInt();
      record_minute = mySerial.parseInt();
      temp_box = mySerial.parseFloat();
      heave_f_peak = mySerial.parseFloat();
      heave_a_peak = mySerial.parseFloat();
      batteryVoltage = mySerial.parseFloat();
      pHValue = mySerial.parseFloat();
      temp_sea = mySerial.parseFloat();
      rms_voltage_gen = mySerial.parseFloat();
      rms_current_gen = mySerial.parseFloat();
      range_f_peak = mySerial.parseFloat();
      range_a_peak = mySerial.parseFloat();
      if(verbose){
        Serial.println(temp_box,2);
        Serial.println(heave_f_peak,4);
        Serial.println(heave_a_peak,4);
        Serial.println(batteryVoltage,2);
        Serial.println(pHValue,2);
        Serial.println(temp_sea,2);
        Serial.println(rms_voltage_gen,4);
        Serial.println(rms_current_gen,4);
        Serial.println(range_f_peak,4);
        Serial.println(range_a_peak,4);
      }
      waitFor_ms(10);  // spare enough time for the checksum to arrive
      mySerial.read();  // read NL that came before the checksum
      mySerial.read();  // read CR that came before the checksum

      bool checksumError = 0;
      bool sdError = 0;
      
      if(mySerial.read()=='#'){
        if(verbose){
          Serial.println("checksum is found");
        }
        checksumError = 0;
      }else{
        if(verbose){
          Serial.println("checksum is not found");
        }
        checksumError = 1;
      }

      if(verbose){
        Serial.print(record_year, DEC);
        Serial.print(F("/"));
        Serial.print(record_month, DEC);
        Serial.print(F("/"));
        Serial.print(record_day, DEC);
        Serial.print(F("    "));
        Serial.print(record_hour, DEC);
        Serial.print(F(":"));
        Serial.print(record_minute, DEC);
        Serial.println(F(""));
      }
      
      if(!checksumError){
        String file_name = "";          // file name should not prefix with "prefix_word"
        String baseName = "LOG";
        file_name = baseName;
        file_name = baseName + "_" + String(record_year, DEC) + \
                    "_" + String(record_month, DEC) + \
                    "_" + String(record_day, DEC) + \
                    "_" + String(record_hour, DEC) + \
                    ".csv";
  
        int charLen = file_name.length()+1;
        char fileName[charLen];
        file_name.toCharArray(fileName, charLen);

        if(verbose){
          Serial.println(F("Loading the SD card"));
        }
        if (!sd.begin(chipSelect, SD_SCK_MHZ(5))) {
          // can detect both module fail or the sd card fail
          if(verbose){
            Serial.println(F("Fail to connect to the sd card"));
          }
          sdError = 1;
        }else{
          if(verbose){
           Serial.println(F("Initialised the sd card"));
          }
          sdError = 0;
          if (sd.exists(fileName)) {
            if(verbose){
              Serial.println(F("File already exist"));
              Serial.println(fileName);
            }
  
            file.open(fileName, O_WRONLY | O_APPEND);
            loggingData();
            file.close();
  
          }else{
            if(verbose){
              Serial.println(F("Create new file"));
              Serial.println(fileName);
            }
  
            file.open(fileName, O_WRONLY | O_CREAT);
            writeHeader();
            loggingData();
            file.close();
          }
        }
      }

      if(checksumError){
        mySerial.println(-2);
      }else if(sdError){
        mySerial.println(-1);
      }else{
        mySerial.println(1);
      }

    }

  }
}
