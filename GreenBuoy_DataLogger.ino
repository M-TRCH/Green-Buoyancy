// TODO: remove verbose to save computational power
#include <MemoryFree.h>  // TODO: remove me: for debugging only

#include <SPI.h>
#include "RTClib.h"
#include <OneWire.h>
#include <Wire.h>
#include <Arduino_LSM6DS3.h>
#include "arduinoFFT.h"
  
// to enable the extra hardware serial ports on the NANO 33 IOT
#include <Arduino.h>
#include "wiring_private.h"
const uint32_t serialWaitTime = 10;  // wait time for the data to get through the serial pipe
const uint32_t serialTimeLoop = 50;  // [ms] for loop timing control of the FFT
Uart mySerialGSM (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0); // : verify me
// Attach the interrupt handler to the SERCOM
void SERCOM0_Handler()  // : verify me
{
  mySerialGSM.IrqHandler();
}
Uart mySerialNANO (&sercom1, 12, 11, SERCOM_RX_PAD_3, UART_TX_PAD_0);
// Attach the interrupt handler to the SERCOM
void SERCOM1_Handler()
{
    mySerialNANO.IrqHandler();
}

RTC_DS3231 rtc;  // for the real-time clock module
bool eventTriggered = true;  // alarm flag to wake the system up and log the data
#define CLOCK_INTERRUPT_PIN 2  // the pin that is connected to SQW

float aX, aY, aZ;

const int analog_nSamples = 40;  // some sensors take this many samples then average to get more stable reading

const int FFT_nSamples = 1024;  //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1000.0/float(serialTimeLoop);
const int FFT_sampling_timeout = serialTimeLoop*FFT_nSamples*1.10;  // [ms] timeout for the range reading loop
double data[FFT_nSamples];  // a buffer for the FFT calculation

#define phSensorPin A0
const float pH_offset = 0.00;            //deviation compensate

#define solarVolSensorPin A1
const float R1 = 30000.0; //
const float R2 = 7500.0; //

//const uint32_t timeLoop = 10000;  // [ms] for loop timing control  TODO, the RTC alarm will be used instead

void waitFor_ms(uint32_t timeWait, bool verboseLocal = false){
  uint32_t timeLoop_Start = millis();

  if(verboseLocal){
    Serial.print("start waiting: ");
    Serial.println(timeLoop_Start);
  }

  while ( (uint32_t)(millis() - timeLoop_Start) < timeWait ) {
//    delay(1);  // [ms]
  }
  if(verboseLocal){
    Serial.print("stop waiting: "); 
    Serial.println((uint32_t)millis());
  }

}

struct FFT_results {
// structured variable to store the peak frequency and peak value from the FFT
   double f_peak;  // frequency of the highest peak
   double a_peak;  // amplitude of the highest peak
};

struct FFT_results computeFFT(double _data[]){
  arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

  struct FFT_results results;
  double vReal[FFT_nSamples];
  double vImag[FFT_nSamples];

  for (uint16_t i = 0; i < FFT_nSamples; i++){
      vReal[i] = _data[i];
      vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  
  FFT.Windowing(vReal, FFT_nSamples, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, FFT_nSamples, FFT_FORWARD); /* Compute FFT */
  double offset_heave = vReal[0]/abs(vReal[0])*sqrt(vReal[0]*vReal[0]+vImag[0]*vImag[0])/FFT_nSamples;
  for (uint16_t i = 0; i < FFT_nSamples; i++)
  {
    vReal[i] = _data[i]-offset_heave;
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }
  FFT.Windowing(vReal, FFT_nSamples, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, FFT_nSamples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, FFT_nSamples); /* Compute magnitudes */
  results.f_peak = FFT.MajorPeak(vReal, FFT_nSamples, samplingFrequency);
  results.a_peak = 2.0/(float)FFT_nSamples*vReal[round(results.f_peak * FFT_nSamples / samplingFrequency)];
  
  return results;
}

int writeStatus;
float temp_box;
float batteryVoltage;
float temp_sea;
float pHValue;
FFT_results results_heave;
FFT_results results_range;
float rms_voltage_gen;
float rms_current_gen;
float rms_voltage_solar;
float rms_current_solar;
  
// EARTH EXTEND CODE
int8_t _ready = -1;
int8_t gpsFix = -1;

void GSM_wait(unsigned long timeout)
{
  Serial.print("\nGSM Waiting");
  unsigned long prevtime = millis();
  while(true)
  {
    delay(3000); Serial.print(".");

    mySerialGSM.write('@');
    mySerialGSM.write('C');
    
    if(mySerialGSM.find('c')) 
    {
      _ready = mySerialGSM.parseInt();
      gpsFix = mySerialGSM.parseInt();
      
      mySerialGSM.read(); // read \n
      mySerialGSM.read(); // read \r
      if(mySerialGSM.find('#')) 
      {
             if(_ready == 1){_ready = 1; Serial.println("\nUC20 : Ready"); }
        else if(_ready == 2){_ready = 0; Serial.println("\nUC20 : Not Ready"); }
             if(gpsFix == 3){gpsFix = 1; Serial.println("GPS : Fixed"); } 
        else if(gpsFix == 4){gpsFix = 0; Serial.println("GPS : Not Fixed"); }  
        break;
      }
      else 
      {
        _ready = -2;
        gpsFix = -2;
      }
    }
    else if(millis()-prevtime >= timeout)
    {
      Serial.println("\nTimeout!");
      _ready = -3;
      gpsFix = -3;
      break;
    }
  }
}

float latitude  = -1.00000f; 
float longitude = -1.00000f;

void get_GPS()
{
  mySerialGSM.write('@');
  mySerialGSM.write('G');

  if(mySerialGSM.find('g')) 
  {
    latitude  = mySerialGSM.parseFloat();
    longitude = mySerialGSM.parseFloat();
  }
  mySerialGSM.read(); // read \n
  mySerialGSM.read(); // read \r

  if(mySerialGSM.find('#')) 
  {
    Serial.println("\nget complete package");
  }else{
    latitude  = -2.00000f; 
    longitude = -2.00000f;
  }
}

#define MAX_DATA 12
float value[MAX_DATA] = {0};

void data_publish()
{
//  Serial.println();
//  for(int i=0; i<MAX_DATA; i++) value[i] = random(1000, 9000)/100.0f;

  /*V1*/ value[0] = writeStatus;  // logger status
//  /*V2*/ value[XX] = GPS status // already on the UNO
  /*V3*/ value[1] = temp_box;
  /*V4*/ value[2] = batteryVoltage;
  /*V5*/ value[3] = freeMemory(); // sRAM
  /*V6*/ value[4] = temp_sea;
  /*V7*/ value[5] = pHValue;
  /*V8*/ value[6] = results_heave.f_peak;
  /*V9*/ value[7] = results_range.a_peak;
  /*V10*/ value[8] = rms_voltage_gen;
  /*V11*/ value[9] = rms_current_gen;
  /*V12*/ value[10] = rms_voltage_solar;
  /*V12*/ value[11] = rms_current_solar;
  
  mySerialGSM.write('@');
  mySerialGSM.write('P');

  if(mySerialGSM.find('p')) 
  {
  
    for(int i=0; i<MAX_DATA; i++)
    {
      mySerialGSM.println(value[i]);
      delay(10);
      Serial.print("data random[V");
      Serial.print(i);
      Serial.print("] : ");
      Serial.println(value[i]); 
    }
    mySerialGSM.write('#');
    Serial.println("send data finished\n");
  }
}
void connectToServer(unsigned long timeout)
{
  mySerialGSM.write('@');
  mySerialGSM.write('S');

  unsigned long prevtime = millis();
  Serial.print(F("\nServer Connecting"));  
  while(1)
  {
    delay(2000);
    Serial.print(F("."));
      
    if(mySerialGSM.find('s')) 
    {
      Serial.println(F("\nInternet and Server Connected"));
      break;
    }
    else if(millis()-prevtime >= timeout)
    {
      Serial.println(F("\nTimeout!")); 
      break;
    }
  }
}
void setup() {

  Serial.begin(9600);
  Wire.setClock(10000);
  waitFor_ms(3000); // give enough time for the USB serial port to get ready
  
  pinMode(phSensorPin,INPUT);
  pinMode(solarVolSensorPin, INPUT);

  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    abort();  // do not proceed because the software should not be working without the reference time
  }

  // Hardware Serial for GSM module (Reassign pins 5 and 6 to SERCOM alt)
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  // Hardware Serial for ArduinoNANO (Reassign pins 12 and 11 to SERCOM alt)
  pinPeripheral(12, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

}

//------------------------------------------------------------------------------
void loop() {
  
  // get the current time from the RTC module
  DateTime now = rtc.now();
          
  // get the current temperature of the electronic box
  temp_box = rtc.getTemperature();

  Serial.println(F(">>>>>>>>>>>>>>>>>>>"));
  Serial.println(F(">>>>>>>>>>>>>>>>>>>"));
  Serial.print(now.year(), DEC);
  Serial.print(F("/"));
  Serial.print(now.month(), DEC);
  Serial.print(F("/"));
  Serial.print(now.day(), DEC);
  Serial.print(F("    "));
  Serial.print(now.hour(), DEC);
  Serial.print(F(":"));
  Serial.print(now.minute(), DEC);
  Serial.print(F(":"));
  Serial.print(now.second(), DEC);
  Serial.println(F(""));

  Serial.print(F("Temperature of the electronic box: "));
  Serial.print(temp_box);
  Serial.println(F(" C"));

  // TODO: take voltage and current reading from the solar cell
  // TODO: average the reading
  rms_voltage_solar = 18.43;
  rms_current_solar = 5.12;

  Serial.print(F("FFT sampling frequency [Hz]: "));
  Serial.println(samplingFrequency);

  Serial.print(F("taking "));
  Serial.print(FFT_nSamples);
  Serial.println(F(" samples from the IMU on 33IOT"));

  int heave_reading_fail = 0;
  if (!IMU.begin()) {
    Serial.println(F("Failed to initialize IMU!"));
    heave_reading_fail = 1;
  }else{
    waitFor_ms(2000); // allow the IMU to come online
    uint32_t heave_sampling_start_time = millis();
    for(int idx_loop = 0; idx_loop<FFT_nSamples; idx_loop++){
      uint32_t serialTimeLoop_start = millis();

      if(millis()-heave_sampling_start_time>FFT_sampling_timeout){
        heave_reading_fail = 3;
        Serial.println("heave reading timeout");
        break;  // break the reading loop
      }
      
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(aX, aY, aZ);
        data[idx_loop] = (float)aZ;
      }else{
        Serial.println("fail to get heave reading");
        heave_reading_fail = 2;
        break;
//        data[idx_loop] = (float)0;
      }

      while ((uint32_t)(millis() - serialTimeLoop_start) < serialTimeLoop) {
        waitFor_ms(1);  // [ms]
      }
    }
    
    Serial.print("Total reading time [ms]: ");
    Serial.println(millis()-heave_sampling_start_time);
 
  }

  if(heave_reading_fail==1){
    Serial.println(F("IMU will not start: generate the dummy data"));
    results_heave.f_peak = -1;
    results_heave.a_peak = -1;
  }else if(heave_reading_fail==2){
    Serial.println(F("heave reading fail: generate the dummy data"));
    results_heave.f_peak = -2;
    results_heave.a_peak = -2;
  }else if(heave_reading_fail==3){
    Serial.println(F("heave reading timeout: generate the dummy data"));
    results_heave.f_peak = -3;
    results_heave.a_peak = -3;
  }else{
    Serial.println(F("heave reading complete: proceed with the FFT"));
    results_heave = computeFFT(data);
  }
  Serial.print("FFT result, heave: ");
  Serial.print(results_heave.f_peak, 6);
  Serial.print(", ");
  Serial.println(results_heave.a_peak, 6);
    

  Serial.println(F("Get the battery voltage reading"));
  float batteryVoltage_reading = 0;
  for (int i=0; i < analog_nSamples; i++) {
    batteryVoltage_reading = batteryVoltage_reading + analogRead(solarVolSensorPin);
    waitFor_ms(50);
  }
  batteryVoltage_reading = batteryVoltage_reading/analog_nSamples*(5.0/1024.0)+0.0; // see text
  batteryVoltage = batteryVoltage_reading / (R2/(R1+R2)) * 1.2;
  Serial.print(F("measured V= "));
  Serial.print(batteryVoltage_reading,2);
  Serial.print(F(":  INPUT V= "));
  Serial.println(batteryVoltage,2);

  float phVoltage = 0;
  for (int i=0; i < analog_nSamples; i++) {
    phVoltage = phVoltage + analogRead(phSensorPin);
    waitFor_ms(50);
  }
  phVoltage = phVoltage/analog_nSamples*(5.0/1024);  // compute the average and convert the analog value to voltage

  pHValue = 3.5*phVoltage+pH_offset;
  Serial.print(F("phVoltage:"));
  Serial.println(phVoltage);
  Serial.print(F("    pH value: "));
  Serial.println(pHValue,2);
  waitFor_ms(500);

  // initiate the connection with the NANO slave that is connected to VL53LOX and INA219
  Serial.print(F("Open the serial port for NANO"));
  mySerialNANO.begin(19200);
  waitFor_ms(3000);
//    if(!mySerialNANO){
//      Serial.println(F("Fail to connect with NANO"));
//    }
//    Serial.println(F("Success"));
  
  // get sea temperature from DS18x20
  temp_sea = 0;
  mySerialNANO.setTimeout(2000);  // spare enough timeout for the DS18x20 to response
  mySerialNANO.write('@');  // send request header
  mySerialNANO.write('T');  // request temp. reading from DS18x20
  waitFor_ms(serialWaitTime);
  if(mySerialNANO.find('t')){
//      Serial.println(F("found the ack of temperature reading"));
    temp_sea = mySerialNANO.parseFloat();
  }
  waitFor_ms(serialTimeLoop);
  Serial.print(F("  Sea temperature = "));
  Serial.print(temp_sea);
  Serial.println(F(" Celsius"));

  // get power reading from INA219 // INFO: take 20ms each request
  float inst_voltage_gen = 0;
  float sum_squared_voltage_gen = 0;
  float inst_current_gen = 0;
  float sum_squared_current_gen = 0;
  
  mySerialNANO.setTimeout(200);  // spare enough timeout for the INA219 to response
  Serial.print(F("taking "));
  Serial.print(FFT_nSamples);
  Serial.println(F(" samples from INA219"));
  
  bool power_reading_fail = 0;
  uint32_t power_sampling_start_time = millis();
  for(int idx_loop = 0; idx_loop<FFT_nSamples; idx_loop++){
    uint32_t serialTimeLoop_start = millis();

    if((uint32_t)(millis()-power_sampling_start_time)>FFT_sampling_timeout){
      power_reading_fail = 1;
      break;  // break the reading loop
    }
    
    mySerialNANO.write('@');  // send request header
    mySerialNANO.write('P');  // request power reading from INA219
    waitFor_ms(serialWaitTime);
    if(mySerialNANO.find('p')){
      inst_voltage_gen = mySerialNANO.parseFloat();
      inst_current_gen = mySerialNANO.parseFloat();
      sum_squared_voltage_gen += inst_voltage_gen * inst_voltage_gen;
      sum_squared_current_gen += inst_current_gen * inst_current_gen;
    }
    waitFor_ms(serialWaitTime);
    
    uint32_t timeElapsed = millis()-serialTimeLoop_start;
    if (timeElapsed>serialTimeLoop){
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F("too long"));
      Serial.print(F("loop time: "));
      Serial.println(timeElapsed);
      Serial.print(F("loop idx: "));
      Serial.println(idx_loop);
      Serial.print(F("Reading vol [V]: "));
      Serial.print(inst_voltage_gen, DEC);
      Serial.print(F("   mAmp [mA]: "));
      Serial.print(inst_current_gen, DEC);
      Serial.println();
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
    }
    
    while ( (uint32_t)(millis() - serialTimeLoop_start) < serialTimeLoop) {
      waitFor_ms(1);  // [ms]
    }
  }
  Serial.print("Total reading time [ms]: ");
  Serial.println((uint32_t)(millis()-power_sampling_start_time));

  if(power_reading_fail){
    Serial.println(F("power reading timeout/fail: generate the dummy data"));
    rms_voltage_gen = 0;
    rms_current_gen = 0;
  }else{
    Serial.println(F("power reading complete: proceed with the FFT"));
    rms_voltage_gen = sqrt(sum_squared_voltage_gen/(float)FFT_nSamples);
    rms_current_gen = sqrt(sum_squared_current_gen/(float)FFT_nSamples);
  }
  
  Serial.print(F("rms voltage [V]: "));
  Serial.print(rms_voltage_gen, DEC); 
  Serial.print(F("   rms current [mA]: "));
  Serial.print(rms_current_gen, DEC);
  Serial.println();
  
  // get range reading from VL53L0X // INFO: take about 40ms each request
  int range = 0;
  mySerialNANO.setTimeout(200);  // spare enough timeout for the VL53L0X to response
  Serial.print(F("taking "));
  Serial.print(FFT_nSamples);
  Serial.println(F(" samples from VL53L0X"));

  bool range_reading_fail = 0;
  uint32_t range_sampling_start_time = millis();
  for(int idx_loop = 0; idx_loop<FFT_nSamples; idx_loop++){
    uint32_t serialTimeLoop_start = millis();

    if((uint32_t)(millis()-range_sampling_start_time)>FFT_sampling_timeout){
      range_reading_fail = 1;
      break;  // break the reading loop
    }
    
    mySerialNANO.write('@');  // send request header
    mySerialNANO.write('R');  // request range reading from VL53L0X
    waitFor_ms(serialWaitTime);
    if(mySerialNANO.find('r')){
      range = mySerialNANO.parseInt();
      data[idx_loop] = range;
    }
    waitFor_ms(serialWaitTime);

    uint32_t timeElapsed = millis()-serialTimeLoop_start;
    if (timeElapsed>serialTimeLoop){
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F("too long"));
      Serial.print(F("loop time: "));
      Serial.println(timeElapsed);
      Serial.print(F("loop idx: "));
      Serial.println(idx_loop);
      Serial.print(F("range sensor reading [mm]: "));
      Serial.print(range, DEC);
      Serial.println();
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
      Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>>>"));
    }

    while ( (uint32_t)(millis() - serialTimeLoop_start) < serialTimeLoop) {
      waitFor_ms(1);  // [ms]
    }
  }

  Serial.print("Total reading time [ms]: ");
  Serial.println((uint32_t)(millis()-range_sampling_start_time));
  
  Serial.print(F("range sensor reading [mm]: "));
  Serial.print(range, DEC);
  Serial.println();

  if(range_reading_fail){
    Serial.println(F("range reading timeout/fail: generate the dummy data"));
    results_range.f_peak = -1;
    results_range.a_peak = -1;
  }else{
    Serial.println(F("range reading complete: proceed with the FFT"));
    results_range = computeFFT(data);
  }
  Serial.print("FFT result, range: ");
  Serial.print(results_range.f_peak, 6);
  Serial.print(", ");
  Serial.println(results_range.a_peak, 6);


  /////////////////////////////////////////////////////////
  //
  //
  //
  mySerialGSM.begin(9600);
  GSM_wait(120000);
  if(gpsFix)
  {
    get_GPS();
    Serial.print(F("latitude: "));  
    Serial.println(latitude);
    Serial.print(F("longitude: "));
    Serial.println(longitude);
  }
  //
  //
  //
  /////////////////////////////////////////////////////////


  // send data to NANO and write on the SD card
  mySerialNANO.setTimeout(2000);  // spare enough timeout for the DS18x20 to response
  mySerialNANO.write('@');  // send request header
  mySerialNANO.write('S');  // request temp. reading from DS18x20

//  mySerialNANO.println(now.year(), DEC);
//  mySerialNANO.println(now.month(), DEC);
//  mySerialNANO.println(now.day(), DEC);
//  mySerialNANO.println(now.hour(), DEC);
//  mySerialNANO.println(now.minute(), DEC);
//  mySerialNANO.println(temp_box,2);
//  mySerialNANO.println(results_heave.f_peak,4);
//  mySerialNANO.println(results_heave.a_peak,4);
//  mySerialNANO.println(batteryVoltage,2);
//  mySerialNANO.println(pHValue,2);
//  mySerialNANO.println(temp_sea,2);
//  mySerialNANO.println(rms_voltage_gen,4);
//  mySerialNANO.println(rms_current_gen,4);
//  mySerialNANO.println(results_range.f_peak,4);
//  mySerialNANO.println(results_range.a_peak,4);

  //TODO: verify the data to be upload
  // time stamp
  mySerialNANO.println(now.year(), DEC);
  mySerialNANO.println(now.month(), DEC);
  mySerialNANO.println(now.day(), DEC);
  mySerialNANO.println(now.hour(), DEC);
  mySerialNANO.println(now.minute(), DEC);

  // system status
  // TODO: insert the logger status here
  mySerialNANO.println(gpsFix, DEC);
  mySerialNANO.println(_ready, DEC);

  // electronic
  mySerialNANO.println(temp_box,2);
  mySerialNANO.println(batteryVoltage,2);
  mySerialNANO.println(freeMemory(), DEC);  // print how much RAM is available.

  // environment
  mySerialNANO.println(temp_sea,2);
  mySerialNANO.println(pHValue,2);
  mySerialNANO.println(results_heave.f_peak,4);

  // solar energy  // TODO: this has to be taken from the solar panels
  mySerialNANO.println(rms_voltage_solar,4);
  mySerialNANO.println(rms_current_solar,4);

  // wave energy
  mySerialNANO.println(rms_voltage_gen,4);
  mySerialNANO.println(rms_current_gen,4);
  mySerialNANO.println(results_range.a_peak,4);

  mySerialNANO.write('#');

  waitFor_ms(serialWaitTime);

  writeStatus = 0;
  writeStatus = mySerialNANO.parseInt();
  waitFor_ms(serialTimeLoop);
  Serial.print("sd card writing status: [");
  Serial.print(writeStatus);
  Serial.print("] ");
  if(writeStatus==1){
    Serial.println("working fine");  
  }else if(writeStatus==0){
    Serial.println("no ack received");  
  }else if(writeStatus==-1){
    Serial.println("sd card error");  
  }else if(writeStatus==-2){
    Serial.println("no checksum on NANO");  
  }else{
    Serial.println("unknow");
  }
  mySerialNANO.end();



  
  /////////////////////////////////////////////////////////
  //
  //
  //
  connectToServer(90000);  // [ms] timeout in 
  data_publish();
  
  mySerialGSM.end();
  delay(20000);  // ensure the server receive the data
  //
  //
  //
  /////////////////////////////////////////////////////////




  // TODO: remove me: for debugging only
  Serial.print(F("Here is how much free memory you have left:  "));
  Serial.println(freeMemory(), DEC);  // print how much RAM is available.
  Serial.println(F(">>>>>>>>>>>>>>>>>>>>>>"));
  Serial.println(F(""));
  
  while(true);
}
