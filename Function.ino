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
