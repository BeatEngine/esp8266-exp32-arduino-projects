
//WIFI AccessPoint
#include <ESP8266WiFi.h>

//Voice control
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"


//Wifi config
#define ssid "WiFiName"
#define password "wIfiPassword"
#define AP_CHANNEL 1
#define AP_MAX_CON 4
#define AP_HIDDEN false

//IPAddress w_host;
WiFiServer webserver(80);

void setupWebserverAP()
{
  delay(100);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password, AP_CHANNEL, AP_HIDDEN, AP_MAX_CON);
    wifi_station_set_hostname("markise");
    delay(1500);
    webserver.begin();
    delay(100);
}

#define D0 16
#define D1 5
#define D2 4
#define D3 0
#define D4 2
#define D5 14
#define D6 12
#define D7 13
#define D8 15
#define RX 3
#define TX 1

//Voice control
VR myVR(D7,D8);

void setupVoiceControl()
{
  pinMode(D2, OUTPUT);
  digitalWrite(D2, HIGH);
  delay(100);
  myVR.begin(9600);
  delay(100);
  myVR.load(uint8_t(0)); // ausfahren
  myVR.load(uint8_t(1)); // stopp
  myVR.load(uint8_t(2)); // einfahren

  myVR.load(uint8_t(3)); // ausfahren
  myVR.load(uint8_t(4)); // stopp
  myVR.load(uint8_t(5)); // einfahren
}

void setup()
{

    Serial.begin(9600);
    
    Serial.println("Setup!");
    setupWebserverAP();
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
    Serial.println("WiFi ready!");
    setupVoiceControl();
    Serial.println("Voice control ready!");
}

int webRequestAction = 0;

void handleWebserverClient()
{
  WiFiClient webclient = webserver.available();
  if(webclient)
  {
    String requestHeader = "";
    Serial.println("Handle client!");
    int c = 0;
    while(!webclient.available());
    while (webclient.available()) {
      if(c < 255)
      {
        requestHeader += (char)(webclient.read());
        c++;
      }
      else
      {
        webclient.read();
      }
    }
    //Serial.println(requestHeader.substring(0, 15));
    if(requestHeader.startsWith("POST /einfahren"))
    {
      webRequestAction = 3;
    }
    else if(requestHeader.startsWith("POST /ausfahren"))
    {
      webRequestAction = 1;
    }
    else if(requestHeader.startsWith("POST /stop"))
    {
      webRequestAction = 2;
    }
    webclient.flush();
    webclient.stop();
  }
}
int currentAction = 0;

unsigned long topenBegin = 0;
unsigned long tcloseBegin = 0;

unsigned long sumOpenTime = 0;
unsigned long sumCloseTime = 0;

void actionStop()
{
  digitalWrite(D6, LOW);
  digitalWrite(D5, LOW);
  if(currentAction == 1)
  {
    sumOpenTime += millis()-topenBegin;
    topenBegin = 0;
  }
  else if(currentAction == 2)
  {
    sumCloseTime += millis()-tcloseBegin;
    tcloseBegin = 0;
  }
  
  currentAction = 0;
  delay(300);
}

void actionOpen()
{
  actionStop();
  currentAction = 1;
  digitalWrite(D5, HIGH);
  topenBegin = millis();
}

void actionClose()
{
  if(sumOpenTime > 0)
  {
  actionStop();
  currentAction = 2;
  digitalWrite(D6, HIGH);
  tcloseBegin = millis();
  }
}


unsigned long MAX_TIME_OPEN = 40000;
unsigned long MAX_TIME_CLOSE = 41000;


void checkTimeout()
{
  //LIMITS Maximal task duration...
  if(currentAction == 1)
  {
    //OPEN
    unsigned long sum = sumOpenTime + (millis()-topenBegin) - sumCloseTime;
    if(sum >= MAX_TIME_OPEN)
    {
      actionStop();
    }
  }
  else if(currentAction == 2)
  {
    //CLOSE
    unsigned long sum = sumCloseTime + (millis()-tcloseBegin);
    if(sum >= (1000+sumOpenTime))
    {
      actionStop();
      sumOpenTime = 0;
      sumCloseTime = 0;
    }
  }
  
}
uint8_t voiceSerialBuffer[255];
void handleVoiceRequest()
{
  int ret = myVR.recognize(voiceSerialBuffer, 50);
  if(ret>0){
    if(voiceSerialBuffer[1] == 0 || voiceSerialBuffer[1] == 3)
    {
      //Ausfahren
      actionOpen();
    }
    else if(voiceSerialBuffer[1] == 1 || voiceSerialBuffer[1] == 4)
    {
      actionStop();
    }
    else if(voiceSerialBuffer[1] == 2 || voiceSerialBuffer[1] == 5)
    {
      //Einfahren
      actionClose();
    }

    Serial.println("Voice action!");
  }
}


void loop()
{
  handleWebserverClient();
  checkTimeout();
  if(webRequestAction != 0)
  {
    if(webRequestAction == 1)
    {
      //Ausfahren
      actionOpen();
    }
    else if(webRequestAction == 2)
    {
      actionStop();
    }
    else if(webRequestAction == 3)
    {
      //Einfahren
      actionClose();
    }
    webRequestAction = 0;
  }
  else
  {
    handleVoiceRequest();
  }
}
