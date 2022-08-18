

//WIFI AccessPoint
#include <ESP8266WiFi.h>
#define ssid "YOURremoteControl"
#define password "yOurPassword"
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

void setup()
{

    Serial.begin(9600);
    
    Serial.print("Setup!");
    setupWebserverAP();
    pinMode(D5, OUTPUT);
    pinMode(D6, OUTPUT);
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
    unsigned long sum = sumOpenTime + (millis()-topenBegin);
    if(sum >= MAX_TIME_OPEN)
    {
      actionStop();
      sumCloseTime = 0;
    }
  }
  else if(currentAction == 2)
  {
    //CLOSE
    unsigned long sum = sumCloseTime + (millis()-tcloseBegin);
    if(sum >= MAX_TIME_CLOSE | sum >= (1000+sumOpenTime))
    {
      actionStop();
      sumOpenTime = 0;
      sumCloseTime = 0;
    }
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
}
