




#define GPIO5   5   // GPIO 5 (often used for Flash)

// LEFT SIDE TOP-DOWN (Cam is top)
//5V
//GND
#define GPIO12  12  // GPIO 12 (MTDI)
#define GPIO13  13  // GPIO 13 (MTCK)
#define GPIO15  15  // GPIO 15 (MTDO)
#define GPIO14  14  // GPIO 14 (MTMS)
#define GPIO2   2   // GPIO 2 (often used for boot mode, LED)
#define GPIO4   4   // GPIO 4

//RIGHT SIDE TOP-DOWN
#define GPIO16  16  // GPIO 16 (RXD1 - Serial Receive)
#define GPIO0   0   // GPIO 0 (often used for boot mode)
//GND
//3.3V
#define GPIO3   3   // GPIO 3 (RXD0 - Serial Receive)
#define GPIO1   1   // GPIO 1 (TXD0 - Serial Transmit)
//GND

#define GPIO17  17  // GPIO 17 (TXD1 - Serial Transmit)
#define GPIO18  18  // GPIO 18
#define GPIO19  19  // GPIO 19
#define GPIO21  21  // GPIO 21 (SDA - I2C Data)
#define GPIO22  22  // GPIO 22 (SCL - I2C Clock)
#define GPIO23  23  // GPIO 23
#define GPIO25  25  // GPIO 25 (often used for 5V)
#define GPIO26  26  // GPIO 26
#define GPIO27  27  // GPIO 27
#define GPIO32  32  // GPIO 32
#define GPIO33  33  // GPIO 33
#define GPIO34  34  // GPIO 34 (Input only)
#define GPIO35  35  // GPIO 35 (Input only)
#define GPIO36  36  // GPIO 36 (Input only)
#define GPIO39  39  // GPIO 39 (Input only)



// ESP32 V4
//LEFT SIDE
//3V
//EN
#define VP 36
#define VN 39
#define P34 34
#define P35 35
#define P32 32
#define P33 33
#define P25 25
#define P26 26
#define P27 27
#define P14 14
#define P12 12
//GND
#define P13 13
#define D2 9
#define D3 10
#define CMD 11
//5V


//RIGHT SIDE
//GND
#define P23 23
#define P22 22
#define TX 1
#define RX 3
#define P21 21
//GND
#define P19 19
#define P18 18
#define P5 5
#define P17 17
#define P16 16
#define P4 4
#define P0 0
#define P2 2
#define P15 15
#define D1 8
#define D0 7
#define CLK 6

#define P3 3
#define P1 1
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <ArduinoOTA.h>

#define led 2

const char ssid[] = "RoverDavid";        // your network SSID (name)
const char pass[] = "roverdavid";    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;  

unsigned long driveTimeout = millis();

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

int status = WL_IDLE_STATUS;

// InfraredDistance
VL53L0X infraredFront;


void notifyClients() {
 
}

void splitString(const String &str, char delimiter, String* arrOut) {
  int startIndex = 0;
  int delimiterIndex = str.indexOf(delimiter);
  int arrLength = 0;

  // Loop through the string and split by the delimiter
  while (delimiterIndex != -1) {
    arrOut[arrLength] = str.substring(startIndex, delimiterIndex);
    arrLength++;
    startIndex = delimiterIndex + 1;
    delimiterIndex = str.indexOf(delimiter, startIndex);
  }
  
  // Add the last part after the last delimiter
  arrOut[arrLength] = str.substring(startIndex);
}

inline float smooth(float min, float x)
{
  return (1.0-min)*x*x;
}
int lastX = 0;
int lastY = 0;
// PINS 19,18 TL TR    5,17 BL BR
void controlMotorByVector(int x, int y)
{
  lastX = x;
  lastY = y;
  if(y >= 0)
  {
    analogWrite(P0, 0);
    analogWrite(P2, 0);
    analogWrite(P16, 0);
    analogWrite(P4, 0);
    float front = (float)(y) / 100.0f;
    front = smooth(0.0, front) * 255.0f;
    int tl,tr,bl,br;
    float rot = (float)(x) / 100.0f;
    if(x <= 0)
    {
      rot = smooth(0.1, -rot);
      tl = (int)(front - front * rot);
      br = (int)(front - front * rot);
      bl = (int)(front);
      tr = (int)(front);
    }
    else
    {
      rot = smooth(0.1, rot);
      tr = (int)(front - front * rot);
      bl = (int)(front - front * rot);
      br = (int)(front);
      tl = (int)(front);
    }
    analogWrite(P5, bl);
    analogWrite(P17, br);
    analogWrite(P19, tl);
    analogWrite(P18, tr);
  }
  else // Backward
  {
    analogWrite(P5, 0);
    analogWrite(P17, 0);
    analogWrite(P19, 0);
    analogWrite(P18, 0);
    // PINS 16,4 RTL RTR    0,2 RBL RBR
    float front = (float)(y) / 100.0f;
    front = smooth(0.0, front) * 255.0f;
    int rtl,rtr,rbl,rbr;
    float rot = (float)(x) / 100.0f;
    if(x <= 0)
    {
      rot = smooth(0.1, -rot);
      rtl = (int)(front - front * rot);
      rbr = (int)(front - front * rot);
      rbl = (int)(front);
      rtr = (int)(front);
    }
    else
    {
      rot = smooth(0.1, rot);
      rtr = (int)(front - front * rot);
      rbl = (int)(front - front * rot);
      rbr = (int)(front);
      rtl = (int)(front);
    }
    analogWrite(P0, rbl);
    analogWrite(P2, rbr);
    analogWrite(P16, rtl);
    analogWrite(P4, rtr);
  }
}

void stopDriving()
{
    analogWrite(P5, 0);
    analogWrite(P17, 0);
    analogWrite(P19, 0);
    analogWrite(P18, 0);

    analogWrite(P0, 0);
    analogWrite(P2, 0);
    analogWrite(P16, 0);
    analogWrite(P4, 0);
}

bool sensorEnabled = false;

void connectInfrared() {
  //Serial.println("--- INFRARED I2C START ---");
  
  // Power up sensor pin if used as a power source
  pinMode(14, OUTPUT);
  digitalWrite(14, HIGH);
  delay(10); // Give sensor more time to wake up

  // Initialize Wire ONCE. No while loop.
  Wire.begin(26, 27);
  Wire.setClock(10000);
  Wire.setTimeOut(200);
  
  // Use a timeout-protected initialization
  // Most VL53L0X libraries return a boolean on init() or begin()
  if (infraredFront.init()) {
    Serial.println("Infrared Sensor Found!");
    infraredFront.setMeasurementTimingBudget(20000);
    sensorEnabled = true;
  } else {
    Serial.println("!!! SENSOR NOT FOUND - CONTINUING WITHOUT IT !!!");
    sensorEnabled = false;
  }
}

void disconnectInfrared() {
  infraredFront.stopContinuous();
  delay(10); // Give sensor more time to wake up
  Wire.end();
  delay(10); // Give sensor more time to wake up
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);
  delay(10); // Give sensor more time to wake up
}
int lastd = -1;
unsigned long senstime = 0;
int getDistanceSafe(int sensortimeoutmilli) {
  if (!sensorEnabled) return -1;

  if(lastd != -1 && millis() - senstime < sensortimeoutmilli)
  {
    return lastd;
  }

  yield(); // This resets the watchdog and processes Wi-Fi
  //WiFi.setTxPower(WIFI_POWER_8_5dBm);
  //delay(50);
  //connectInfrared();  
  //delay(50);
  senstime = millis();
  int val = infraredFront.readRangeSingleMillimeters();
  lastd = val;
  //delay(200);
  //disconnectInfrared();
  //WiFi.setTxPower(WIFI_POWER_15dBm);
  //delay(300);
  return val;
}
bool anticollision = false;
bool checkFrontDistanceSensor(int x = lastX, int y = lastY)
{
  int d = getDistanceSafe(200);
  if (d == -1) {
    Serial.println("I2C Error: Sensor timed out!");
    return false; 
  }
  if(y > 0 && d < 250 && d>0)
  {
    anticollision = true;
    Serial.print("Barriere: "); Serial.println(d);
    int l = lastY;
    int yn = (int)(100.0*((d / 125.0) - 1.0));
    if(d < 126)
    {
      yn = -100;
    }
    controlMotorByVector(x,yn);
    lastY = l;
    return true;
  }
  else if(anticollision)
  {
    controlMotorByVector(0,0);
    anticollision = false;
  }
  return false;
}

bool checkTimeout()
{
  if(millis()-driveTimeout >= 1100)
  {
    stopDriving();
    return true;
  }
  return false;
}

void keepAliveTimeout()
{
    driveTimeout = millis();
}

void processJoystick(String msg)
{
  // websocket.send('joystick:' + x + '|' + y + '|' + Joy3.GetPosDir() + '|' + Joy3.GetX() + '|' + Joy3.GetY());
  String list = msg.substring(9);
  String values[6];

  splitString(list, '|', values);

  //split(list, '|', values); // posx, posy, dirName, vecX, vecY
  int x = values[3].toInt();
  int y = values[4].toInt();

  if(x > 0 || y > 0)
  {
    x = (int)((x+30)/1.3); // motor minimal move force
    y = (int)((y+30)/1.3); // motor minimal move force
  }
  //Serial.println("Control motor");
  if(!checkFrontDistanceSensor(x, y))
  {
    controlMotorByVector(x, y);
  }
}
const String KEEP_ALIVE = String("keep-alive");
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    char tmp[len+1];
    strncpy(tmp, (char*)data, len);
    tmp[len] = 0;
    String string_msg(tmp);
    //Serial.println(string_msg);
    // websocket.send('joystick:' + x + '|' + y + '|' + Joy3.GetPosDir() + '|' + Joy3.GetX() + '|' + Joy3.GetY());
    if (string_msg.startsWith("joystick:")) {
        processJoystick(string_msg);
        keepAliveTimeout();
    }
    else if(KEEP_ALIVE == string_msg)
    {
      keepAliveTimeout();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}
 

const String index_html = "<!DOCTYPE HTML><html> \
<head> \
  <title>ESP Web Server</title> \
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"> \
  <link rel=\"icon\" href=\"data:,\"> \
<style> \
* \
{ \
	box-sizing: border-box; \
} \
body \
{ \
	margin: 0px; \
	padding: 0px; \
	font-family: monospace; \
} \
.row \
{ \
	display: inline-flex; \
	clear: both; \
} \
.columnLateral \
{ \
  float: left; \
  width: 15%; \
  min-width: 300px; \
} \
.columnCetral \
{ \
  float: left; \
  width: 70%; \
  min-width: 300px; \
} \
</style> \
 \
<title>ESP Rover</title> \
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"> \
<link rel=\"icon\" href=\"data:,\"> \
</head> \
<body> \
 <img style=\"top: 10px;width: 100%;\" src=\"/stream\"></img> \
  <div id=\"joy3Div\" style=\"width:200px;height:200px;margin:50px;position:fixed;bottom:30px;left:150px;\"></div> \
		<div style=\"position:fixed;bottom:125px;left:750px; visibility: collapse;\"> \
			Posizione X:<input id=\"joy3PosizioneX\" type=\"text\" style=\"visibility: collapse;\" /><br /> \
			Posizione Y:<input id=\"joy3PosizioneY\" type=\"text\" style=\"visibility: collapse;\" /><br /> \
			Direzione:<input id=\"joy3Direzione\" type=\"text\" style=\"visibility: collapse;\" /><br /> \
			X :<input id=\"joy3X\" type=\"text\" style=\"visibility: collapse;\" /></br> \
			Y :<input id=\"joy3Y\" type=\"text\" style=\"visibility: collapse;\" /> \
		</div> \
 \
 <script> \
let StickStatus = \
{ \
    xPosition: 0, \
    yPosition: 0, \
    x: 0, \
    y: 0, \
    cardinalDirection: \"C\" \
}; \
 \
var JoyStick = (function(container, parameters, callback) \
{ \
    parameters = parameters || {}; \
    var title = (typeof parameters.title === \"undefined\" ? \"joystick\" : parameters.title), \
        width = (typeof parameters.width === \"undefined\" ? 0 : parameters.width), \
        height = (typeof parameters.height === \"undefined\" ? 0 : parameters.height), \
        internalFillColor = (typeof parameters.internalFillColor === \"undefined\" ? \"#00AA00\" : parameters.internalFillColor), \
        internalLineWidth = (typeof parameters.internalLineWidth === \"undefined\" ? 2 : parameters.internalLineWidth), \
        internalStrokeColor = (typeof parameters.internalStrokeColor === \"undefined\" ? \"#003300\" : parameters.internalStrokeColor), \
        externalLineWidth = (typeof parameters.externalLineWidth === \"undefined\" ? 2 : parameters.externalLineWidth), \
        externalStrokeColor = (typeof parameters.externalStrokeColor ===  \"undefined\" ? \"#008000\" : parameters.externalStrokeColor), \
        autoReturnToCenter = (typeof parameters.autoReturnToCenter === \"undefined\" ? true : parameters.autoReturnToCenter); \
 \
    callback = callback || function(StickStatus) {}; \
 \
     \
    var objContainer = document.getElementById(container); \
     \
    objContainer.style.touchAction = \"none\"; \
 \
    var canvas = document.createElement(\"canvas\"); \
    canvas.id = title; \
    if(width === 0) { width = objContainer.clientWidth; } \
    if(height === 0) { height = objContainer.clientHeight; } \
    canvas.width = width; \
    canvas.height = height; \
    objContainer.appendChild(canvas); \
    var context=canvas.getContext(\"2d\"); \
 \
    var pressed = 0;  \
    var circumference = 2 * Math.PI; \
    var internalRadius = (canvas.width-((canvas.width/2)+10))/2; \
    var maxMoveStick = internalRadius + 5; \
    var externalRadius = internalRadius + 30; \
    var centerX = canvas.width / 2; \
    var centerY = canvas.height / 2; \
    var directionHorizontalLimitPos = canvas.width / 10; \
    var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1; \
    var directionVerticalLimitPos = canvas.height / 10; \
    var directionVerticalLimitNeg = directionVerticalLimitPos * -1; \
     \
    var movedX=centerX; \
    var movedY=centerY; \
 \
     \
    if(\"ontouchstart\" in document.documentElement) \
    { \
        canvas.addEventListener(\"touchstart\", onTouchStart, false); \
        document.addEventListener(\"touchmove\", onTouchMove, false); \
        document.addEventListener(\"touchend\", onTouchEnd, false); \
    } \
    else \
    { \
        canvas.addEventListener(\"mousedown\", onMouseDown, false); \
        document.addEventListener(\"mousemove\", onMouseMove, false); \
        document.addEventListener(\"mouseup\", onMouseUp, false); \
    } \
     \
    drawExternal(); \
    drawInternal(); \
    function drawExternal() \
    { \
        context.beginPath(); \
        context.arc(centerX, centerY, externalRadius, 0, circumference, false); \
        context.lineWidth = externalLineWidth; \
        context.strokeStyle = externalStrokeColor; \
        context.stroke(); \
    } \
    function drawInternal() \
    { \
        context.beginPath(); \
        if(movedX<internalRadius) { movedX=maxMoveStick; } \
        if((movedX+internalRadius) > canvas.width) { movedX = canvas.width-(maxMoveStick); } \
        if(movedY<internalRadius) { movedY=maxMoveStick; } \
        if((movedY+internalRadius) > canvas.height) { movedY = canvas.height-(maxMoveStick); } \
        context.arc(movedX, movedY, internalRadius, 0, circumference, false); \
         \
        var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200); \
         \
        grd.addColorStop(0, internalFillColor); \
         \
        grd.addColorStop(1, internalStrokeColor); \
        context.fillStyle = grd; \
        context.fill(); \
        context.lineWidth = internalLineWidth; \
        context.strokeStyle = internalStrokeColor; \
        context.stroke(); \
    } \
 \
    let touchId = null; \
    function onTouchStart(event) \
    { \
        pressed = 1; \
        touchId = event.targetTouches[0].identifier; \
    } \
 \
    function onTouchMove(event) \
    { \
        if(pressed === 1 && event.targetTouches[0].target === canvas) \
        { \
            movedX = event.targetTouches[0].pageX; \
            movedY = event.targetTouches[0].pageY; \
             \
            if(canvas.offsetParent.tagName.toUpperCase() === \"BODY\") \
            { \
                movedX -= canvas.offsetLeft; \
                movedY -= canvas.offsetTop; \
            } \
            else \
            { \
                movedX -= canvas.offsetParent.offsetLeft; \
                movedY -= canvas.offsetParent.offsetTop; \
            } \
             \
            context.clearRect(0, 0, canvas.width, canvas.height); \
             \
            drawExternal(); \
            drawInternal(); \
 \
             \
            StickStatus.xPosition = movedX; \
            StickStatus.yPosition = movedY; \
            StickStatus.x = (100*((movedX - centerX)/maxMoveStick)).toFixed(); \
            StickStatus.y = ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed(); \
            StickStatus.cardinalDirection = getCardinalDirection(); \
            callback(StickStatus); \
        } \
    } \
 \
    function onTouchEnd(event) \
    { \
        if (event.changedTouches[0].identifier !== touchId) return; \
 \
        pressed = 0; \
         \
        if(autoReturnToCenter) \
        { \
            movedX = centerX; \
            movedY = centerY; \
        } \
         \
        context.clearRect(0, 0, canvas.width, canvas.height); \
         \
        drawExternal(); \
        drawInternal(); \
 \
         \
        StickStatus.xPosition = movedX; \
        StickStatus.yPosition = movedY; \
        StickStatus.x = (100*((movedX - centerX)/maxMoveStick)).toFixed(); \
        StickStatus.y = ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed(); \
        StickStatus.cardinalDirection = getCardinalDirection(); \
        callback(StickStatus); \
    } \
 \
    function onMouseDown(event)  \
    { \
        pressed = 1; \
    } \
 \
    function onMouseMove(event)  \
    { \
        if(pressed === 1) \
        { \
            movedX = event.pageX; \
            movedY = event.pageY; \
             \
            if(canvas.offsetParent.tagName.toUpperCase() === \"BODY\") \
            { \
                movedX -= canvas.offsetLeft; \
                movedY -= canvas.offsetTop; \
            } \
            else \
            { \
                movedX -= canvas.offsetParent.offsetLeft; \
                movedY -= canvas.offsetParent.offsetTop; \
            } \
             \
            context.clearRect(0, 0, canvas.width, canvas.height); \
             \
            drawExternal(); \
            drawInternal(); \
 \
             \
            StickStatus.xPosition = movedX; \
            StickStatus.yPosition = movedY; \
            StickStatus.x = (100*((movedX - centerX)/maxMoveStick)).toFixed(); \
            StickStatus.y = ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed(); \
            StickStatus.cardinalDirection = getCardinalDirection(); \
            callback(StickStatus); \
        } \
    } \
 \
    function onMouseUp(event)  \
    { \
        pressed = 0; \
         \
        if(autoReturnToCenter) \
        { \
            movedX = centerX; \
            movedY = centerY; \
        } \
         \
        context.clearRect(0, 0, canvas.width, canvas.height); \
         \
        drawExternal(); \
        drawInternal(); \
 \
         \
        StickStatus.xPosition = movedX; \
        StickStatus.yPosition = movedY; \
        StickStatus.x = (100*((movedX - centerX)/maxMoveStick)).toFixed(); \
        StickStatus.y = ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed(); \
        StickStatus.cardinalDirection = getCardinalDirection(); \
        callback(StickStatus); \
    } \
 \
    function getCardinalDirection() \
    { \
        let result = \"\"; \
        let orizontal = movedX - centerX; \
        let vertical = movedY - centerY; \
         \
        if(vertical >= directionVerticalLimitNeg && vertical <= directionVerticalLimitPos) \
        { \
            result = \"C\"; \
        } \
        if(vertical < directionVerticalLimitNeg) \
        { \
            result = \"N\"; \
        } \
        if(vertical > directionVerticalLimitPos) \
        { \
            result = \"S\"; \
        } \
         \
        if(orizontal < directionHorizontalLimitNeg) \
        { \
            if(result === \"C\") \
            {  \
                result = \"W\"; \
            } \
            else \
            { \
                result += \"W\"; \
            } \
        } \
        if(orizontal > directionHorizontalLimitPos) \
        { \
            if(result === \"C\") \
            {  \
                result = \"E\"; \
            } \
            else \
            { \
                result += \"E\"; \
            } \
        } \
         \
        return result; \
    } \
    this.GetWidth = function ()  \
    { \
        return canvas.width; \
    }; \
    this.GetHeight = function ()  \
    { \
        return canvas.height; \
    }; \
    this.GetPosX = function () \
    { \
        return movedX; \
    }; \
    this.GetPosY = function () \
    { \
        return movedY; \
    }; \
    this.GetX = function () \
    { \
        return (100*((movedX - centerX)/maxMoveStick)).toFixed(); \
    }; \
    this.GetY = function () \
    { \
        return ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed(); \
    }; \
    this.GetDir = function() \
    { \
        return getCardinalDirection(); \
    }; \
}); \
 \
 \
var joy3Param = {\"title\":\"joystick3\" }; \
var Joy3 = new JoyStick('joy3Div', joy3Param); \
 \
var joy3IinputPosX = document.getElementById(\"joy3PosizioneX\"); \
var joy3InputPosY = document.getElementById(\"joy3PosizioneY\"); \
var joy3Direzione = document.getElementById(\"joy3Direzione\"); \
var joy3X = document.getElementById(\"joy3X\"); \
var joy3Y = document.getElementById(\"joy3Y\"); \
 \
var gateway = `ws://${window.location.hostname}/ws`; \
var websocket; \
function initWebSocket() { \
  console.log('Trying to open a WebSocket connection...'); \
  websocket = new WebSocket(gateway); \
  websocket.onopen    = onOpen; \
  websocket.onclose   = onClose; \
  websocket.onmessage = onMessage;  \
  websocket.onerror = reConnect; \
} \
 \
function onOpen(event) { \
  console.log('Connection opened'); \
} \
 \
function onClose(event) { \
  console.log('Connection closed'); \
  setTimeout(initWebSocket, 2000); \
} \
 \
function onMessage(event) { \
  var state; \
  if (event.data ==\"1\"){ \
    state =\"ON\"; \
  } \
  else{ \
    state =\"OFF\"; \
  } \
   \
} \
 \
var lastX = 0; \
var lastY = 0; \
 \
function update(){ \
  var x = Joy3.GetPosX(); \
  var y = Joy3.GetPosY(); \
  if(x != lastX || y != lastY) \
  { \
    websocket.send('joystick:' + x + '|' + y + '|' + Joy3.GetDir() + '|' + Joy3.GetX() + '|' + Joy3.GetY()); \
    lastX = x; \
    lastY = y; \
  } \
} \
function keepalive(){ \
  websocket.send('keep-alive'); \
} \
 \
function reConnect(event) { \
  setTimeout(function(){ initWebSocket(); }, 332); \
   \
} \
 \
setInterval(function(){ update(); }, 50); \
setInterval(function(){ keepalive(); }, 777); \
 \
initWebSocket(); \
</script> \
</body> \
</html> ";

void setup() {
  analogWrite(P5, 0);
  analogWrite(P17, 0);
  analogWrite(P19, 0);
  analogWrite(P18, 0);
  analogWrite(P0, 0);
  analogWrite(P2, 0);
  analogWrite(P16, 0);
  analogWrite(P4, 0);
  
 //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000);
  connectInfrared();  
  delay(500);
  yield();
  Serial.println("Access Point Web Server");

  pinMode(led, OUTPUT);      // set the LED pin mode

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_15dBm);
  if(WiFi.softAP(ssid, pass)) {
    Serial.println("Access Point is Ready");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP()); 
  } else {
    Serial.println("Failed to start AP");
  }

  /*wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B);
  //Create open network. Change this line if you want to create an WEP network:
  bool wstatus = WiFi.softAP(ssid, pass);

  if(!wstatus)
  {
    Serial.println("WIFI Failed!!!");
  }*/

  // wait 3 seconds for connection:
  delay(50);
  keepAliveTimeout();
  // you're connected now, so print out the status
  printWiFiStatus();
  initWebSocket();
  //esp_wifi_set_ps(WIFI_PS_NONE);

  // Route for root / web page
  server.on("/", HTTP_GET, [index_html](AsyncWebServerRequest *request){
    //uint8_t[index_html.length()] rawhtml;
    //index_html.getBytes(rawhtml, index_html.length());
    //const size_t rawsize = (size_t)index_html.length();
    request->send_P(200, "text/html", (uint8_t*)index_html.c_str(), (size_t)index_html.length());
    //request->send(200, "text/html", index_html);
    //request->send(200, "/", index_html, false);  
  });


  // Start server
  server.begin();
  delay(50);
  // Over the air update
  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%   \r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });
    ArduinoOTA.begin();
    delay(50);
    //yield();
    delay(1000);
    //recoverI2CBus();
}

void loop() {
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
      printWiFiStatus();
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
      printWiFiStatus();
    }
  }

  ws.cleanupClients();
  checkFrontDistanceSensor();
  checkTimeout();
  delay(2);
  ArduinoOTA.handle();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}


