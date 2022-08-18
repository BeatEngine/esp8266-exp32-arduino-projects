# esp8266-exp32-arduino-projects
Collection of my arduino / esp microcontroller projects.

## Motor Control with esp8266 (or different modules):

The time of close is reduced to a bit higher maximum as the time of open was.

### Pins motor control:
D5 & D6 control the direction where the motor is powered.
WiFi use the HTTP requests to control the actions open,stop,close ... for example with an android app.

### Optional:
D7 & D8 are used for serial connection when you use the voice recognizer module.
--> you have to speak (record 0,1,2,3,4,5) ( max 1.5 seconds per voice command)

0 <---> open 

1 <---> stop

2 <---> close

(For example female voice or very different to the first):

3 <---> open 

4 <---> stop

5 <---> close
