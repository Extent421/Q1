rx5808 takes approx 26ms for rssi to stabilize after changing channels
10 bytes "Q1Settings"
1 byte settings version
1 byte enable connect to AP mode
2 byte frequency
2 byte low calibrate
2 byte med calibrate
2 byte high calibrate
32 character this station ssid
64 character this station password
32 character max ssid length
64 character max password length



required build environment

arduino 1.8.1
esp8266 community board  2.3.0

https://github.com/Links2004/arduinoWebSockets
https://github.com/me-no-dev/ESPAsyncTCP

recomended
https://github.com/esp8266/arduino-esp8266fs-plugin