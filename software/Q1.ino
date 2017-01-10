#include "Q1.h"
#include "pins.h"

#include "spi.h"

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <Ticker.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <EEPROM.h>


Ticker refresh;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);
File fsUploadFile;

uint8_t APConnectMode = 0;

char AP_SSID[34]     = "";
char AP_password[65] = "";

/* Soft AP network parameters */
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
char station_password[65] = "";
char station_SSID[33] = "";

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

int readValue = 0;

uint16_t minCal = 0;
uint16_t threshCal = 1023;
uint16_t maxCal = 1023;
uint16_t rxFrequency = 0;
uint8_t minPulse = 0;


pulseData recordedPulses[100];
int recordedPulsesIndex = 0;

bool samplerRunning = false;
bool sessionRunning = false;
bool pulseActive = false;
unsigned long pulseStart = 0;
unsigned long pulseEnd = 0;
unsigned long pulseEndLast = 0;
unsigned long lastSampleTime =0;

unsigned long connectingTime = 0;

unsigned long timerZero = 0;
int lapCount = 0;
unsigned long lastLapTime = 0;

bool blinkFlag = false;
bool sendSampleFlag = false;

void setup() {
	Serial.begin(115200);
	delay(100);

	pinMode(BLINK_PIN, OUTPUT);
	digitalWrite(BLINK_PIN, HIGH);
	pinMode(BEEP_PIN, OUTPUT);
	digitalWrite(BEEP_PIN, LOW);

	pinMode(SETTINGS_RESET_PIN, INPUT_PULLUP);

	spi_init(HSPI);
	// RX datasheet says to use leading clock, but with hardware SPI it only seems to work with trailing
	spi_mode(HSPI, 1, 0); // trailing clock, low idle

	SPIFFS.begin();

	if ( !digitalRead( SETTINGS_RESET_PIN) ) {
		Serial.println();
		Serial.println("settings reset requested");
		resetSettings();
	}

	// power down parts of the RX we're not using
	uint32_t registerValue = 0;
	registerValue = PD_VCLAMP | PD_VAMP | PD_IFAF | PD_DIV4 | PD_5GVCO | PD_AU6M | PD_6M | PD_AU6M5 | PD_6M5 | PD_REG1D8 | PD_DIV80 | PD_PLL1D8;
	vtxWrite(0x0A, registerValue);

	loadSettings();


	WiFi.mode(WIFI_AP_STA);

	/* Setup the DNS server redirecting all the domains to the apIP */
	Serial.println("Starting DNS ");
	dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
	dnsServer.start(DNS_PORT, "*", apIP);

	Serial.println("Starting softAP ");
	WiFi.softAPConfig(apIP, apIP, netMsk);
	WiFi.softAP(station_SSID, station_password);

	if (APConnectMode == AP_AUTOCONNECT){
		Serial.print("Connecting to ");
		Serial.println(AP_SSID);
		WiFi.begin(AP_SSID, AP_password);
		connectingTime = millis();
		while (WiFi.status() != WL_CONNECTED) {
			if ((millis() - connectingTime) > 10000) {
				WiFi.disconnect();
				break;
			}
			delay(500);
			Serial.print(".");
		}

		if (WiFi.status() == WL_CONNECTED) {
			Serial.println("");
			Serial.println("WiFi connected");
			Serial.println("IP address: ");
			Serial.println(WiFi.localIP());
		}
	} else {
		WiFi.disconnect();
	}

	ArduinoOTA.setHostname("");
	ArduinoOTA.onStart([]() {
		Serial.println("Start");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
	Serial.printf("Error[%u]: ", error);
	if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();

	webSocket.begin();
	webSocket.onEvent(webSocketEvent);

	//SERVER INIT
	//called when the url is not defined here
	//use it to load content from SPIFFS
	server.onNotFound([](){
	if(!handleFileRead(server.uri()))
		server.send(404, "text/plain", "FileNotFound");
	});
	server.begin();


}

void loop() {
	dnsServer.processNextRequest();
	ArduinoOTA.handle();
    webSocket.loop();
    server.handleClient();
    lapTimerLoop();
    if (blinkFlag){
    	blinkFlag=false;
    	doBlink();
    }
    if (sendSampleFlag){
    	sendSampleFlag=false;
    	char buffer[7];
    	sprintf(buffer, "a%i", readValue);
    	webSocket.broadcastTXT(buffer);
    }
}

void resetSettings(){

	uint8_t mac[WL_MAC_ADDR_LENGTH];
	WiFi.softAPmacAddress(mac);
	char buffer[20];
	sprintf(buffer, "Q1 node - %X%X",  mac[WL_MAC_ADDR_LENGTH-2],  mac[WL_MAC_ADDR_LENGTH-1] );

	EEPROM.begin(256);
	for (int i = 0; i < 256; i++)  EEPROM.write(i, 0);
	EEPROM.put(0, "Q1Settings");
	EEPROM.put(10, 0); // settings version number
	EEPROM.put(11, AP_CONNECT_DISABLED); // ap connect mode
	EEPROM.put(12, 5704); // rx frequency
	EEPROM.put(14, 600); // low calibrate
	EEPROM.put(16, 1024); // mid calibrate
	EEPROM.put(18, 1024); // high calibrate
	EEPROM.put(20, buffer); // station SSID
	EEPROM.put(52, "12345678"); // station password
	EEPROM.put(116, ""); // ap SSID
	EEPROM.put(148, ""); // ap password
	EEPROM.put(212, 15); // ap password

	EEPROM.end();
}

void saveSettings(){

	EEPROM.begin(256);

	EEPROM.put(11, APConnectMode); // ap connect mode
	EEPROM.put(12, rxFrequency); // rx frequency
	EEPROM.put(14, minCal); // low calibrate
	EEPROM.put(16, threshCal); // mid calibrate
	EEPROM.put(18, maxCal); // high calibrate
	EEPROM.put(20, station_SSID); // station SSID
	EEPROM.put(52, station_password); // station password
	EEPROM.put(116, AP_SSID); // ap SSID
	EEPROM.put(148, AP_password); // ap password
	EEPROM.put(212, minPulse); // ap password

	// next byte 213

	EEPROM.end();
}

void sendSettings(uint8_t num){
	char buffer[128];

	sprintf(buffer, "dAPMode:%i", APConnectMode );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dfreq:%i", rxFrequency );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dminCal:%i", minCal );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dthreshCal:%i", threshCal );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dmaxCal:%i", maxCal );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dstationSSID:%s", station_SSID );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dstationPass:%s", station_password );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dAPSSID:%s", AP_SSID );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dAPPass:%s", AP_password );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dminPulse:%i", minPulse );
	webSocket.sendTXT(num, buffer);
}

void sendStatus(uint8_t num){
	char buffer[128];
	sprintf(buffer, "dAPstatus:%i", WiFi.status() );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dminCal:%i", minCal );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dthreshCal:%i", threshCal );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dmaxCal:%i", maxCal );
	webSocket.sendTXT(num, buffer);
	sprintf(buffer, "dsessionRunning:%i", sessionRunning );
	webSocket.sendTXT(num, buffer);
	if (WiFi.status() == WL_CONNECTED) {
		sprintf(buffer, "dAPIP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
		webSocket.sendTXT(num, buffer);
	}
}

void loadSettings(){
	EEPROM.begin(256);

	char headMessage[12] = "";
	uint8_t settingsVersion = 0;

	for (int i = 0; i < 11; i++){
		headMessage[i] = EEPROM.read(i);
	}
	headMessage[11] = '\0';

	EEPROM.get( 10, settingsVersion );
	EEPROM.get( 11, APConnectMode );
	EEPROM.get( 12, rxFrequency );
	setVTXChannel(rxFrequency);

	EEPROM.get( 14, minCal );
	EEPROM.get( 16, threshCal );
	EEPROM.get( 18, maxCal );

	for (int i = 0; i < 32; i++){
		station_SSID[i] = EEPROM.read(i+20);
	}
	station_SSID[32] = '\0';
	for (int i = 0; i < 64; i++){
		station_password[i] = EEPROM.read(i+52);
	}
	station_password[64] = '\0';


	for (int i = 0; i < 32; i++){
		AP_SSID[i] = EEPROM.read(i+116);
	}
	AP_SSID[32] = '\0';
	for (int i = 0; i < 64; i++){
		AP_password[i] = EEPROM.read(i+148);
	}
	AP_password[64] = '\0';

	EEPROM.get( 212, minPulse );
	EEPROM.end();

	if (strcmp(headMessage, "Q1Settings")  != 0) {
		Serial.println("invalid settings EEPROM, resetting");
		resetSettings();
		loadSettings();
		return;
	}

}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "mobile.html";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}


void lapTimerLoop(){
	unsigned long currTime = 0;

	if (samplerRunning){
    	currTime = getLapTime();
    	if ((currTime - lastSampleTime) > 2) {
        	readValue = getRSSI();
        	lastSampleTime = currTime;
    	}
    }

    if (sessionRunning){
    	if (readValue >= convertRange(threshCal) ){

    		if (!pulseActive){
    			pulseActive = true;
				pulseStart = currTime;  // more than a single sample drop, start a new pulse
    		}

    	} else {
    		if (pulseActive){
    			pulseEnd = currTime;
    			pulseActive = false;
    			recordedPulses[recordedPulsesIndex].start = pulseStart;
    			recordedPulses[recordedPulsesIndex].end = pulseEnd;
    			recordedPulsesIndex++;
    			
    			//unsigned long pulseLength = currTime-pulseStart;

    			//char buffer[128];
    			//sprintf(buffer, "c%i, %i", (currTime-pulseStart)/1000, (pulseStart-pulseEndLast)/1000 );
    			//webSocket.sendTXT(0, buffer);

    			pulseEndLast = pulseEnd;

    		}
    	}

    	if ( ((currTime - pulseEndLast) >1000) && (recordedPulsesIndex > 0) && !pulseActive ){
    		pulseData expandedPulses[recordedPulsesIndex];
    		for (int i = 0; i< recordedPulsesIndex;i++){
    			unsigned long newStart = 0;
    			unsigned long newEnd = 0;
    			unsigned long expand = 0;
    			expand = (recordedPulses[i].end - recordedPulses[i].start) * 0.1;
    			expandedPulses[i].start = recordedPulses[i].start - expand;
    			expandedPulses[i].end = recordedPulses[i].end + expand;
    			
    		}
    		
    		pulseData mergedPulses[recordedPulsesIndex];
    		int mergedIndex = 0;
    		bool merged = false;
    		
    		for (int i = 0; i< recordedPulsesIndex;i++){
        		for (int j = 0; j< mergedIndex;j++){
        			if( mergedPulses[j].end > expandedPulses[i].start ) {
        				mergedPulses[j].start = _min(mergedPulses[j].start,  expandedPulses[i].start  );
        				mergedPulses[j].end = _max(mergedPulses[j].end,  expandedPulses[i].end  );
        				mergedIndex = j+1;
        				merged = true;
        			}
        			
        		}
        		if (!merged){
					mergedPulses[mergedIndex].start = recordedPulses[i].start;
					mergedPulses[mergedIndex].end = recordedPulses[i].end;
					mergedIndex++;
        		} else {
        			merged = false;
        		}
        		
    		}
    		
    		unsigned long maxLength = 0;
    		int maxIndex = 0;
    		for (int i = 0; i< mergedIndex;i++){
    			unsigned long tempLength = mergedPulses[i].end - mergedPulses[i].start;
    			if ( tempLength > maxLength ) {
    				maxLength = tempLength;
    				maxIndex = i;
    			}
    		}
    		
    	if (maxLength > minPulse) {	// if the pulse time was too short just throw it away
			unsigned long offsetTime = getLapTime();
			digitalWrite(BLINK_PIN, LOW);
			digitalWrite(BEEP_PIN, HIGH);
			delay(250);
			digitalWrite(BLINK_PIN, HIGH);
			digitalWrite(BEEP_PIN, LOW);

			unsigned long lapTime = (mergedPulses[maxIndex].end + mergedPulses[maxIndex].start ) / 2;
			int offset = offsetTime - lapTime ;
			char buffer[128];
			char timecodeStr[16];
			char lapTimeStr[16];
			dtostrf(lapTime/1000.0, 4, 2, timecodeStr);
			dtostrf((lapTime-lastLapTime)/1000.0, 4, 2, lapTimeStr);

			sprintf(buffer, "cL%i,%s,%s,%i,%i", lapCount, timecodeStr, lapTimeStr, maxLength, offset );
			webSocket.broadcastTXT(buffer);

			lastLapTime = lapTime;
			lapCount++;
    	}
		/*
		char buffer[128];
		sprintf(buffer, "clap %i of %i(%i), %i", maxIndex, mergedIndex, recordedPulsesIndex, maxLength/1000 );
		webSocket.broadcastTXT(buffer);
		for (int i = 0; i< mergedIndex;i++){
			delay(0);
			sprintf(buffer, "c%i, %i", i, (mergedPulses[i].end - mergedPulses[i].start ) /1000 );
			
			webSocket.broadcastTXT(buffer);
		}
		*/

    	//reset for the next lap	
    	recordedPulsesIndex = 0;
    	}
    }
}

void calibrateHigh(){
	int avg = analogAverage(1000);
	Serial.print(avg);
	Serial.println( " high.");
	if (avg <= minCal) avg = minCal + 1;
	maxCal = avg;
	saveSettings();
}
void calibrateThreshold(){
	int avg = analogAverage(1000);
	Serial.print(avg);
	Serial.println( " thresh.");
	threshCal = avg;
	saveSettings();
}

void calibrateLow(){
	int avg = analogAverage(1000);
	Serial.print(avg);
	Serial.println( " low.");
	if (avg >= minCal) avg = maxCal - 1;
	minCal = avg;
	saveSettings();
}

int analogAverage(int samples){
	long acu = 0;
	for (int i=0; i<samples; i++){
		acu = acu + analogRead(A0);
		delay(0);
	}
	int avg = acu / samples;
	return (avg);
}

int getRSSI(){
	int val = 0;

	val = analogRead(A0);
	val = map(val, minCal, maxCal, 0, 255);
	return( val );
}
int convertRange(int val){
	val = map(val, minCal, maxCal, 0, 255);
	return( val );
}
void sample(){
	int val = 0;

	val = readValue;//getRSSI();
	char buffer[7];
	sprintf(buffer, "a%i", val);
	//itoa(val,buffer,10);
	webSocket.sendTXT(0, buffer);
}
void sampleTimer(){
	sendSampleFlag = true;
}
void start(){
	refresh.attach(0.1, sampleTimer);
	samplerRunning = true;
}
void stop(){
	refresh.detach();
	if (!sessionRunning){
		samplerRunning = false;
	}
}
void startSession(){
	timerReset();
	lapCount = 0;
	lastLapTime = 0;
	samplerRunning = true;
	sessionRunning = true;
	blinkFlag = true;
}
void doBlink(){
	digitalWrite(BLINK_PIN, LOW);
	digitalWrite(BEEP_PIN, HIGH);
	delay(250);
	digitalWrite(BLINK_PIN, HIGH);
	digitalWrite(BEEP_PIN, LOW);
	delay(250);
	digitalWrite(BLINK_PIN, LOW);
	digitalWrite(BEEP_PIN, HIGH);
	delay(250);
	digitalWrite(BLINK_PIN, HIGH);
	digitalWrite(BEEP_PIN, LOW);

}

void endSession(){
	timerReset();
	lapCount = 0;
	lastLapTime = 0;
	samplerRunning = false;
	sessionRunning = false;
}
void timerReset(){
	timerZero = millis();
}

unsigned long getLapTime(){
	return millis() - timerZero;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t lenght) {

    switch(type) {
        case WStype_DISCONNECTED:
        	Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
				
				// send message to client
				webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
        	//Serial.printf("[%u] get Text: %s\n", num, payload);

			if (strcmp((char*)payload, "callo")  == 0) {
				calibrateLow();
			}else if (strcmp((char*)payload, "calthresh")  == 0) {
				calibrateThreshold();
			}else if (strcmp((char*)payload, "calhi")  == 0) {
				calibrateHigh();
			}else if (strcmp((char*)payload, "sample")  == 0) {
				sample();
			}else if (strcmp((char*)payload, "start")  == 0) {
				start();
			}else if (strcmp((char*)payload, "stop")  == 0) {
				stop();
			}else if (strcmp((char*)payload, "sessionStart")  == 0) {
				startSession();
			}else if (strcmp((char*)payload, "sessionEnd")  == 0) {
				endSession();
			}else if (strcmp((char*)payload, "seep")  == 0) {
				saveSettings();
			}else if (strcmp((char*)payload, "getSettings")  == 0) {
					sendSettings(num);
			}else if (strcmp((char*)payload, "getStatus")  == 0) {
					sendStatus(num);
			}else if (strcmp((char*)payload, "reboot")  == 0) {
				ESP.restart();
				// reboot
			}else if (strcmp((char*)payload, "vidDwn")  == 0) {

			}else if (strncmp((char*)payload, "sch", 3)  == 0) {
				// set rx channel
				char *payloadData = (char*)payload + 3; //get the embedded asci channel
				uint16_t channel = 0;
				channel = atoi(payloadData);
				channel = setVTXChannel(channel);
				rxFrequency=channel;
				saveSettings();
			}else if (strncmp((char*)payload, "smp", 3)  == 0) {
				char *payloadData = (char*)payload + 3;
				minPulse = atoi(payloadData);
				saveSettings();
			}else if (strncmp((char*)payload, "apc", 3)  == 0) {
				// connect to AP
			}else if (strncmp((char*)payload, "apdc", 4)  == 0) {
				// disconnect from AP
			}else if (strncmp((char*)payload, "sapn", 4)  == 0) {
				// set the AP name
				char *payloadData = (char*)payload + 4;
				if ( strlen(payloadData) > 32) {
					//error case
					return;
				}
				strcpy( AP_SSID, payloadData);
				saveSettings();
			}else if (strncmp((char*)payload, "sapp", 4)  == 0) {
				// set the AP password
				char *payloadData = (char*)payload + 4;
				if ( strlen(payloadData) > 64) {
					//error case
					return;
				}
				strcpy( AP_password, payloadData);
				saveSettings();
			}else if (strncmp((char*)payload, "sapm", 4)  == 0) {
				// set the AP mode
				char *payloadData = (char*)payload + 4;
				int mode = 0;
				mode = atoi(payloadData);
				APConnectMode = mode;
				Serial.print("set mode ");
				Serial.println(mode);
				saveSettings();
			}else if (strncmp((char*)payload, "sstn", 4)  == 0) {
				// set the station password
				char *payloadData = (char*)payload + 4;
				if ( strlen(payloadData) > 32) {
					//error case
					return;
				}
				strcpy( station_SSID, payloadData);
				saveSettings();
			}else if (strncmp((char*)payload, "sstp", 4)  == 0) {
				// set the station password
					char *payloadData = (char*)payload + 4;
					if ( strlen(payloadData) > 64) {
						//error case
						return;
					}
					strcpy( station_password, payloadData);
					saveSettings();
			} else {
				webSocket.sendTXT(0, "pong");
			}
			
            // send message to client
            // webSocket.sendTXT(num, "message here");

            // send data to all connected clients
            // webSocket.broadcastTXT("message here");
            break;
        case WStype_BIN:
        	Serial.printf("[%u] get binary lenght: %u\n", num, lenght);
            hexdump(payload, lenght);

            // send message to client
            // webSocket.sendBIN(num, payload, lenght);
            break;
    }

}


uint16_t setVTXChannel(uint16_t channel)
{
	if (channel > 5950) channel = 5950;
	if (channel < 5630) channel = 5630;

	uint32_t registerValue = 0;
	registerValue = getChannelValueFromFreq(channel);
	vtxWrite(0x1, registerValue);
	return(channel);
}

uint32_t getChannelValueFromFreq(uint16_t frequency){
	//get the register value for the supplied frequency (in mhz)
	//frequency=2*(N*32+A)
	uint16_t nTerm = 0;
	uint16_t aTerm = 0;
	uint32_t registerValue = 0;

	nTerm = ((frequency-479)/2)/32;
	aTerm = ((frequency-479)/2)-(nTerm*32);

	registerValue = (nTerm<<7) | (aTerm);
	return(registerValue);
}

void vtxWrite(uint8_t address, uint32_t data){
	// pack a buffer to be written to the vtx
	// VTX expects LS bit first, so we assemble the buffer in reverse
	// so when the SPI hardware sends it MS bit first it will come in the right order
	uint32_t sendBuffer = 0;
	uint32_t temp = 0;

	// assemble register address
	temp = address;
	for (uint8_t i = 0; i < 4; i++)
	{
		sendBuffer <<= 1;
		sendBuffer = sendBuffer| (temp & 0x1);
		temp >>= 1;
	}

	// set write bit (we're always in write mode)
	sendBuffer <<= 1;
	sendBuffer = sendBuffer| 0x1;

	// assemble register data
	temp = data;
	for (uint8_t i = 0; i < 20; i++)
	{
		sendBuffer <<= 1;
		sendBuffer = sendBuffer| (temp & 0x1);
		temp >>= 1;
	}

	// send command to VTX

	spi_txd(HSPI, 25, sendBuffer);

}
