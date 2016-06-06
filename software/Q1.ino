#include "Q1.h"
#include "pins.h"

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

Ticker refresh;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer server(80);
File fsUploadFile;

const char* ssid     = "";
const char* password = "";

/* Soft AP network parameters */
IPAddress apIP(192, 168, 4, 1);
IPAddress netMsk(255, 255, 255, 0);
const char *softAP_password = "12345678";

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

int readValue = 0;

int minCal = 0;
int threshCal = 1023;
int maxCal = 1023;

pulseData recordedPulses[100];
int recordedPulsesIndex = 0;

bool running = false;
bool pulseActive = false;
unsigned long pulseStart = 0;
unsigned long pulseEnd = 0;
unsigned long pulseEndLast = 0;
unsigned long lastSampleTime =0;

unsigned long timerZero = 0;
int lapCount = 0;
unsigned long lastLapTime = 0;


void setup() {
  Serial.begin(115200);
  delay(100);
 
  pinMode(BLINK_PIN, OUTPUT);
  digitalWrite(BLINK_PIN, LOW);

  
  WiFi.mode(WIFI_AP_STA);
	uint8_t mac[WL_MAC_ADDR_LENGTH];
	WiFi.softAPmacAddress(mac);
	char buffer[20];
	sprintf(buffer, "test - %X%X",  mac[WL_MAC_ADDR_LENGTH-1],  mac[WL_MAC_ADDR_LENGTH-2] );
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(buffer, softAP_password);
  Serial.print("Starting softAP ");
  Serial.print(buffer);
  Serial.print(" ");
  Serial.println(softAP_password);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  /* Setup the DNS server redirecting all the domains to the apIP */
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", apIP);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
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

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  SPIFFS.begin();

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
    if (running){
    	int val = 0;

    	unsigned long currTime = getLapTime();
    	if ((currTime - lastSampleTime) > 5) {
        	readValue = getRSSI();
        	lastSampleTime = currTime;
    	}
    	val = readValue;

    	if (val >= convertRange(threshCal) ){

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
    		
    	unsigned long offsetTime = getLapTime();
		digitalWrite(BLINK_PIN, HIGH);
		delay(250);
		digitalWrite(BLINK_PIN, LOW);

		unsigned long lapTime = (mergedPulses[maxIndex].end + mergedPulses[maxIndex].start ) / 2;
		int offset = offsetTime - lapTime ;
		char buffer[128];
		char timecodeStr[16];
		char lapTimeStr[16];
		dtostrf(lapTime/1000.0, 4, 2, timecodeStr);
		dtostrf((lapTime-lastLapTime)/1000.0, 4, 2, lapTimeStr);

		sprintf(buffer, "cL%i,%s,%s,%i,%i", lapCount, timecodeStr, lapTimeStr, maxLength, offset );
		webSocket.sendTXT(0, buffer);

		lastLapTime = lapTime;
		lapCount++;

		/*
		char buffer[128];
		sprintf(buffer, "clap %i of %i(%i), %i", maxIndex, mergedIndex, recordedPulsesIndex, maxLength/1000 );
		webSocket.sendTXT(0, buffer);
		for (int i = 0; i< mergedIndex;i++){
			delay(0);
			sprintf(buffer, "c%i, %i", i, (mergedPulses[i].end - mergedPulses[i].start ) /1000 );
			
			webSocket.sendTXT(0, buffer);
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
	maxCal = avg;}

void calibrateThreshold(){
	int avg = analogAverage(1000);
	Serial.print(avg);
	Serial.println( " thresh.");
	threshCal = avg;
	char buffer[7];
	sprintf(buffer, "b%i", avg);
	webSocket.sendTXT(0, buffer);
}

void calibrateLow(){
	
	int avg = analogAverage(1000);
	Serial.print(avg);
	Serial.println( " low.");
	minCal = avg;

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
	int val = 0;

	val = readValue;//getRSSI();
	char buffer[7];
	sprintf(buffer, "a%i", val);
	//itoa(val,buffer,10);
	//webSocket.sendTXT(0, buffer);

}
void start(){

	refresh.attach(0.1, sample);
	running = true;

}
void stop(){
	refresh.detach();
	running = false;
}

void startSession(){
	timerReset();
	lapCount = 0;
	lastLapTime = 0;
	running = true;

	delay(0);
	digitalWrite(BLINK_PIN, HIGH);
	delay(250);
	digitalWrite(BLINK_PIN, LOW);
	delay(250);
	digitalWrite(BLINK_PIN, HIGH);
	delay(250);
	digitalWrite(BLINK_PIN, LOW);


}

void endSession(){
	timerReset();
	lapCount = 0;
	lastLapTime = 0;
	running = false;
	/*
	digitalWrite(BLINK_PIN, HIGH);
	delay(250);
	digitalWrite(BLINK_PIN, LOW);
	delay(250);
	digitalWrite(BLINK_PIN, HIGH);
	delay(250);
	digitalWrite(BLINK_PIN, LOW);
	*/
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
