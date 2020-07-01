/*
   ESPblinds  V 1.0
   
   - shutters controlled according to a timetable
   - fabric shutters controlled according to a timetable and weather
   - OTA uploading of sketches supported
   - experimental webserver
   - RGB led indicates events like sending data
   - log of all IR transmissions on the server to better check if a transmission has failed
   
   
   by Kai Arnetzl

*/

/*
   note: autoformat in the Arduino IDE screws the timetable formatting up, need to adjust afterwards
   true = up, false = down
	 
	 
	 RGB led colors:
	 - white:  booting up
	 - green:  sending IR
	 - cyan:   getting data from the internet
	 - purple: sending a client page contents
	 - blue:   recieving a new scetch via OTA
	 - yellow: scheduled restart
	 - red blinking: no WiFi connection/not reading temperature sensor
*/



#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <JSON_Decoder.h>
#include <DarkSkyWeather.h>
#include <TimeLib.h>

#ifndef STASSID
#define STASSID "xxxxxxxx"    //add network name here
#define STAPSK  "xxxxxxxxxxxxx"  //add network password here
#endif
#define OTA_PASSWORD "xxxxxxxxxxx"  //add OTA password here
#define FRONT_ALL 7
#define BACK_MOST 8
#define BACK_LAST 9
#define ONE_WIRE_BUS 12
#define IR_PIN 4		 //D2
#define RED_PIN 14   //D5
#define GREEN_PIN 12 //D6
#define BLUE_PIN 13  //D7

String api_key = "xxxxxxxxxxxxxxxxxxx";   //add DarkSky API key here
String latitude = "xx.xxxx";     //add latitude of the blinds here
String longitude = "xx.xxxx";    //add longitude of the blinds here
String units = "si";
String language = "";
const char* ssid = STASSID;
const char* password = STAPSK;
const long utcOffsetInSeconds = 3600;
static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 1;
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];

String log = ""
bool sync_done = false;
int last;
bool blink_state = false;
bool error_notification = false;
bool error_blink_state = false;
byte check_day = 0;
byte check_month = 0;
bool r_state[3] = {true, true, true}; //0 = front shutters, 1 = fabric, 2 = back shutters
bool last_state[3] = {false, false, false};
int weather_data[3];
byte date;


//Compressed IR Codes
bool r1_1[24] =    {0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1};
bool r1_2[24] =    {0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0};
bool r1_3[24] =    {0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0};
bool r1_4[24] =    {0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0};
bool r1_5[24] =    {0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
bool r1_6[24] =    {0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1};
bool r1_all[24] =  {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
bool r2[24] =      {0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0};
bool r3[24] =      {0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0};

/* 
   times ignore daylight saving time, all times are UTC-1
   timetable structure:
   down{Hour, Minute}, up{Hour, Minute}    for all of the r1 shutters (includes fabric)
   down{Hour, Minute}, up{Hour, Minute}    for the r2 and r3 shutters
   from{Day, Month}                        end date is the start date of the next section
   Setting a time to {254, x} means that the shutters wont go down that day, setting it to {255, x} means that the shutters wont go up that day
*/
const byte times[2][9][2][2] =  {{{{10, 30}, {13, 0}},  {{9, 30},  {13, 30}}, {{8, 0},   {14, 0}},  {{255, 0}, {255, 0}}, {{255, 0}, {255, 0}}, {{8, 0},   {14, 0}}, 	{{8, 45}, {13, 30}}, {{9, 30},  {13, 0}},  {{10, 30}, {13, 0}}}, 
                                 {{{254, 0}, {254, 0}}, {{254, 0}, {254, 0}}, {{254, 0}, {254, 0}}, {{11, 30}, {20, 30}}, {{11, 30}, {19,  0}}, {{11, 45}, {18, 30}},	{{12, 0}, {18, 0}},  {{254, 0}, {254, 0}}, {{254, 0}, {254, 0}}}};
const byte dates[10][2] =          {{1, 4},              {15, 4},             {1, 5},            		{16, 5},              {1, 8},               {16, 8}, 							{1, 9},              {16, 9},            	 {1, 10},           {20, 10}};

//html for the server
const PROGMEM char * root_page = "<!DOCTYPE HTML>"
            "<html>"
              "<head>"
                "<title>ESP Rollo</title>"
                "<meta http-equiv='refresh' content='60'/>"
              "</head>"
              "<body>"
	  						"<CENTER>"
                	"<h1>ESP Rollo - Server</h1>"
	  						"</CENTER>"
                "<p><sup>" + timeAsString() + "</sup></p>"
                "<p>Roll&aumlden zur Stra&szlige: " + expressState(0) + "</p>"
                "<p>" + showCurrentTimes(0) + "<p>"
                "<p>Roll&aumlden zum Garten: " + expressState(2) + "</p>"
                "<p>" + showCurrentTimes(2) + "<p>"
                "<p>Stoffroll&aumlden: " + expressState(1) + "</p>"
                "<p>" + showCurrentTimes(1) + "</p>"
                "<p>&nbsp;</p>"
                "<p>Temperatur Dachboden: " + temperatureString(getTemperature()) + "</p>"
                "<p>&nbsp;</p>"
                "<p>Wetter laut Darksky</p>"
                "<p>Niederschlagswahrscheinlichkeit: " + getDarksky(0) + "&#037</p>"
                "<p>Wolkenbedeckung: " + getDarksky(1) + "&#037</p>"
                "<p>Luftfeuchtigkeit: " + getDarksky(2) + "&#037</p>"
                "<p>&nbsp;</p>"
								"<a href=\"/values\">Zeiten und Schwellwerte</a>"
                "<a href=\"/log\">Sende-Log</a>"
              "</body>"
            "</html>";

const PROGMEM char * log_page = "<!DOCTYPE HTML>"
            "<html>"
              "<head>"
                "<title>ESP Rollo Log</title>"
                "<meta http-equiv='refresh' content='60'/>"
              "</head>"
              "<body>"
	  						"<CENTER>"
                	"<h1>ESP Rollo - Log</h1>"
	  						"</CENTER>"
								"<p><sup>" + timeAsString() + "</sup></p>"
                "<p>Alle an die Rollos gesendeten Befehle seit Anfang des Monats.</p>"
	  						"<p>Wird auch bei Stromverlust, Resets und Abstürzen geleert.</p>"
								"<p>&nbsp;</p>"
								"<a href=\"/\"> Zurück</a>"
								"<p>&nbsp;</p>"
								"<p>&nbsp;</p>" +
								getLog() +
              "</body>"
            "</html>";

byte current_times[2][2];

MDNSResponder mdns;
DS_Weather dsw;
ESP8266WebServer server(80);
IRsend irsend(4);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
ADC_MODE(ADC_VCC);
WiFiUDP Udp;
EspClass ESPm;
unsigned int localPort = 8888;
time_t getNtpTime();

//********************************setup function***************************************

void setup() {    //set everything up
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("rollo-esp8266");
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
  });
  ArduinoOTA.onEnd([]() {
     digitalWrite(blue_pin, LOW);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
     analogWrite(blue_pin, map(progress, 0, total, 0, 255));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    error_notification = true;
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if(!MDNS.begin("rollos", WiFi.localIP())){
    Serial.println("MDNS error :(");
  }
  sensors.begin();
  irsend.begin();
  setSyncInterval(300);
  Udp.begin(localPort);
  setSyncProvider(getNtpTime);
  server.on("/", handleRoot);
  server.on("/log", handleLog);
  //server.on("/values", handleTable);	Not yet implemented
  server.onNotFound(handleNotFound);
  server.begin();
  MDNS.addService("http","tcp",80);
	check_month = month(localT());
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); //turn internal LED off (states are inverted) because you can't see it
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(IR_PIN, OUTPUT);
  for(int i = 0; i<255; i++){
    analogWrite(RED_PIN , i);
    analogWrite(GREEN_PIN , i);
    analogWrite(BLUE_PIN , i);
    delay(2);
  }
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
	addToLog("Hochgefahren, Setup beendet.");
}

//********************************regular functions***************************************

void handleRoot() {
  char temp[400];
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  server.send(200, "text/html", root_page);
  server.send(200, "text/html", temp);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

void handleLog() {
  char temp[400];
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  server.send(200, "text/html", log_page);
  server.send(200, "text/html", temp);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

void addToLog(String text) {
	log += "<p>" + timeAsString() + ": " + text + "</p>";
}

String getLog() {
	return log;
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  server.send(404, "text/plain", message);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

uint16_t* translate(bool data[], int nr, bool direction) { //decompresses the timing data for a given group and direction
  uint16_t *real_data = new uint16_t[48];
  real_data[47] = 1000;
  if (!direction) {
    data[1] = 1;
    if (nr == 1) {
      data[21] = 1;
      data[23] = 0;
    } else if (nr == 2 || nr == 3) {
      data[21] = 0;
      data[23] = 1;
    } else if (nr == 5 || nr == 6) {
      data[21] = 0;
      data[23] = 0;
    } else {
      data[21] = 1;
      data[23] = 1;
    }
  }
  for (int i = 0; i < 24; i++) {
    if (data[i] == true) {
      real_data[i * 2] = 1300;
      if (i < 23) {
        real_data[(i * 2) + 1] = 400;
      }
    } else {
      real_data[i * 2] = 400;
      if (i < 23) {
        real_data[(i * 2) + 1] = 1300;
      }
    }
  }
  return real_data;
}

void sendIr(uint16_t data[]) {
  digitalWrite(GREEN_PIN, HIGH);
	for(int i = 0; i < 3; i++){ //sends the data three times just to be sure
		irsend.sendRaw(data , 48, 36);
		delay(20)
	}
  digitalWrite(GREEN_PIN, LOW);
}

int* getWeather() {
  int *weather = new int[3];
  DSW_current *current = new DSW_current;
  DSW_hourly *hourly = new DSW_hourly;
  DSW_daily *daily = new DSW_daily;
  time_t time;
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  dsw.getForecast(current, hourly, daily, api_key, latitude, longitude, units, language);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
  weather[0] = current->precipProbability;
  weather[1] = current->cloudCover;
  weather[2] = current->humidity;
  delete current;
  delete hourly;
  delete daily;
  return weather;
}

String timeAsString(){
	String real_time = String(hour(localT())) + ":" + String(minute(localT())) + ":" + String(second(localT()));
	return real_time;
}

void moveShutter(int nr, bool direction) {
  uint16_t *arr;
	String to_log = "Gruppe " + String(nr);
	if direction = true {
		to_log += " hoch.";
	} 
	else {
		to_log += " runter.";
	}
	addToLog(to_log)
  switch (nr) {
    case 0:
      arr = translate(r1_2, 2, direction);
      sendIr(arr);
      delay(20);
      arr = translate(r1_3, 3, direction);
      sendIr(arr);
      delay(20);
      arr = translate(r1_5, 5, direction);
      sendIr(arr);
      delay(20);
      arr = translate(r1_6, 6, direction);
      sendIr(arr);
    case 1:
      arr = translate(r1_1, 1, direction);
      sendIr(arr);
      delay(20);
      arr = translate(r1_4, 4, direction);
      sendIr(arr);
    case 2:
      arr = translate(r2, BACK_MOST, direction);
      sendIr(arr);
      delay(20);
      arr = translate(r3, BACK_LAST, direction);
      sendIr(arr);
  }
  delete []arr;
}

void controllShutter(bool must_send) {    //updates the states and the global weather array (weather_data)
  if(WiFi.status() == WL_CONNECTED){
		int *temp_weather = getWeather();
  	weather_data[0] = temp_weather[0];
  	weather_data[1] = temp_weather[1];
  	weather_data[2] = temp_weather[2];
  	delete []temp_weather;
	}
	else{
		error_notification = true;
	}
  r_state[1] = scheduleState(1);
  if (weather_data[0] > 20 || weather_data[1] > 60 || weather_data[2] > 90) {
    r_state[1] = true;
		if r_state[1] != last_state[1] {
			addToLog("Gruppe 1 hoch: Nds: "+ weather_data[0] + "&#037, Wolken: " + weather_data[1] + "&#037, Feucht: " + weather_data[2] + "&#037";
  }
  r_state[0] = scheduleState(0);  //decide for normal shutters
  r_state[2] = scheduleState(2);
  for (int i = 0; i < 3; i++) {
    if (r_state[i] != last_state[i] || must_send == true) {
      moveShutter(i, r_state[i]);
      delay(20);
      last_state[i] = r_state[i];
    }
  }
}

bool scheduleState(int shutters) { //calculates the states according to the schedule
  int nr = shutters;
  bool result = true;
  if (shutters == 1) {
    nr = 0;
  }
  else if (shutters == 2) {
    nr = 1;
  }
  if (date != -1 && date != (sizeof(dates) / sizeof(dates[0])) && (times[nr][date][0][0] != 254 || times[nr][date][0][0] != 255) && timeToMins(hour(localT()), minute(localT())) > current_times[nr][0] && timeToMins(hour(localT()), minute(localT())) < current_times[nr][1]) {
    result = false;
  }
  if (times[nr][date][0][0] == 254) {
    result = true;
  }
	else if(times[nr][date][0][0] == 255){
		result = false;
	}
  return result;
}

int getDarksky(int index){
  return weather_data[index];
}

void updateDate(){
   date = -1;
   for (int i = 0; i < 9; i++) {
    if (dates[i][1] < month(localT()) || (dates[i][1] == month(localT()) && dates[i][0] <= day(localT()))) {
      date = i;
    }
   }
}

String showCurrentTimes(int nr){
   int table_nr = 0
      String sentence;
   if (nr != 0) {
     table_nr = nr - 1;
   }
   if (times[table_nr][date][0][0] == 254 || date == -1 || date == 9){
      sentence = "Diese Rollos werden im aktuellen Zeitraum nicht heruntergefahren.";
	 }
   else if (times[table_nr][date][0][0] == 255){
      sentence = "Diese Rollos werden im aktuellen Zeitraum nicht hochgefahren.";
	 }
   else if (nr == 1){
      sentence = "Diese Rollos sind heute planmäßig von " + minsToTime(current_times[table_nr][0]) + " bis " + minsToTime(current_times[table_nr][1]) + " unten.";
	 }
   else{
      sentence = "Diese Rollos sind heute von " + minsToTime(current_times[table_nr][0]) + " bis " + minsToTime(current_times[table_nr][1]) + " unten.";
	 }
}
      
String minsToTime(int _time){
   int _minutes = _time % 60;
   int _hours = (_times - _minutes) / 60;
   String result = "%02d:%02d", _hours, _minutes;
   return result;
}

int timeToMins(byte _hour, byte _minute){
   int minutes_since_0 = _hour * 60;
   minutes_since_0 += _minutes;
   return minutes_since_0;
}

time_t localT(){
	return now() + 3600;
}
							 
int dailyTime(byte nr, bool direction){		//calculates the times the given group has to move in the specified direction on the current day for a smooth transition
   int time_diff;
   int date_diff;
   int og_date_diff;
   int new_time;
	 if(times[nr][date][direction][0] != 254 || times[nr][date][direction][0] != 255){
		 og_date_diff = (dates[date + 1][0] - dates[date][0]) + (dates[date + 1][1] - dates[date][1])*30;
		 date_diff = dates[date + 1][0] - day(localT());
		 date_diff += (dates[date + 1][1] - month(localT())) * 30;
		 time_diff = timeToMins(times[nr][date + 1][direction][0], times[nr][date + 1][direction][1]) - timeToMins(times[nr][date][direction][0], times[nr][date][direction][1]);
		 time_diff -= (date_diff) * (time_diff / og_date_diff);
		 new_time = timeToMins(times[nr][date][direction][0], times[nr][date][direction][1]) + time_diff;
	 }
	 else{
		 new_time = 0;
	 }
   return new_time;
}

char* expressState(int index){
  if(r_state[index]){
    return "Hoch";
  }
  else{
    return "Runter";
  }
}

float getTemperature(){
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);
  if(tempC == DEVICE_DISCONNECTED_C){
    error_notification = true;
  }
  return tempC;
}

String temperatureString(float temp){
  String result;
  if(temp == DEVICE_DISCONNECTED_C){
    result = "ERROR: Kein Temperatursensor erkannt";
  }
  else{
    
    result = String(temp) + "&deg;C";
  }
  return result;
}

void handleError(){
	if(error_notification = true){
		error_blink_state = !error_blink_state;
		digitalWrite(RED_PIN, error_blink_state);
	}
}

time_t getNtpTime() {
  if(WiFi.status() == WL_CONNECTED){
		IPAddress ntpServerIP; // NTP server's ip address

		while (Udp.parsePacket() > 0) ; // discard any previously received packets
		// get a random server from the pool
		digitalWrite(GREEN_PIN, HIGH);
		digitalWrite(BLUE_PIN, HIGH);
		WiFi.hostByName(ntpServerName, ntpServerIP);
		sendNTPpacket(ntpServerIP);
		uint32_t beginWait = millis();
		while (millis() - beginWait < 1500) {
			int size = Udp.parsePacket();
			if (size >= NTP_PACKET_SIZE) {
				Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
				digitalWrite(GREEN_PIN, LOW);
				digitalWrite(BLUE_PIN, LOW);
				unsigned long secsSince1900;
				// convert four bytes starting at location 40 to a long integer
				secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
				secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
				secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
				secsSince1900 |= (unsigned long)packetBuffer[43];
				return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
			}
		}
		return 0; // return 0 if unable to get the time
	}
	else{
		error_notification = true;
		return 0;
	}
}

void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

		 
//********************************Main loop***************************************
void loop() {
	if(month(localT()) != check_month){	//check if its a new month
		digitalWrite(RED_PIN, HIGH);
		digitalWrite(GREEN_PIN, HIGH);
		delay(5000)
		digitalWrite(RED_PIN, LOW);
		digitalWrite(GREEN_PIN, LOW);
		ESP.restart();	//restart every month to prevent problems from building up
	}
	if(day(localT()) != check_day && hour(localT()) >= 9){	//dawn of the first day   ..or any day. Damnit now I have the clock town theme stuck in my head.
    updateDate();
    current_times[0][0] = dailyTime(0, false);
    current_times[0][1] = dailyTime(0, true);
    current_times[1][0] = dailyTime(1, false);
    current_times[1][1] = dailyTime(1, true);
		controllShutter(true);
		check_day = day(localT());
	}
	else{
		controllShutter(false);
	}
	for(int i = 0; i < 600; i++){	//total time of 300000ms = 5 minutes
		last = millis(); //never have to worry about millis() overflow because that happens after ~50 days but the ESP resets every month
  	while (millis() - last < 500){	//timing of the error blinking without needing a second millis comparison
			ArduinoOTA.handle();
    	server.handleClient();
    	delay(1);		//give the ESP some rest. Because the two handle functions also take some time each loop actually takes more than 1ms so the total wait time is also above 5 mins but who cares
   	}
		handleError();
	}
}
